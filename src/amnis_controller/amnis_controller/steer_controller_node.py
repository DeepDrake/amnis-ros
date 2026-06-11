#!/usr/bin/env python3
"""Steer controller node for controlling H-bridge motor driver via I2C using Closed-Loop P-Control.

This node subscribes to SteerCommand messages (joystick) AND SensorData (potmeter).
It uses an exponential curve and a Proportional (P) controller to smoothly drive
the steering wheel to the exact target position, just like a racegame force-feedback wheel.

Author: amnis_controller
Target: Jetson Orin (or any Linux system with I2C)
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

# Importeer zowel het stuur-commando als de sensor-data
from amnis_controller.msg import SteerCommand, SensorData
from amnis_controller.drivers import HBridgeDriver


class SteerControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('steer_controller')

        # Declareer de topics
        self.declare_parameter('input_topic', 'steer_command')
        self.declare_parameter('sensor_topic', 'sensor_data')
        self.declare_parameter('diagnostic_topic', 'steer_diagnostics')
        
        # Hardware configuration
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x58)
        self.declare_parameter('max_power', 100)
        self.declare_parameter('mock_mode', False)
        
        # Pigpio configuration
        self.declare_parameter('pigpio_host', 'localhost')
        self.declare_parameter('pigpio_port', 8888)
        
        # Safety parameters
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('auto_stop_on_error', True)
        self.declare_parameter('error_threshold', 3)
        
        # --- RACEGAME CONTROL PARAMETERS ---
        self.declare_parameter('update_rate_hz', 50.0)
        
        # Potmeter waarden (Gebaseerd op: Links=0.55, Rechts=0.20)
        self.declare_parameter('potmeter_center', 0.375)
        self.declare_parameter('potmeter_range', 0.175) # (0.55 - 0.375)
        
        # P-Controller & Filter instellingen
        self.declare_parameter('kp', 700.0)             # Snelheid multiplier
        self.declare_parameter('deadzone', 0.01)        # Hoe strak stopt hij op het doel?
        self.declare_parameter('filter_alpha', 0.4)     # Ruis-filter (0.1 = traag/glad, 1.0 = direct/ruis)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)
        
        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        sensor_topic = self.get_parameter('sensor_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        max_power = self.get_parameter('max_power').value
        mock_mode = self.get_parameter('mock_mode').value
        pigpio_host = self.get_parameter('pigpio_host').value
        pigpio_port = self.get_parameter('pigpio_port').value
        
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        auto_stop_on_error = self.get_parameter('auto_stop_on_error').value
        error_threshold = self.get_parameter('error_threshold').value
        update_rate = self.get_parameter('update_rate_hz').value
        
        self.pot_center = self.get_parameter('potmeter_center').value
        self.pot_range = self.get_parameter('potmeter_range').value
        self.kp = self.get_parameter('kp').value
        self.deadzone = self.get_parameter('deadzone').value
        self.alpha = self.get_parameter('filter_alpha').value
        
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Initialize hardware driver
        self.driver = HBridgeDriver(
            i2c_bus=i2c_bus,
            i2c_address=i2c_address,
            max_power=max_power,
            mock_mode=mock_mode,
            pigpio_host=pigpio_host,
            pigpio_port=pigpio_port,
            auto_stop_on_error=auto_stop_on_error,
            error_threshold=error_threshold,
        )
        
        if not self.driver.is_connected():
            self.get_logger().error("Failed to initialize H-bridge driver! Running in degraded mode.")
        
        # State tracking
        self._last_command: SteerCommand | None = None
        self._last_command_time: Time | None = None
        self._filtered_steering: float | None = None
        
        self._current_direction = 0
        self._current_speed = 0
        self._is_timed_out = False
        self._last_error_count = 0
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            SteerCommand, input_topic, self.steer_command_callback, 10
        )
        
        self.sensor_sub = self.create_subscription(
            SensorData, sensor_topic, self.sensor_data_callback, 10
        )
        
        # Diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(String, diagnostic_topic, 10)
        
        # Timers
        self.update_timer = self.create_timer(1.0 / update_rate, self.update_callback)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.last_log_time = self.get_clock().now()
        
        if self.verbose:
            self.get_logger().info("Steer controller (Closed-Loop) initialized.")

    def steer_command_callback(self, msg: SteerCommand) -> None:
        """Handle incoming joystick commands."""
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False

    def sensor_data_callback(self, msg: SensorData) -> None:
        """Handle incoming potmeter data and apply Low-Pass filter."""
        raw_steer = msg.steering_wheel
        
        if self._filtered_steering is None:
            self._filtered_steering = raw_steer
        else:
            # Exponential Moving Average Filter
            self._filtered_steering = (self.alpha * raw_steer) + ((1.0 - self.alpha) * self._filtered_steering)

    def update_callback(self) -> None:
        """Calculate PID and send motor commands at 50Hz."""
        if self._last_command is None or self._filtered_steering is None:
            return
        
        # 1. Bepaal Target met Expo-Curve (Racegame feel)
        joystick_x = self._last_command.steer
        expo_input = joystick_x ** 3
        
        # Als joystick positief is (links sturen), verhogen we de potmeterwaarde
        target_potmeter = self.pot_center - (expo_input * self.pot_range)
        
        # 2. Bereken de Error
        error = target_potmeter - self._filtered_steering
        
        # 3. P-Controller & Deadzone
        if abs(error) < self.deadzone:
            speed = 0
            direction = 0
        else:
            # Reken de snelheid uit via de Proportional gain (Kp)
            raw_speed = abs(error) * self.kp
            speed = int(max(20.0, min(100.0, raw_speed)))  # Begrens tussen 20 en 100%
            
            # 4. Bepaal de richting
            # Is het doel hoger dan de huidige positie? Stuur Links (potmeterwaarde stijgt)
            if error > 0:
                direction = 1  # 1 = Left
            else:
                direction = 2  # 2 = Right
                
        # (Hardware Check: Draait het stuur de verkeerde kant op en blijft hij in 
        # de hoek duwen? Verwissel dan hierboven de '1' en de '2'!)
        
        # 5. Stuur commando naar de driver
        success = self.driver.set_direction_speed(direction, speed)
        
        if success:
            self._current_direction = direction
            self._current_speed = speed
        
        # Error handling & Logging
        current_errors = self.driver.get_error_count()
        if current_errors > self._last_error_count:
            consecutive = self.driver.get_consecutive_errors()
            if consecutive >= 3:
                self.get_logger().warning(f"I2C errors: {consecutive} consecutive")
        self._last_error_count = current_errors
        
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                dir_name = {0: "STOP", 1: "LEFT", 2: "RIGHT"}.get(direction, "?")
                self.get_logger().info(
                    f"Target: {target_potmeter:.3f} | Actual: {self._filtered_steering:.3f} | "
                    f"Err: {error:.3f} -> Dir: {direction}({dir_name}), Spd: {speed}%"
                )
                self.last_log_time = now
        
        if self.publish_diagnostics:
            self._publish_diagnostics(target_potmeter, self._filtered_steering, direction, speed, success)

    def watchdog_callback(self) -> None:
        """Stops the motor if Xbox controller disconnects."""
        if self._last_command_time is None: return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(f"Command timeout ({time_since_command:.2f}s) - stopping motor!")
            self.driver.stop()
            self._current_direction = 0
            self._current_speed = 0
            self._is_timed_out = True

    def _publish_diagnostics(self, target, actual, direction, speed, success) -> None:
        msg = String()
        dir_name = {0: "STOP", 1: "LEFT", 2: "RIGHT"}.get(direction, "UNKNOWN")
        msg.data = (
            f"target={target:.3f}, actual={actual:.3f}, "
            f"direction={direction}({dir_name}), speed={speed}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.driver.stop()
        self.driver.close()
        return super().destroy_node()

def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SteerControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == '__main__':
    main()
