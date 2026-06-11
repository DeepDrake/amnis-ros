#!/usr/bin/env python3
"""Brake controller node for controlling EHB (Electro-Hydraulic Brake) via CAN bus.

This node subscribes to BrakeCommand messages and translates them into
CAN commands for the electro-hydraulic brake system. Hardware override is 
handled natively by the EHBDriver.

Author: amnis_controller
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from amnis_controller.msg import BrakeCommand

from amnis_controller.drivers import EHBDriver


class BrakeControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('brake_controller')

        self.declare_parameter('input_topic', 'brake_command')
        self.declare_parameter('diagnostic_topic', 'brake_diagnostics')
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('pressure_scale', 40.0)
        self.declare_parameter('mock_mode', False)  
        self.declare_parameter('command_timeout_sec', 0.5) 
        self.declare_parameter('deadzone', 0.01) 
        self.declare_parameter('update_rate_hz', 10.0) 
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)
        
        # Override Parameters
        self.declare_parameter('pedal_can_id', 0x180)
        self.declare_parameter('pedal_byte_index', 1)
        self.declare_parameter('pedal_threshold', 12)
        
        input_topic = self.get_parameter('input_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        can_channel = self.get_parameter('can_channel').value
        can_interface = self.get_parameter('can_interface').value
        pressure_scale = self.get_parameter('pressure_scale').value
        mock_mode = self.get_parameter('mock_mode').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        self.deadzone = self.get_parameter('deadzone').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        pedal_can_id = self.get_parameter('pedal_can_id').value
        pedal_byte_index = self.get_parameter('pedal_byte_index').value
        pedal_threshold = self.get_parameter('pedal_threshold').value
        
        self.driver = EHBDriver(
            can_channel=can_channel,
            can_interface=can_interface,
            pressure_scale=pressure_scale,
            mock_mode=mock_mode,
            pedal_can_id=pedal_can_id,
            pedal_byte_index=pedal_byte_index,
            pedal_threshold=pedal_threshold
        )
        
        if not self.driver.is_connected():
            self.get_logger().error("Failed to initialize EHB driver! Running in degraded mode.")
        
        self._last_command: BrakeCommand | None = None
        self._last_command_time: Time | None = None
        self._current_pressure = 0.0
        self._is_timed_out = False
        self._was_overridden = False
        
        self.subscription = self.create_subscription(
            BrakeCommand, input_topic, self.brake_command_callback, 10
        )
        
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(String, diagnostic_topic, 10)
        
        self.update_timer = self.create_timer(1.0 / update_rate, self.update_callback)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.last_log_time = self.get_clock().now()
    
    def brake_command_callback(self, msg: BrakeCommand) -> None:
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False
        
        brake = msg.brake
        if abs(brake) < self.deadzone: brake = 0.0
        brake = max(0.0, min(1.0, brake))
        
        success = self.driver.set_pressure(brake)
        if success: self._current_pressure = brake
    
    def update_callback(self) -> None:
        # Check override status logging
        is_overridden = self.driver.is_override_active()
        
        if is_overridden and not self._was_overridden:
            raw_val = self.driver.get_physical_pressure_raw()
            self.get_logger().warn(f"HARDWARE OVERRIDE ACTIEF! Mens grijpt in. Raw Pedaalwaarde: {raw_val}")
        elif not is_overridden and self._was_overridden:
            self.get_logger().info("Pedaal losgelaten. Terug naar Autonome/Xbox modus.")
            
        self._was_overridden = is_overridden

        if self._last_command is None and not is_overridden: return
        
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                time_str = f"{self.driver.get_time_since_last_message():.3f}s" if self.driver.get_time_since_last_message() else "None"
                mode_str = "MANUAL (OVERRIDE)" if is_overridden else "AUTO"
                
                # Als overridden, is de actuele druk die van het pedaal (via driver).
                log_pressure = (self.driver.get_physical_pressure_raw() / 255.0) if is_overridden else self._current_pressure
                
                self.get_logger().info(
                    f"Brake [{mode_str}]: pressure={log_pressure:.3f} | "
                    f"CAN OK: {self.driver.has_can_communication()} | Last msg: {time_str} ago"
                )
                self.last_log_time = now
        
        if self.publish_diagnostics:
            self._publish_diagnostics()
    
    def watchdog_callback(self) -> None:
        if not self.driver.has_can_communication():
            self.get_logger().error("CAN communication lost!")
            self.driver.stop()
            self._current_pressure = 0.0
            return
            
        # SAFETY: Niet afbreken op timeout als de mens fysiek remt!
        if self.driver.is_override_active():
            return
            
        if self._last_command_time is None: return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(f"Command timeout ({time_since_command:.2f}s) - releasing brake!")
            self.driver.stop()
            self._current_pressure = 0.0
            self._is_timed_out = True
    
    def _publish_diagnostics(self) -> None:
        msg = String()
        is_overridden = self.driver.is_override_active()
        pub_pressure = (self.driver.get_physical_pressure_raw() / 255.0) if is_overridden else self._current_pressure
        
        time_str = f"{self.driver.get_time_since_last_message():.3f}" if self.driver.get_time_since_last_message() else "None"
        
        msg.data = (
            f"brake_pressure={pub_pressure:.3f}, "
            f"mode={'MANUAL' if is_overridden else 'AUTO'}, "
            f"can_ok={self.driver.has_can_communication()}, "
            f"time_since_last_msg={time_str}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        self.driver.stop()
        self.driver.close()
        return super().destroy_node()

def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BrakeControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
