#!/usr/bin/env python3
"""Brake controller node for controlling EHB (Electro-Hydraulic Brake) via CAN bus.

This node subscribes to BrakeCommand messages and translates them into
CAN commands for the electro-hydraulic brake system. It includes safety features,
error handling, and diagnostics.

Author: amnis_controller
Target: Systems with CAN bus support (socketcan on Linux)
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

# AANGEPAST: Importeer ook VehicleState voor de 'Heartbeat' fix
from amnis_controller.msg import BrakeCommand, VehicleState
from amnis_controller.drivers import EHBDriver


class BrakeControllerNode(Node):
    """ROS2 node for brake control via EHB system over CAN bus."""

    def __init__(self) -> None:
        """Initialize the brake controller node."""
        super().__init__('brake_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'brake_command')
        self.declare_parameter('diagnostic_topic', 'brake_diagnostics')
        self.declare_parameter('vehicle_state_topic', 'vehicle_state') # NIEUW
        
        # Hardware configuration
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('pressure_scale', 40.0)
        self.declare_parameter('mock_mode', False)  # For testing without hardware
        
        # Safety parameters
        self.declare_parameter('command_timeout_sec', 0.5)  # Release brake if no command
        self.declare_parameter('deadzone', 0.01)  # Ignore small brake commands
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 10.0)  # Status update rate
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)  # Enable/disable info logging
        
        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        vehicle_state_topic = self.get_parameter('vehicle_state_topic').value # NIEUW
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
        
        # Initialize hardware driver
        self.driver = EHBDriver(
            can_channel=can_channel,
            can_interface=can_interface,
            pressure_scale=pressure_scale,
            mock_mode=mock_mode
        )
        
        if not self.driver.is_connected():
            self.get_logger().error(
                "Failed to initialize EHB driver! Running in degraded mode."
            )
        
        # State tracking
        self._last_command: BrakeCommand | None = None
        self._last_command_time: Time | None = None
        self._current_pressure = 0.0
        self._is_timed_out = False
        self._current_mode = None # NIEUW: Houdt bij in welke stand we rijden
        
        # Create subscriber for brake commands
        self.subscription = self.create_subscription(
            BrakeCommand,
            input_topic,
            self.brake_command_callback,
            10
        )
        
        # NIEUW: Create subscriber voor de state machine (om EHB override te sturen)
        self.state_subscription = self.create_subscription(
            VehicleState,
            vehicle_state_topic,
            self.vehicle_state_callback,
            10
        )
        
        # Create diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(
                String,
                diagnostic_topic,
                10
            )
        
        # Create update timer for periodic status updates and diagnostics
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(
            update_period,
            self.update_callback
        )
        
        # Create watchdog timer for safety timeout
        self.watchdog_timer = self.create_timer(
            0.1,  # Check every 100ms
            self.watchdog_callback
        )
        
        # Logging timer
        self.last_log_time = self.get_clock().now()
        
        if self.verbose:
            self.get_logger().info(
                f"Brake controller initialized: "
                f"topic={input_topic}, "
                f"can_channel={can_channel}, "
                f"can_interface={can_interface}, "
                f"mock={mock_mode}"
            )
    
    # NIEUW: Functie om de state op te vangen en naar de driver te sturen
    def vehicle_state_callback(self, msg: VehicleState) -> None:
        """Pas het gedrag van de EHB driver aan op basis van de state."""
        if msg.state != self._current_mode:
            self._current_mode = msg.state
            
            if msg.state == 'EXTERNAL':
                # Autonoom = Jetson bestuurt, ECU negeert pedaal
                self.driver.set_autonomous_mode(True)
                if self.verbose:
                    self.get_logger().info("EXTERNAL mode: EHB CAN override ingeschakeld (controller actief)")
            else:
                # MANUAL / IMMOBILIZED / EHB_ERROR = ECU leest fysiek pedaal uit
                self.driver.set_autonomous_mode(False)
                if self.verbose:
                    self.get_logger().info(f"{msg.state} mode: EHB CAN override uitgeschakeld (pedaal actief)")
    
    def brake_command_callback(self, msg: BrakeCommand) -> None:
        """Handle incoming brake commands."""
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False
        
        # Process brake command immediately
        brake = msg.brake
        
        # Apply deadzone
        if abs(brake) < self.deadzone:
            brake = 0.0
        
        # Clamp to valid range [0.0, 1.0]
        brake = max(0.0, min(1.0, brake))
        
        # Send to hardware
        success = self.driver.set_pressure(brake)
        
        if success:
            self._current_pressure = brake
    
    def update_callback(self) -> None:
        """Periodic update for status monitoring and diagnostics."""
        # Check if we have a command
        if self._last_command is None:
            return
        
        pressure = self._current_pressure
        
        # Periodic logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                time_since_msg = self.driver.get_time_since_last_message()
                time_str = f"{time_since_msg:.3f}s" if time_since_msg is not None else "None"
                self.get_logger().info(
                    f"Brake: pressure={pressure:.3f} | "
                    f"CAN OK: {self.driver.has_can_communication()} | "
                    f"Last msg: {time_str} ago"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(pressure)
    
    def watchdog_callback(self) -> None:
        """Safety watchdog - releases brake if no command received recently."""
        if self._last_command_time is None:
            return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        # Check command timeout
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(
                f"Command timeout ({time_since_command:.2f}s) - releasing brake!"
            )
            self.driver.stop()
            self._current_pressure = 0.0
            self._is_timed_out = True
        
        # Check CAN communication - simple: did we receive ANY message in last 0.5s?
        if not self.driver.has_can_communication():
            time_since_msg = self.driver.get_time_since_last_message()
            if time_since_msg is not None:
                self.get_logger().error(
                    f"CAN communication lost! No messages for {time_since_msg:.2f}s"
                )
            else:
                self.get_logger().error(
                    "CAN communication lost! No messages received since startup"
                )
            # Emergency stop
            self.driver.stop()
            self._current_pressure = 0.0
    
    def _publish_diagnostics(self, pressure: float) -> None:
        """Publish diagnostic information."""
        msg = String()
        time_since_msg = self.driver.get_time_since_last_message()
        
        # Format time_since_msg properly
        if time_since_msg is not None:
            time_str = f"{time_since_msg:.3f}"
        else:
            time_str = "None"
        
        # AANGEPAST: Voeg de huidige mode en override info toe aan de diagnostiek
        msg.data = (
            f"brake_pressure={pressure:.3f}, "
            f"mode={self._current_mode}, "
            f"can_ok={self.driver.has_can_communication()}, "
            f"time_since_last_msg={time_str}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed."""
        if self.verbose:
            self.get_logger().info("Shutting down brake controller...")
        
        # Release brake before shutdown
        self.driver.stop()
        
        # Close hardware connection
        self.driver.close()
        
        # Call parent cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable."""
    rclpy.init(args=args)
    node = BrakeControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()