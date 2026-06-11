"""
This launch file starts:
0. joy_node - Publishes raw joystick data from hardware device
1. joystick_normalizer_node - Normalizes joystick inputs
2. vehicle_controller_node - Manages vehicle state machine and command filtering
3. steer_controller_node - Controls steering motor via I2C H-bridge
4. brake_controller_node - Controls EHB brake system via CAN bus
5. powertrain_controller_node - Controls throttle via PWM on GPIO
6. sensor_reader_node - Reads gas pedal and steering wheel from ADC
7. topic_aggregator_node - Bridges ROS topics to a WebSocket frontend
8. Firefox browser - Opens dashboard in fullscreen/kiosk mode
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with joystick, controller, and steer nodes."""
    
    # game_controller_node - reads from joystick hardware
    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[{
            'device_id': 0,  # Usually 0 for /dev/input/js0
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,  # Hz
        }]
    )
    
    # Joystick normalizer node
    joystick_node = Node(
        package='amnis_controller',
        executable='joystick_normalizer_node',
        name='joystick_normalizer',
        output='screen',
        parameters=[{
            'input_topic': '/joy',
            'output_topic': 'vehicle_controller_command',
            'trigger_axes': [2],
            'deadzone': 0.05,
            'log_throttle_sec': 0.5,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Vehicle controller node with state machine
    controller_node = Node(
        package='amnis_controller',
        executable='vehicle_controller_node',
        name='vehicle_controller',
        output='screen',
        parameters=[{
            'input_topic': 'vehicle_controller_command',
            'sensor_topic': 'sensor_data',
            'powertrain_topic': 'powertrain_command',
            'steer_topic': 'steer_command',
            'brake_topic': 'brake_command',
            'vehicle_state_topic': 'vehicle_state',
            'mode_command_topic': 'mode_command',
            'safety_button': False,
            'mode_button': True,
            'enable_external_mode_control': True,
            'external_mode_pin': 4,
            'pigpio_host': '192.168.10.2',
            'pigpio_port': 8888,
            'mock_mode': False,
            'enable_gas_override': True,
            'gas_override_raw_min': 500,
            'gas_override_raw_max': 1500,
            'verbose_override': True,
            'log_throttle_sec': 0.5,
            'verbose': False,
        }]
    )
    
    # Steer controller node
    steer_controller_node = Node(
        package='amnis_controller',
        executable='steer_controller_node',
        name='steer_controller',
        output='screen',
        parameters=[{
            'input_topic': 'steer_command',
            'diagnostic_topic': 'steer_diagnostics',
            'i2c_bus': 1,
            'i2c_address': 0x58,
            'max_power': 100,
            'pigpio_host': '192.168.10.2',
            'pigpio_port': 8888,
            'mock_mode': False,
            'command_timeout_sec': 0.5,
            'deadzone': 0.05,
            'update_rate_hz': 20.0,
            'steer_to_power_scale': 100.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,
        }]
    )
    
    # Brake controller node - Controls EHB via CAN bus AND Hardware Override
    brake_controller_node = Node(
        package='amnis_controller',
        executable='brake_controller_node',
        name='brake_controller',
        output='screen',
        parameters=[{
            'input_topic': 'brake_command',
            'diagnostic_topic': 'brake_diagnostics',
            'can_channel': 'can1',
            'can_interface': 'socketcan',
            'pressure_scale': 40.0,
            'pedal_can_id': 0x180,
            'pedal_byte_index': 1,
            'pedal_threshold': 12,
            'mock_mode': False,
            'command_timeout_sec': 0.5,
            'deadzone': 0.01,
            'update_rate_hz': 10.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,
        }]
    )
    
    # Powertrain controller node
    powertrain_controller_node = Node(
        package='amnis_controller',
        executable='powertrain_controller_node',
        name='powertrain_controller',
        output='screen',
        parameters=[{
            'input_topic': 'powertrain_command',
            'diagnostic_topic': 'powertrain_diagnostics',
            'pwm_pin': 22,
            'pwm_frequency': 1000,
            'max_throttle': 1.0,
            'pigpio_host': '192.168.10.2',
            'pigpio_port': 8888,
            'enable_transmission_control': True,
            'disable_neutral_pin': 12,
            'enable_reverse_pin': 5,
            'mock_mode': False,
            'command_timeout_sec': 0.5,
            'deadzone': 0.01,
            'update_rate_hz': 20.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,
        }]
    )
    
    # Sensor reader node
    sensor_reader_node = Node(
        package='amnis_controller',
        executable='sensor_reader_node',
        name='sensor_reader',
        output='screen',
        parameters=[{
            'output_topic': 'sensor_data',
            'diagnostic_topic': 'sensor_diagnostics',
            'i2c_bus': 1,
            'i2c_address': 0x48,
            'pigpio_host': '192.168.10.2',
            'pigpio_port': 8888,
            'mock_mode': False,
            'gas_pedal_min': 0,
            'gas_pedal_max': 4093,
            'steering_wheel_min': 0,
            'steering_wheel_max': 2047,
            'auto_calibrate': False,
            'calibration_duration_sec': 10.0,
            'update_rate_hz': 10.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': True,
        }]
    )
    
    # Topic aggregator node
    aggregator_node = Node(
        package='amnis_controller',
        executable='topic_aggregator_node',
        name='topic_aggregator',
        output='screen',
        parameters=[{
            'topic_poll_interval': 2.0,
            'include_hidden_topics': False,
            'ignored_topics': ['/parameter_events', '/rosout'],
            'websocket_host': '0.0.0.0',
            'websocket_port': 8765,
            'update_frequency_hz': 30.0,
        }]
    )
    
    # Firefox browser
    workspace_dir = Path(os.getcwd())
    dashboard_path = workspace_dir / 'dashboard.html'
    dashboard_url = f'file://{dashboard_path.absolute()}'
    
    firefox_process = ExecuteProcess(
        cmd=['firefox', '--kiosk', dashboard_url],
        output='screen',
        name='firefox_dashboard'
    )

    return LaunchDescription([
        game_controller_node,
        joystick_node,
        controller_node,
        steer_controller_node,
        brake_controller_node,
        powertrain_controller_node,
        sensor_reader_node,
        aggregator_node,
        firefox_process,
    ])
