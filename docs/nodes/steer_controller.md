# Steer Controller Node

**Node Name:** `steer_controller`

The Steer Controller node implements closed-loop steering control for the vehicle. It receives steering commands from the Vehicle Controller and steering wheel position feedback from the Sensor Reader node. Using a proportional control algorithm and filtered potentiometer feedback, it continuously adjusts the steering motor until the requested steering position is reached.

The node communicates with the steering H-bridge motor driver over I2C and includes safety monitoring, watchdog protection, hardware diagnostics, and communication error handling.

## Responsibilities

1. **Closed-Loop Steering Control**: Uses steering wheel position feedback from the potentiometer to drive the steering actuator toward the requested position.

2. **Proportional Control**: Calculates motor speed based on the difference between the target steering position and the current steering position.

3. **Exponential Steering Mapping**: Applies a cubic steering curve to improve steering precision around the center position while maintaining full steering range.

4. **Sensor Filtering**: Uses an exponential moving average filter to reduce noise in potentiometer feedback.

5. **H-Bridge Motor Control**: Converts steering corrections into motor direction and speed commands for the steering actuator.

6. **Safety Watchdog**: Automatically stops the steering motor if steering commands are no longer received.

7. **Diagnostics**: Publishes steering targets, actual positions, motor commands, timeout state, and hardware status information.

## Subscribed Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `steer_command` | `amnis_controller/msg/SteerCommand` | Desired steering position command from the Vehicle Controller. |
| `sensor_data` | `amnis_controller/msg/SensorData` | Steering wheel potentiometer feedback from the Sensor Reader node. |

## Published Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `steer_diagnostics` | `std_msgs/msg/String` | Diagnostic information including steering target, actual position, motor direction, speed, and timeout status. |

## Control Strategy

### Steering Setpoint Generation

The incoming steering command is transformed using an exponential curve:

```text
expo_input = steer³
```

This provides:

- Higher precision around the center position.
- Smoother low-speed steering response.
- Full steering authority at large steering inputs.

The resulting value is converted into a target potentiometer position using the configured steering calibration parameters.

### Feedback Filtering

The steering wheel position is measured using a potentiometer connected through the Sensor Reader node.

To reduce sensor noise, the node applies an Exponential Moving Average (EMA) filter:

```text
filtered = α × current + (1 - α) × previous
```

where `α` is configured using the `filter_alpha` parameter.

### Proportional Control

The steering error is calculated as:

```text
error = target_position - actual_position
```

Motor speed is then determined using a proportional gain:

```text
speed = |error| × kp
```

The output is automatically limited to:

- Minimum speed: `20%`
- Maximum speed: `100%`

### Deadzone

A configurable deadzone prevents oscillation when the steering wheel reaches the target position.

If:

```text
|error| < deadzone
```

the steering motor is stopped.

## Steering Direction Logic

| Condition | Direction |
| :-------- | :-------- |
| Error > 0 | Left |
| Error < 0 | Right |
| Error within deadzone | Stop |

The controller continuously adjusts motor direction and speed until the steering wheel reaches the target position.

## Parameters

| Parameter | Type | Default | Description |
| :-------- | :--- | :------ | :---------- |
| `input_topic` | `string` | `'steer_command'` | Input steering command topic. |
| `sensor_topic` | `string` | `'sensor_data'` | Steering feedback topic. |
| `diagnostic_topic` | `string` | `'steer_diagnostics'` | Diagnostic output topic. |
| `i2c_bus` | `int` | `1` | I2C bus used for H-bridge communication. |
| `i2c_address` | `int` | `0x58` | H-bridge I2C address. |
| `max_power` | `int` | `100` | Maximum motor power output. |
| `mock_mode` | `bool` | `False` | Simulates hardware for testing. |
| `pigpio_host` | `string` | `'localhost'` | Hostname of the pigpio daemon. |
| `pigpio_port` | `int` | `8888` | Port used to communicate with pigpio. |
| `command_timeout_sec` | `double` | `0.5` | Timeout before steering motor is stopped. |
| `auto_stop_on_error` | `bool` | `True` | Automatically stop after repeated communication errors. |
| `error_threshold` | `int` | `3` | Number of consecutive errors before protection activates. |
| `update_rate_hz` | `double` | `50.0` | Steering control update frequency. |
| `potmeter_center` | `double` | `0.375` | Potentiometer value corresponding to centered steering. |
| `potmeter_range` | `double` | `0.175` | Distance from center to full steering lock. |
| `kp` | `double` | `700.0` | Proportional control gain. |
| `deadzone` | `double` | `0.01` | Error threshold below which steering stops. |
| `filter_alpha` | `double` | `0.4` | Exponential moving average filter coefficient. |
| `publish_diagnostics` | `bool` | `True` | Enables diagnostic publishing. |
| `log_throttle_sec` | `double` | `1.0` | Time between status log messages. |
| `verbose` | `bool` | `True` | Enables detailed console logging. |

## Safety Features

### Command Timeout Protection

If no steering command is received within `command_timeout_sec`:

- The steering motor is stopped.
- Motor direction is set to zero.
- The controller enters a timeout state.
- Steering resumes only after a new command is received.

### Communication Error Protection

The node continuously monitors H-bridge communication errors.

If repeated communication failures occur:

- Errors are counted and logged.
- Automatic stop protection can be activated using `auto_stop_on_error`.

### Feedback Validation

Steering control only operates when both:

- A steering command has been received.
- Valid steering position feedback is available.

This prevents uncontrolled steering motion during startup or sensor failures.

## Diagnostics

The diagnostic topic includes:

- Target steering position
- Actual steering position
- Motor direction
- Motor speed
- Command timeout status

These diagnostics can be viewed in the dashboard or ROS tools to verify steering performance and assist with controller tuning.

## Shutdown Behavior

When the node shuts down:

1. A final stop command is sent to the steering motor.
2. The H-bridge connection is closed safely.
3. ROS timers and publishers are cleaned up.

This ensures the steering actuator cannot continue moving after the node exits or the system is powered down.