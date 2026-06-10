# PID Steering Controller Node

**Node Name:** `pid_steer_controller`

The PID Steering Controller node provides closed-loop steering control using a PID (Proportional-Integral-Derivative) algorithm. It receives steering commands from the Joystick Normalizer node, compares them with steering position feedback from the potentiometer sensor, and generates corrected steering commands for the Vehicle Controller. The node can operate in either PID mode for precise steering control or passthrough mode when feedback is unavailable.

## Responsibilities

1. **PID Steering Control**: Calculates corrected steering commands using a PID algorithm based on the difference between the desired steering position and actual steering position.

2. **Sensor Feedback Processing**: Receives steering wheel position feedback from the sensor system and maps potentiometer values to the steering range.

3. **Passthrough Mode**: Automatically forwards joystick steering commands unchanged when PID control is disabled or feedback data is unavailable.

4. **Anti-Windup Protection**: Prevents excessive integral accumulation to improve controller stability and responsiveness.

5. **Derivative Filtering**: Applies a low-pass filter to the derivative term to reduce sensitivity to sensor noise.

6. **Diagnostics and Tuning Support**: Publishes detailed PID state information including error values, PID terms, output values, and feedback status.

## Subscribed Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `normalized_joystick` | `amnis_controller/msg/JoystickCommand` | Normalized joystick commands used as steering setpoints. |
| `sensor_data` | `amnis_controller/msg/SensorData` | Sensor feedback containing steering wheel position and calibration status. |

## Published Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `vehicle_controller_command` | `amnis_controller/msg/JoystickCommand` | PID-corrected vehicle control commands. |
| `pid_diagnostics` | `std_msgs/msg/String` | Diagnostic information for monitoring and PID tuning. |

## Control Modes

### PID Enabled

When PID control is enabled and valid sensor feedback is available:

- Steering commands are treated as position setpoints.
- Potentiometer feedback is used as the actual steering position.
- The PID controller computes a corrected steering output.
- Throttle, brake, gear, and command fields are passed through unchanged.

### Passthrough Mode

The node automatically switches to passthrough mode when:

- PID control is disabled.
- No sensor feedback has been received.
- Sensor feedback has timed out.
- Potentiometer values are invalid.

In passthrough mode, steering commands are forwarded directly without modification.

## PID Algorithm

The controller calculates steering output using:

```text
Output = P + I + D
```

Where:

```text
P = Kp × Error
I = Ki × Integral(Error)
D = Kd × Derivative(Error)
```

The error is calculated as:

```text
Error = Setpoint - Feedback
```

The final output is limited to the configured steering range.

## Potentiometer Mapping

The node converts potentiometer readings from the sensor system into steering values.

| Potentiometer Value | Steering Value |
| :----------------- | :------------- |
| `pot_min` | `-1.0` (Full Left) |
| `pot_center` | `0.0` (Center) |
| `pot_max` | `1.0` (Full Right) |

A configurable deadzone around the center position prevents jitter when the steering wheel is near the center.

## Subsystem Data Flow

```text
Joystick Normalizer
          ↓
PID Steering Controller
          ↑
     Sensor Data
          ↓
Vehicle Controller
```

The PID controller sits between the joystick normalizer and vehicle controller, modifying only the steering command while passing all other commands through unchanged.

## Parameters

| Parameter | Type | Default | Description |
| :-------- | :--- | :------ | :---------- |
| `input_topic` | `string` | `'normalized_joystick'` | Input topic containing joystick commands. |
| `output_topic` | `string` | `'vehicle_controller_command'` | Output topic for corrected commands. |
| `feedback_topic` | `string` | `'sensor_data'` | Sensor feedback topic. |
| `diagnostic_topic` | `string` | `'pid_diagnostics'` | Diagnostic output topic. |
| `kp` | `double` | `1.5` | Proportional gain. |
| `ki` | `double` | `0.2` | Integral gain. |
| `kd` | `double` | `0.1` | Derivative gain. |
| `integral_limit` | `double` | `0.5` | Maximum allowed integral value. |
| `output_limit` | `double` | `1.0` | Maximum steering output. |
| `max_integral_time_sec` | `double` | `2.0` | Maximum time allowed for integral accumulation. |
| `pot_min` | `double` | `0.2` | Potentiometer value corresponding to full left steering. |
| `pot_center` | `double` | `0.35` | Potentiometer value corresponding to centered steering. |
| `pot_max` | `double` | `0.5` | Potentiometer value corresponding to full right steering. |
| `pot_deadzone` | `double` | `0.02` | Deadzone around the center position. |
| `enable_pid` | `bool` | `True` | Enables PID steering control. |
| `update_rate_hz` | `double` | `10.0` | PID update frequency. |
| `feedback_timeout_sec` | `double` | `0.5` | Maximum allowed age of sensor feedback. |
| `derivative_filter_alpha` | `double` | `0.1` | Low-pass filter coefficient for derivative smoothing. |
| `mock_mode` | `bool` | `False` | Simulates steering feedback for testing without hardware. |
| `mock_time_constant` | `double` | `0.3` | Response speed used in mock mode simulation. |
| `verbose` | `bool` | `True` | Enables detailed logging. |
| `log_throttle_sec` | `double` | `1.0` | Time between status log messages. |
| `publish_diagnostics` | `bool` | `True` | Enables diagnostic publishing. |

## Safety Features

### Feedback Timeout Protection

If sensor feedback is not received within `feedback_timeout_sec`, the node automatically switches to passthrough mode.

### Integral Windup Protection

The integral term is limited using:

- Integral value clamping.
- Maximum accumulation time limits.
- Automatic reset when PID becomes inactive.

### Derivative Noise Filtering

A configurable low-pass filter reduces noise in the derivative term, improving controller stability.

### Automatic Mode Switching

The controller automatically falls back to passthrough mode whenever valid feedback is unavailable, ensuring the vehicle remains controllable even if the steering sensor fails.

## Diagnostics

The diagnostic topic includes:

- Steering setpoint
- Steering feedback
- Raw potentiometer value
- PID error
- Proportional term (P)
- Integral term (I)
- Derivative term (D)
- Controller output
- PID active status
- Passthrough reason
- Calibration status
- Mock mode status

## Shutdown Behavior

When the node shuts down:

1. PID computation stops.
2. ROS publishers and subscribers are cleaned up.
3. Diagnostic publishing is terminated.
4. The node exits gracefully without affecting downstream controllers.

This ensures safe shutdown while allowing other vehicle control nodes to terminate independently.