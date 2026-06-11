

# Steer Controller Node

**Node Name:** `steer_controller`

The Steer Controller node manages the vehicle's steering actuator. It converts high-level steering commands into low-level motor control signals and communicates with the steering H-bridge motor driver over I2C. The node provides safety monitoring, command validation, hardware diagnostics, and automatic shutdown protection.

## Responsibilities

1. **Steering Command Processing**: Receives normalized steering commands and converts them into motor direction and speed values for the steering actuator.

2. **I2C Hardware Communication**: Interfaces with the H-bridge motor driver over I2C to control steering motor movement.

3. **Safety Watchdog**: Monitors incoming steering commands. If no command is received within the timeout period (default 0.5s), the steering motor is stopped automatically.

4. **Command Validation**: Applies a deadzone to filter small steering inputs caused by joystick drift or signal noise.

5. **Error Handling**: Monitors communication errors with the H-bridge driver and can automatically stop steering operation after repeated failures.

6. **Diagnostics**: Publishes real-time status information including steering commands, motor state, connection health, timeout status, and communication error counts.

## Subscribed Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `steer_command` | `amnis_controller/msg/SteerCommand` | Input command containing the desired steering value in the range `[-1.0, 1.0]`. |

## Published Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `steer_diagnostics` | `std_msgs/msg/String` | Diagnostic information including steering state, connection status, timeout state, and error counters. |

## Steering Command Mapping

The node converts incoming steering values into motor direction and speed commands.

### Steering (`steer`)

| Input Range | Direction | Motor Direction |
| :---------- | :-------- | :-------------- |
| `> 0.0` | Right | `2` |
| `< 0.0` | Left | `1` |
| `= 0.0` | Stop | `0` |

### Speed Calculation

Motor speed is calculated as:

```text
speed = abs(steer) * steer_to_power_scale
```

With the default scale factor of `100.0`, steering values are mapped directly to motor output percentage:

| Steer Value | Motor Speed |
| :---------- | :---------- |
| `0.25` | `25%` |
| `0.50` | `50%` |
| `1.00` | `100%` |

## Parameters

| Parameter | Type | Default | Description |
| :-------- | :--- | :------ | :---------- |
| `input_topic` | `string` | `'steer_command'` | Input topic name. |
| `diagnostic_topic` | `string` | `'steer_diagnostics'` | Diagnostic output topic. |
| `i2c_bus` | `int` | `1` | I2C bus used for H-bridge communication. |
| `i2c_address` | `int` | `0x58` | I2C address of the H-bridge driver. |
| `max_power` | `int` | `100` | Maximum motor power output. |
| `mock_mode` | `bool` | `False` | Simulates hardware for testing without a physical steering controller. |
| `pigpio_host` | `string` | `'localhost'` | Hostname of the pigpio daemon. |
| `pigpio_port` | `int` | `8888` | Port used to communicate with pigpio. |
| `command_timeout_sec` | `double` | `0.5` | Safety timeout; stops steering if exceeded. |
| `deadzone` | `double` | `0.05` | Steering inputs below this threshold are treated as zero. |
| `auto_stop_on_error` | `bool` | `True` | Automatically stop steering after excessive communication errors. |
| `error_threshold` | `int` | `3` | Number of consecutive errors before triggering an automatic stop. |
| `update_rate_hz` | `double` | `50.0` | Frequency of hardware updates in Hz. |
| `steer_to_power_scale` | `double` | `100.0` | Scaling factor used to convert steering commands into motor power. |
| `publish_diagnostics` | `bool` | `True` | Enables diagnostic message publishing. |
| `log_throttle_sec` | `double` | `1.0` | Time between status log messages. |
| `verbose` | `bool` | `True` | Enables detailed console logging. |

## Safety Features

### Command Timeout Protection

If no steering command is received within `command_timeout_sec`, the node:

- Stops the steering motor.
- Sets motor direction to `0` (STOP).
- Marks the controller as timed out.
- Waits for a new command before resuming operation.

### Deadzone Filtering

Steering values whose magnitude is below the configured deadzone are treated as zero to prevent unintended steering movements.

### Automatic Error Protection

The node continuously monitors H-bridge communication errors. If the number of consecutive errors exceeds `error_threshold`, the driver can automatically stop steering operation to protect the hardware.

## Shutdown Behavior

When the node is shut down:

1. A final stop command is sent to the steering motor.
2. The H-bridge connection is closed safely.
3. All ROS 2 resources are released.

This ensures the steering actuator cannot continue moving after the node exits or the system is powered down.

ADD after proper implementation of PID node
