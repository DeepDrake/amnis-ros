# Sensor Reader Node

**Node Name:** `sensor_reader`

The Sensor Reader node is responsible for acquiring analog sensor data from the vehicle through an ADS1015 ADC connected over I2C. It reads the gas pedal and steering wheel potentiometer signals, converts them into normalized values, and publishes the data for use by other control nodes. The node supports sensor calibration, diagnostics, and hardware fault monitoring.

## Responsibilities

1. **Analog Sensor Acquisition**: Reads gas pedal and steering wheel potentiometer values from the ADS1015 ADC.

2. **Signal Normalization**: Converts raw ADC readings into normalized values in the range `0.0` to `1.0`.

3. **Calibration Management**: Supports both predefined calibration values and automatic calibration procedures.

4. **Sensor Data Publishing**: Publishes normalized sensor values along with raw ADC readings and calibration status.

5. **Hardware Monitoring**: Monitors ADC communication status and tracks sensor read errors.

6. **Diagnostics**: Publishes detailed diagnostic information including calibration status, raw values, connection status, and error counts.

## Published Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `sensor_data` | `amnis_controller/msg/SensorData` | Normalized gas pedal and steering wheel sensor values along with raw ADC readings. |

## Diagnostic Topics

| Topic | Type | Description |
| :---- | :---- | :---------- |
| `sensor_diagnostics` | `std_msgs/msg/String` | Diagnostic information including sensor readings, calibration status, connection state, and error counts. |

## Sensor Inputs

The node reads two analog channels from the ADS1015 ADC:

| ADC Channel | Sensor | Description |
| :---------- | :----- | :---------- |
| `AIN0` | Gas Pedal | Measures accelerator pedal position. |
| `AIN1` | Steering Wheel Potentiometer | Measures steering wheel position. |

## Published Sensor Data

The node publishes the following fields in the `SensorData` message:

| Field | Description |
| :---- | :---------- |
| `gas_pedal` | Normalized gas pedal position (`0.0 - 1.0`). |
| `steering_wheel` | Normalized steering wheel position (`0.0 - 1.0`). |
| `gas_pedal_raw` | Raw ADC value for the gas pedal. |
| `steering_wheel_raw` | Raw ADC value for the steering wheel potentiometer. |
| `calibrated` | Indicates whether valid calibration data is available. |

## Calibration

### Manual Calibration

Calibration values can be provided through parameters:

| Parameter | Purpose |
| :-------- | :------ |
| `gas_pedal_min` | Minimum gas pedal ADC value. |
| `gas_pedal_max` | Maximum gas pedal ADC value. |
| `steering_wheel_min` | Minimum steering ADC value. |
| `steering_wheel_max` | Maximum steering ADC value. |

These values are used to normalize raw sensor readings into the range `0.0` to `1.0`.

### Automatic Calibration

If `auto_calibrate` is enabled, the node enters calibration mode at startup.

During calibration:

1. The operator moves the gas pedal through its full range.
2. The steering wheel is turned through its full range.
3. The node records minimum and maximum values.
4. Calibration automatically completes after the configured duration.

The resulting calibration values are then used for all future sensor normalization.

## Parameters

| Parameter | Type | Default | Description |
| :-------- | :--- | :------ | :---------- |
| `output_topic` | `string` | `'sensor_data'` | Topic used to publish sensor readings. |
| `diagnostic_topic` | `string` | `'sensor_diagnostics'` | Topic used to publish diagnostics. |
| `i2c_bus` | `int` | `1` | I2C bus used for ADC communication. |
| `i2c_address` | `int` | `0x48` | ADS1015 ADC I2C address. |
| `mock_mode` | `bool` | `False` | Simulates sensor readings without hardware. |
| `pigpio_host` | `string` | `'localhost'` | Hostname of the pigpio daemon. |
| `pigpio_port` | `int` | `8888` | Port used to communicate with pigpio. |
| `gas_pedal_min` | `int` | `0` | Minimum gas pedal calibration value. |
| `gas_pedal_max` | `int` | `2047` | Maximum gas pedal calibration value. |
| `steering_wheel_min` | `int` | `0` | Minimum steering wheel calibration value. |
| `steering_wheel_max` | `int` | `2047` | Maximum steering wheel calibration value. |
| `auto_calibrate` | `bool` | `False` | Enables automatic calibration at startup. |
| `calibration_duration_sec` | `double` | `10.0` | Duration of the auto-calibration process. |
| `update_rate_hz` | `double` | `50.0` | Sensor update frequency. |
| `publish_diagnostics` | `bool` | `True` | Enables diagnostic publishing. |
| `log_throttle_sec` | `double` | `1.0` | Time between status log messages. |
| `verbose` | `bool` | `True` | Enables detailed logging. |

## Safety and Fault Handling

### Sensor Read Failure Detection

If either sensor fails to return a valid reading:

- The read attempt is counted as an error.
- A warning message is generated.
- No sensor message is published for that cycle.

### ADC Connection Monitoring

The node continuously monitors communication with the ADS1015 ADC and reports connection status through diagnostics.

### Calibration Status Tracking

The node tracks whether valid calibration data is available and includes this information in every published sensor message.

## Diagnostics

The diagnostic topic includes:

- Normalized gas pedal value
- Normalized steering wheel value
- Raw gas pedal ADC value
- Raw steering ADC value
- Calibration status
- Calibration progress state
- Calibration ranges
- ADC connection status
- Hardware communication errors
- Sensor read error count

## Shutdown Behavior

When the node is shut down:

1. Sensor polling is stopped.
2. The ADC driver connection is closed.
3. ROS publishers and timers are cleaned up.
4. Diagnostic publishing is terminated.

This ensures the ADC hardware is released cleanly before the node exits.