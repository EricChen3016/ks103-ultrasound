# KS103 ultrasound — ROS 2 Humble

ROS 2 driver for four KS103 sensors on one Linux I2C bus. Measurements are deliberately
triggered **sequentially** (left-front → right-front → left-rear → right-rear by default),
which avoids acoustic interference and concurrent I2C transactions. A failed trigger,
timeout, read error, or out-of-range result is logged and is not published.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ks103_ultrasound
source install/setup.bash
ros2 launch ks103_ultrasound example.launch.py
```

The operator running the node must have read/write access to the configured I2C device.
On Ubuntu/Jetson this normally means membership in the `i2c` group (log out and back in
after changing group membership):

```bash
sudo usermod -aG i2c "$USER"
stat /dev/i2c-1
```

Do not run the node as root merely to bypass device permissions. Addresses below are
examples; program each sensor to a unique address and set the actual values for the robot.

## Four-sensor example and parameters

See [`config/example.yaml`](config/example.yaml). Run it directly with:

```bash
ros2 run ks103_ultrasound ks103_node --ros-args \
  --params-file src/ks103-ultrasound/config/example.yaml
```

| Parameter | Meaning | Default |
|---|---|---|
| `i2c_device` | Linux I2C character device | `/dev/i2c-1` |
| `sensors.<position>.address` | 7-bit address for each of `left_front`, `right_front`, `left_rear`, `right_rear` | `0x74`–`0x77` |
| `sensors.<position>.frame_id` | TF frame written to the message header | position-specific |
| `firing_order` | Permutation controlling sequential triggering | LF, RF, LR, RR |
| `measurement_timeout` | Maximum time allowed for one measurement (seconds) | `0.15` |
| `firing_interval_ms` | Interval between sequential sensor callbacks | `125` |
| `noise_filtering` | KS103 filter level (`0`–`6`) | `1` |
| `field_of_view` | Range message field of view (radians) | `0.52` |
| `min_range`, `max_range` | Valid/published range limits (metres) | `0.02`, `11.28` |

All four addresses and the device path are parameters rather than constants in the driver.
The production backend uses Linux `I2C_RDWR`; a mutex covers each complete trigger/read
transaction. The node owns one timer, so it never triggers four sensors in parallel.

## Topics

Each successful measurement is a `sensor_msgs/msg/Range` with `ULTRASOUND`, a current
timestamp, configured frame, field of view, and range limits:

- `/ultrasound/left_front`
- `/ultrasound/left_rear`
- `/ultrasound/right_front`
- `/ultrasound/right_rear`

## Tests

```bash
colcon test --packages-select ks103_ultrasound
colcon test-result --verbose
```

The tests inject a mock `IKs103Bus`; no physical I2C hardware is accessed.

## RS485 transports

Set `transport` to `rs485_usb` or `rs485_uart`. Both use the shared `Ks103Rs485Protocol`: an exact `[address][0x02][command]` request and two-byte `[HIGH][LOW]` response, without Modbus CRC. Addresses are raw 8-bit values from `0xD0` through `0xFE`, excluding `0xF0`, `0xF2`, `0xF4`, and `0xF6`. `rs485_uart` requires kernel `TIOCSRS485` support; USB adapters are expected to provide automatic direction control.

The default RS485 command is `0xB0`; Linux I2C retains its existing `0xBC` conversion behavior. Serial bytes are written separately with configurable `command_byte_delay_us` (20–100 us, default 50). The driver flushes stale input and permits only one outstanding transaction. Timing telemetry is published on `diagnostics`; `Range.header.stamp` is captured independently after each complete response.

Real USB/UART bus timing has **NOT VERIFIED** status until tested on the target hardware.
