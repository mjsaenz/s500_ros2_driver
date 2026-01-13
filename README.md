# s500_ros2_driver

## Overview

This ROS2 package provides a C++ driver for the Cerulean Sonar S500 echosounder. The driver connects to the S500 device via UDP or a serial connection, allowing users to configure ping parameters and receive data.

The node can be configured to publish two types of messages from the S500's Ping-Protocol:
*   `distance2`: A simple message containing the most recent and an averaged distance measurement.
*   `profile6_t`: A more detailed message containing the full backscatter profile data and other sensor metadata.

The package includes separate launch files for each message type.

## Dependencies

*   `ament_cmake`
*   `rclcpp`
*   `std_msgs`
*   `rcl_interfaces`
*   `rosidl_default_generators`

## Building the Package

To build the package, place it in your ROS2 workspace's `src` directory and run `colcon build`:

```bash
# In your workspace root
cd <sonar_ws>/src
git clone https://github.com/mjsaenz/s500_ros2_driver
cd ../
rosdep install --from-paths src --ignore-src
colcon build --packages-select s500_ros2_driver
```

## Usage

Source your workspace and use the provided launch files to start the node. You will need to edit the parameters within the launch file to match your connection type and device settings.

### Publishing `distance2` messages:

[s500_driver/launch/launch_s500_distance2.launch.py](launch/launch_s500_distance2.launch.py)

This provides a simple, low-bandwidth topic with the primary range finding data.

```bash
ros2 launch s500_ros2_driver launch_s500_distance2.launch.py
```

### Publishing `profile6_t` messages:

[s500_driver/launch/launch_s500_profile6_t.launch.py](launch/launch_s500_profile6_t.launch.py)

This provides the full, high-resolution sonar backscatter profile for tasks like sub-bottom profiling or object detection.

```bash
ros2 launch s500_ros2_driver launch_s500_profile6_t.launch.py
```

---

## Node: [s500_publisher_node](src/s500_node.cpp)

### Published Topics

*   `/s500/distance2` ([s500_ros2_driver/msg/S500Distance2](msg/s500Distance2.msg))
    *   Published when the `packet_type` parameter is set to `'distance2'`.
*   `/s500/profile6_t` ([s500_ros2_driver/msg/S500Profile6T](msg/s500Profile6T.msg))
    *   Published when the `packet_type` parameter is set to `'profile6_t'`.

### Parameters

Parameters are set via the launch files. They are divided into three categories: connection, device (static), and dynamic.

#### Connection Parameters (Read-Only)
These are set once at startup to establish a connection with the S500.

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `connection_type` | string | "udp" | Communication protocol. Valid options: `'udp'` or `'serial'`. |
| `udp_ip` | string | "192.168.0.20" | The IPv4 address of the S500 for UDP connections. |
| `udp_port` | int | 51200 | The UDP port of the S500. |
| `serial_port` | string | "/dev/ttyUSB0" | The serial port device name for serial connections. |
| `baud_rate` | int | 115200 | The baud rate for serial connections. |

#### Device Parameters (Read-Only)
These configure the S500's ping characteristics and are set once at startup.

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `start_mm` | int | 0 | Start of the ping range in mm from the transducer head. |
| `length_mm` | int | 20000 | Length of the profile in mm. `end_range = start_mm + length_mm`. |
| `gain_index` | int | -1 | Gain index from 0-13. Set to `-1` for auto-gain. |
| `ping_interval_ms` | int | 100 | Minimum interval between pings in milliseconds. |
| `packet_type` | string | *varies* | Packet to request. Options: `'distance2'` or `'profile6_t'`. |
| `chirp_enable` | bool | True | `True` to enable CHIRP pings, `False` for monotone. |
| `decimation` | int | 0 | Downsampling factor for the sonar signal. `0` for auto-resolution in CHIRP mode. |
| `frame_id` | string | *varies*| The `frame_id` used in the header of published messages. |

#### Dynamic Parameters
These parameters can be changed at any time after the node has launched.

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `sos_mm_per_sec` | int | 1500000 | Speed of sound in the sounding medium in mm/s. |

---

## Custom Message Definitions

### `S500Distance2.msg`

Message for the simple distance and confidence measurement. Ping-Protocol message documentation can be found here: [distance2](https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1223-distance2)

```
std_msgs/Header header

uint32 ping_distance_mm             # Distance of the most recent ping in millimeters
uint32 averaged_distance_mm         # Average distance over the last 20 pings
uint8  ping_confidence              # Confidence of the most recent ping
uint8  average_distance_confidence  # Confidence of the averaged distance
uint32 timestamp_msec               # Device timestamp in milliseconds
```

### `S500Profile6T.msg`

Message for the full backscatter profile data. Ping-Protocol message documentation can be found here: [profile6_t](https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1308-profile6_t)
```
# https://docs.bluerobotics.com/ping-protocol/pingmessage-s500/#1308-profile6_t

std_msgs/Header header

uint32 ping_number
uint32 start_mm
uint32 length_mm
uint32 start_ping_hz
uint32 end_ping_hz
uint32 adc_sample_hz
uint32 timestamp_msec
float32 pulse_duration_sec
float32 analog_gain
float32 max_pwr_db
float32 min_pwr_db
float32 this_ping_depth_m
float32 smooth_depth_m
uint8 ping_depth_measurement_confidence
uint8 gain_index
uint8 decimation
uint8 smooth_depth_measurement_confidence
uint16 num_results
uint16[] pwr_results                     # Array of backscatter power results
```

---

## TODO

*   Add a proper license declaration to `package.xml`.
*   Update the package description in `package.xml`.
*   Move launch parameter values to a YAML file to allow changing configuration without requiring rebuilding.
