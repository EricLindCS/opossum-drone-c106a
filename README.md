# Project structure 11/28/26

```text
opossum_drone/
├── README.md                     # Build and flashing instructions
├── PROJECT_STRUCTURE.md          # Repository layout reference
│
├── child/                        # Drone-side firmware and flight controller software
│   └── firmware/
│       ├── CMakeLists.txt        # ESP-IDF project definition
│       ├── Dockerfile            # ESP-IDF build environment
│       ├── docker-compose.yml    # Containerized build and flash commands
│       ├── sdkconfig             # ESP-IDF configuration
│       ├── sdkconfig.defaults    # Default ESP-IDF settings
│       ├── partitions.csv        # Flash partition layout
│       ├── main/                 # Application entry point and drone state machine
│       │   ├── main.c
│       │   ├── CMakeLists.txt
│       │   ├── Kconfig.projbuild
│       │   ├── mavlink/          # MAVLink headers and support code
│       │   └── md.md
│       ├── components/           # Locally integrated ESP-IDF components
│       ├── managed_components/   # ESP-IDF component manager dependencies
│       └── build/                # Generated firmware binaries and build products
│
├── ros_workspace/                # ROS 2 source and utility code
│   ├── gcs/                      # Ground control station packages
│   │   └── src/
│   │       ├── aruco_handle/     # ArUco marker and drone handle detection COPY PASTED FROM LAB 5
│   │       └── reciever/         # GCS mission and command receiver
│   ├── calibration/              # COPY PASTED FROM LAB 6 Camera calibration scripts and package
│   │   ├── calibration.py
│   │   ├── common.py
│   │   ├── make_checkerboard.py
│   │   ├── requirements.txt
│   │   └── src/
│   └── src/

```

# Child Firmware flashing:

Building:
- Install docker desktop
- Install Espressif Installation Manager (newest ver): https://docs.espressif.com/projects/idf-im-ui/en/latest/
- Then:

```bash
cd .../child/firmware
```

- Build the code

```bash
docker compose -f child/firmware/docker-compose.yml build
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py build
```

- Flashing (for mac) - in another terminal:

```bash
python3 -m esptool --chip esp32c6 --port /dev/cu.usbmodem101 --baud 460800 write-flash 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x10000 opossum_child.bin
```
(replace /dev/cu... with your usb serial port)

# Connecting to and commanding and the child drone

- Install Micro-Ros on your "ground station" and make sure it's on a wifi network the child can connect to (a hotspot works great for this)

## Quick micro-ROS Menuconfig Guide

Run the ESP-IDF configuration menu inside the project Docker container:

```bash
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py menuconfig
```

Configure these sections:

1. **micro-ROS Settings → micro-ROS middleware**
   - Select **micro-ROS over eProsima Micro XRCE-DDS**.

2. **micro-ROS Settings → micro-ROS network interface select**
   - Select **WLAN interface**.

3. **micro-ROS Settings → WiFi Configuration**
   - Set **WiFi SSID** to the hotspot or network name.
   - Set **WiFi Password** to the network password.
   - Leave **Maximum retry** at the default unless a different reconnection behavior is needed.

4. **micro-ROS Settings → micro-ROS Agent IP**
   - Enter the IP address of the computer running the micro-ROS agent.
   - The ESP32 and agent computer must be reachable on the same network.

5. **micro-ROS Settings → micro-ROS Agent Port**
   - Use `8888` unless the agent is started with another UDP port.

Press `S` to save and `Q` to exit menuconfig. The Docker workflow stores the generated settings in `child/firmware/sdkconfig`.

Rebuild and flash after saving:

```bash
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py build
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py -p /dev/cu.usbmodem101 flash
```
Start the UDP micro-ROS agent on the computer with:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

To start camera;
```bash
ros2 topic pub --once /gcs/drone_1/config std_msgs/msg/String "{data: 'CONFIG_SOURCE_RC'}"
```

## Child Drone ROS 2 Topics

#### See /gcs/src/reciever/cube_mission for a sample flight that traces the path of a cube

The child firmware uses `DRONE_ID` to generate every topic name. The current firmware is configured as drone 1, so the topic names below use `/drone_1` and `/gcs/drone_1`. Change `DRONE_ID` in `child/firmware/main/main.c` and rebuild for another drone ID.

Subscribers use the best-effort QoS profile. Publishers are initialized with the default micro-ROS/rcl publisher QoS unless otherwise noted.

### GCS-to-Drone Subscribers

| Topic | Message type | Purpose |
|---|---|---|
| `/gcs/drone_1/command` | `std_msgs/msg/String` | Flight-state and mission commands |
| `/gcs/drone_1/config` | `std_msgs/msg/String` | Select the flight control source |
| `/gcs/drone_1/control` | `geometry_msgs/msg/PoseStamped` | Position setpoint and optional yaw |

#### `/gcs/drone_1/command`

The payload is a case-sensitive command string. Leading and trailing whitespace, tabs, and newlines are removed.

| Payload | Effect |
|---|---|
| `COMMAND_ARM` | Captures the current PX4 local position as the home position, enters `armed`, immediately publishes state, and starts pre-arm position streaming |
| `COMMAND_DISARM` | Stops pre-arm streaming, disarms PX4, and enters `disarmed` |
| `COMMAND_MISSION_START` | Starts the mission only when currently `armed` or `returning_home`; enters `mission` and runs the takeoff/hover/landing sequence |
| `COMMAND_RETURN_HOME` | Stops the active mission, enters `returning_home`, and flies to the captured home position |
| `COMMAND_ELAND` | Starts emergency landing behavior |
| `COMMAND_KILL` | Stops active flight tasks and sends the PX4 kill/disarm action, entering `killed` |

Unknown values are logged and ignored.

```bash
ros2 topic pub --once /gcs/drone_1/command std_msgs/msg/String "{data: 'COMMAND_ARM'}"
ros2 topic pub --once /gcs/drone_1/command std_msgs/msg/String "{data: 'COMMAND_MISSION_START'}"
ros2 topic pub --once /gcs/drone_1/command std_msgs/msg/String "{data: 'COMMAND_RETURN_HOME'}"
ros2 topic pub --once /gcs/drone_1/command std_msgs/msg/String "{data: 'COMMAND_DISARM'}"
```

#### `/gcs/drone_1/config`

| Payload | Effect |
|---|---|
| `CONFIG_SOURCE_RC` | Disables GCS control forwarding and switches PX4 to stabilized mode so the RC transmitter controls the drone |
| `CONFIG_SOURCE_GCS` | Enables GCS control forwarding and switches PX4 to offboard mode |

```bash
ros2 topic pub --once /gcs/drone_1/config std_msgs/msg/String "{data: 'CONFIG_SOURCE_RC'}"
ros2 topic pub --once /gcs/drone_1/config std_msgs/msg/String "{data: 'CONFIG_SOURCE_GCS'}"
```

Unknown values are logged and ignored.

#### `/gcs/drone_1/control`

The message is a target position and attitude expressed as:

- `pose.position.x`: horizontal local-frame X position in metres
- `pose.position.y`: horizontal local-frame Y position in metres
- `pose.position.z`: altitude in metres, positive upward
- `pose.orientation`: quaternion used to extract a Z-axis yaw angle
- `yaw`: 0 radians faces north; positive yaw is clockwise in the NED convention

The firmware stores the latest setpoint and converts altitude to PX4 NED as `z_ned = -position.z`. While GCS control is active and the drone is `armed`, the position is forwarded immediately to PX4. During a mission, the latest setpoint and yaw are streamed to PX4 at 20 Hz. Setpoint data is not reused across missions; mission start clears it.

Example:

```bash
ros2 topic pub -r 10 /gcs/drone_1/control geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Drone-to-GCS Publishers

| Topic | Message type | Publish behavior | Payload |
|---|---|---|---|
| `/drone_1/state` | `std_msgs/msg/String` | Immediately on state change, and periodically every 2 seconds while the state remains unchanged | Current state name |
| `/drone_1/role` | `std_msgs/msg/String` | Once when the micro-ROS node starts | Role string; currently always `idle` |
| `/drone_1/battery` | `std_msgs/msg/Int8` | Every 5 seconds | Battery remaining from 0–100, or `-1` while unknown |
| `/drone_1/attitude` | `geometry_msgs/msg/Vector3` | Every 100 ms when PX4 attitude is valid | Euler attitude angles in radians: roll, pitch, yaw |
| `/drone_1/position` | `geometry_msgs/msg/Vector3` | Every 100 ms when PX4 local position is valid | Local position in metres with positive-up Z |
| `/drone_1/velocity` | `geometry_msgs/msg/Vector3` | Every 100 ms when PX4 local position is valid | Local velocity in metres per second |

#### `/drone_1/state`

Possible values are:

- `disarmed`
- `armed`
- `mission`
- `returning_home`
- `landing`
- `killed`

#### `/drone_1/battery`

The value comes from the PX4 `BATTERY_STATUS` MAVLink message. If no valid battery telemetry has been received, the initial value is `-1`.

#### `/drone_1/attitude`

This topic currently uses `geometry_msgs/msg/Vector3` rather than a quaternion:

- `x`: roll in radians
- `y`: pitch in radians
- `z`: yaw in radians

#### `/drone_1/position`

Position is derived from PX4 `LOCAL_POSITION_NED` and converted to positive-up Z:

- `x`: PX4 NED X in metres
- `y`: PX4 NED Y in metres
- `z`: `-lpos.z`, so altitude is positive upward

#### `/drone_1/velocity`

Velocity comes directly from PX4 `LOCAL_POSITION_NED`:

- `x`: NED X velocity in metres per second
- `y`: NED Y velocity in metres per second
- `z`: NED Z velocity in metres per second

