#Project structure 11/28/26

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
│   │       ├── aruco_handle/     # ArUco marker and drone handle detection
│   │       └── reciever/         # GCS mission and command receiver
│   ├── calibration/              # Camera calibration scripts and package
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

