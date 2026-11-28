#Project structure 11/28/26




#Firmware flashing:

Building:
- Install docker desktop and enter folder

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

#Connecting to and commanding and the child drone

...

To start camera;
'''bash
ros2 topic pub --once /gcs/drone_1/config std_msgs/msg/String "{data: 'CONFIG_SOURCE_RC'}"
'''

