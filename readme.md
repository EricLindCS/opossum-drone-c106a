How to compile and flash esp32:

Building:
- Install docker desktop

```bash
docker compose -f child/firmware/docker-compose.yml build
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py build
```

Flashing (for mac):
```bash
docker compose -f child/firmware/docker-compose.yml run --rm child idf.py -p {your port} flash
```
ex: docker compose -f child/firmware/docker-compose.yml run --rm child idf.py -p /dev/cu.usbmodem101 flash

One command:
```bash
cd ~/!!path!!/opossum_drone/child/firmware/build

python3 -m esptool --chip esp32c6 --port /dev/cu.usbmodem101 --baud 460800 write-flash 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x10000 opossum_child.bin

```
