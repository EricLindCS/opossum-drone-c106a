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
cd ~/!!path!!/opossum_drone/child/firmware

python3 -m esptool \
  --chip esp32c6 \
  --port /dev/cu.usbmodem101 \
  --baud 460800 \
  write-flash \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 int32_publisher.bin

```
