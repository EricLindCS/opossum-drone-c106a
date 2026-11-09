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
cd opossum_drone

docker compose -f child/firmware/docker-compose.yml run --rm child \
  idf.py -p /dev/ttyACM0 build flash monitor

```
