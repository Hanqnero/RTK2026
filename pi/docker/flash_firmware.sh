#!/usr/bin/env bash
# Сборка и заливка прошивки в Arduino Mega с Raspberry Pi.
#
# Порт может держать только один процесс. Если запущен link_server,
# заливка не пройдёт: остановите его перед прошивкой.
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-pi}"
PORT="${UPLOAD_PORT:-/dev/arduino}"
BAUD="${UPLOAD_BAUD:-115200}"

cd /work

if [ ! -e "$PORT" ]; then
    echo "устройство $PORT не найдено" >&2
    echo "проверьте проброс devices в docker-compose.pi.yml" >&2
    exit 1
fi

bash pi/docker/build_firmware.sh

echo
echo "flash port=$PORT baud=$BAUD"

# Конфиг avrdude берётся системный: он собран под ту же архитектуру,
# что и сам avrdude в образе.
avrdude \
    -p atmega2560 \
    -c wiring \
    -P "$PORT" \
    -b "$BAUD" \
    -D \
    -U "flash:w:arduino/$BUILD_DIR/robot_control_interface.hex:i"

echo
echo "flash = ok"
