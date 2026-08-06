#!/usr/bin/env bash
# Сборка прошивки в контейнере Raspberry Pi.
#
# Каталог сборки отделён от локального build/ разработчика, чтобы кэш
# кросс-компиляции под ARM не смешивался с кэшем ноутбука.
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-pi}"

cd /work/arduino

cmake -S . -B "$BUILD_DIR" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=toolchains/avr-system.cmake

cmake --build "$BUILD_DIR"

echo
echo "hex = arduino/$BUILD_DIR/robot_control_interface.hex"
ls -l "$BUILD_DIR/robot_control_interface.hex"
