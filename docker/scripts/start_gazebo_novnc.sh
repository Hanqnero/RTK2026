#!/bin/bash
set -e
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-gz-gui}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

NOVNC_PORT="${NOVNC_PORT:-6080}"

# Cleanup stale X lock/sock after previous crash.
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

Xvfb :1 -screen 0 "${SCREEN_SIZE:-1600x900x24}" -ac +extension GLX +render -noreset &
sleep 1
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" localhost:5900 >/tmp/websockify.log 2>&1 &

# Дать серверу gz время поднять мир (контейнер gazebo стартует раньше этого).
sleep 4
# Сброс пользовательского GUI-кэша, чтобы не тянуть "кривую" раскладку/камеру из прошлых запусков.
rm -f /root/.gz/sim/8/gui.config 2>/dev/null || true
exec gz sim -g -v 4 --render-engine ogre2
