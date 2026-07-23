#!/usr/bin/env bash

# Завершить скрипт при необработанной ошибке команды.
#
# Считать ошибкой обращение к необъявленной переменной.
#
# Считать pipeline ошибочным, если упала любая его команда.
set -euo pipefail


# Настройки графической среды.
#
# Если переменная уже передана из Compose, используется её значение.
# Иначе применяется значение после :-.
export DISPLAY="${DISPLAY:-:1}"
export DIAGNOSTICS_DISPLAY="${DIAGNOSTICS_DISPLAY:-:2}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"


# Runtime-каталог для Qt, Gazebo и других GUI-приложений.
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"


# Запускаем виртуальный X11-дисплей.
Xvfb "${DISPLAY}" \
    -screen 0 1600x900x24 \
    -ac \
    +extension GLX \
    +render \
    -noreset \
    >/tmp/xvfb.log 2>&1 &

xvfb_pid=$!


# Ждём, пока Xvfb начнёт принимать подключения.
display_ready=0

for _ in $(seq 1 50); do
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        display_ready=1
        break
    fi

    # Проверяем, не завершился ли Xvfb во время запуска.
    if ! kill -0 "${xvfb_pid}" 2>/dev/null; then
        break
    fi

    sleep 0.1
done


if [ "${display_ready}" != "1" ]; then
    echo "Xvfb не запустился на ${DISPLAY}" >&2
    cat /tmp/xvfb.log >&2 || true
    exit 1
fi


# Запускаем оконный менеджер внутри виртуального дисплея.
fluxbox \
    >/tmp/fluxbox.log 2>&1 &

fluxbox_pid=$!


# Запускаем VNC-сервер поверх Xvfb.
#
# Он доступен только внутри контейнера.
# Снаружи подключение выполняется через websockify.
x11vnc \
    -display "${DISPLAY}" \
    -forever \
    -shared \
    -nopw \
    -localhost \
    -rfbport 5900 \
    >/tmp/x11vnc.log 2>&1 &

x11vnc_pid=$!


# Запускаем noVNC.
#
# HTTP/WebSocket:
#     порт 6080
#
# Внутренний VNC:
#     localhost:5900
websockify \
    --web=/usr/share/novnc \
    6080 \
    localhost:5900 \
    >/tmp/novnc.log 2>&1 &

websockify_pid=$!


# Даём фоновым процессам время завершить первичный запуск.
sleep 0.3


# Проверяем, что оконный менеджер работает.
if ! kill -0 "${fluxbox_pid}" 2>/dev/null; then
    echo "Fluxbox завершился во время запуска" >&2
    cat /tmp/fluxbox.log >&2 || true
    exit 1
fi


# Проверяем VNC-сервер.
if ! kill -0 "${x11vnc_pid}" 2>/dev/null; then
    echo "x11vnc завершился во время запуска" >&2
    cat /tmp/x11vnc.log >&2 || true
    exit 1
fi


# Проверяем noVNC/WebSocket-сервер.
if ! kill -0 "${websockify_pid}" 2>/dev/null; then
    echo "websockify завершился во время запуска" >&2
    cat /tmp/novnc.log >&2 || true
    exit 1
fi


# Диагностические окна не должны перекрывать Gazebo и RViz.
# Поэтому для них запускается второй независимый X11 desktop:
#
#     DISPLAY=:2
#     VNC:     localhost:5901
#     noVNC:   http://localhost:6081
if [ "${DIAGNOSTICS_DISPLAY}" = "${DISPLAY}" ]; then
    echo "DIAGNOSTICS_DISPLAY должен отличаться от DISPLAY" >&2
    exit 64
fi


# Запускаем второй виртуальный X11-дисплей.
Xvfb "${DIAGNOSTICS_DISPLAY}" \
    -screen 0 1600x900x24 \
    -ac \
    +extension GLX \
    +render \
    -noreset \
    >/tmp/diagnostics_xvfb.log 2>&1 &

diagnostics_xvfb_pid=$!


# Ждём готовности диагностического дисплея.
diagnostics_display_ready=0

for _ in $(seq 1 50); do
    if xdpyinfo -display "${DIAGNOSTICS_DISPLAY}" >/dev/null 2>&1; then
        diagnostics_display_ready=1
        break
    fi

    if ! kill -0 "${diagnostics_xvfb_pid}" 2>/dev/null; then
        break
    fi

    sleep 0.1
done


if [ "${diagnostics_display_ready}" != "1" ]; then
    echo "Xvfb не запустился на ${DIAGNOSTICS_DISPLAY}" >&2
    cat /tmp/diagnostics_xvfb.log >&2 || true
    exit 1
fi


# Отдельный оконный менеджер для диагностического desktop.
DISPLAY="${DIAGNOSTICS_DISPLAY}" fluxbox \
    >/tmp/diagnostics_fluxbox.log 2>&1 &

diagnostics_fluxbox_pid=$!


# VNC-сервер второго desktop доступен только внутри контейнера.
x11vnc \
    -display "${DIAGNOSTICS_DISPLAY}" \
    -forever \
    -shared \
    -nopw \
    -localhost \
    -rfbport 5901 \
    >/tmp/diagnostics_x11vnc.log 2>&1 &

diagnostics_x11vnc_pid=$!


# WebSocket-прокси второго desktop.
websockify \
    --web=/usr/share/novnc \
    6081 \
    localhost:5901 \
    >/tmp/diagnostics_novnc.log 2>&1 &

diagnostics_websockify_pid=$!


# Проверяем, что все процессы второго desktop остались запущены.
sleep 0.3

if ! kill -0 "${diagnostics_fluxbox_pid}" 2>/dev/null; then
    echo "Диагностический Fluxbox завершился во время запуска" >&2
    cat /tmp/diagnostics_fluxbox.log >&2 || true
    exit 1
fi

if ! kill -0 "${diagnostics_x11vnc_pid}" 2>/dev/null; then
    echo "Диагностический x11vnc завершился во время запуска" >&2
    cat /tmp/diagnostics_x11vnc.log >&2 || true
    exit 1
fi

if ! kill -0 "${diagnostics_websockify_pid}" 2>/dev/null; then
    echo "Диагностический websockify завершился во время запуска" >&2
    cat /tmp/diagnostics_novnc.log >&2 || true
    exit 1
fi


# Dockerfile CMD или поле command из Compose
# должны передать основной процесс контейнера.
if [ "$#" -eq 0 ]; then
    echo "Не передана команда для основного процесса контейнера" >&2
    exit 64
fi


# Заменяем entrypoint Bash переданной командой.
#
# В нашем случае это будет:
#
#     sleep infinity
#
# Она просто удерживает контейнер запущенным.
exec "$@"
