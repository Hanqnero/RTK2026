#!/usr/bin/env bash

# -e:
# завершить скрипт при ошибке обычной команды.
#
# -u:
# считать ошибкой обращение к необъявленной переменной.
#
# -o pipefail:
# считать pipeline неуспешным, если упала любая его команда.
set -euo pipefail


# Получаем настройки из Dockerfile или Compose.
#
# Если переменная отсутствует или пуста,
# используется значение после :-.
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"


# Создаём runtime-каталог для Qt, Gazebo
# и других Linux GUI-приложений.
mkdir -p "${XDG_RUNTIME_DIR}"

# Runtime-каталог должен быть доступен только владельцу.
chmod 700 "${XDG_RUNTIME_DIR}"


# Запускаем виртуальный X11-сервер.
#
# -screen 0:
# экран номер 0.
#
# 1600x900x24:
# разрешение и глубина цвета.
#
# -ac:
# отключить X11 access control внутри контейнера.
#
# +extension GLX:
# включить GLX для OpenGL.
#
# +render:
# включить X Render extension.
#
# -noreset:
# не сбрасывать сервер при отключении клиентов.
Xvfb "${DISPLAY}" \
    -screen 0 1600x900x24 \
    -ac \
    +extension GLX \
    +render \
    -noreset \
    >/tmp/xvfb.log 2>&1 &

# PID процесса Xvfb.
xvfb_pid=$!


# Ждём готовность X11-сервера.
#
# 50 попыток × 0.1 секунды = максимум 5 секунд.
display_ready=0

for _ in $(seq 1 50); do
    # Если xdpyinfo смог подключиться,
    # X11-сервер готов.
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        display_ready=1
        break
    fi

    # kill -0 не отправляет сигнал.
    # Он только проверяет существование процесса.
    if ! kill -0 "${xvfb_pid}" 2>/dev/null; then
        break
    fi

    sleep 0.1
done


# Если X-сервер не поднялся,
# выводим лог и завершаем контейнер.
if [ "${display_ready}" != "1" ]; then
    echo "Xvfb не запустился на ${DISPLAY}" >&2
    cat /tmp/xvfb.log >&2 || true
    exit 1
fi


# Запускаем оконный менеджер.
fluxbox \
    >/tmp/fluxbox.log 2>&1 &

fluxbox_pid=$!


# Запускаем VNC-сервер поверх Xvfb.
#
# -forever:
# продолжать работу после отключения клиента.
#
# -shared:
# разрешить несколько подключений.
#
# -nopw:
# не использовать VNC-пароль.
#
# -localhost:
# принимать VNC только внутри контейнера.
#
# Снаружи доступ осуществляется через websockify.
x11vnc \
    -display "${DISPLAY}" \
    -forever \
    -shared \
    -nopw \
    -localhost \
    -rfbport 5900 \
    >/tmp/x11vnc.log 2>&1 &

x11vnc_pid=$!


# Запускаем noVNC/WebSocket-прокси.
#
# Порт 6080:
# HTTP и WebSocket для браузера.
#
# localhost:5900:
# внутреннее подключение к x11vnc.
websockify \
    --web=/usr/share/novnc \
    6080 \
    localhost:5900 \
    >/tmp/novnc.log 2>&1 &

websockify_pid=$!


# Даём фоновым процессам время на первичный запуск.
sleep 0.3


# Проверяем Fluxbox.
if ! kill -0 "${fluxbox_pid}" 2>/dev/null; then
    echo "Fluxbox завершился во время запуска" >&2
    cat /tmp/fluxbox.log >&2 || true
    exit 1
fi


# Проверяем x11vnc.
if ! kill -0 "${x11vnc_pid}" 2>/dev/null; then
    echo "x11vnc завершился во время запуска" >&2
    cat /tmp/x11vnc.log >&2 || true
    exit 1
fi


# Проверяем websockify.
if ! kill -0 "${websockify_pid}" 2>/dev/null; then
    echo "websockify завершился во время запуска" >&2
    cat /tmp/novnc.log >&2 || true
    exit 1
fi


# ROS setup-скрипты могут проверять переменные окружения,
# которые до первого подключения ROS ещё не существуют.
#
# Временно отключаем nounset, чтобы обращение к такой
# переменной интерпретировалось как пустая строка.
set +u

# Подключаем системную установку ROS 2 Jazzy.
source /opt/ros/jazzy/setup.bash

# Подключаем собранный workspace поверх системного ROS.
source /workspace/install/setup.bash

# Возвращаем строгую проверку необъявленных переменных
# для оставшейся части нашего собственного скрипта.
set -u


# Проверяем, что Docker передал команду через CMD
# или через поле command в Compose.
if [ "$#" -eq 0 ]; then
    echo "Не передана команда для запуска контейнера" >&2
    exit 64
fi


# Заменяем текущий Bash-процесс ROS-командой.
#
# Благодаря exec ROS launch напрямую получает
# сигналы завершения от Docker.
exec "$@"