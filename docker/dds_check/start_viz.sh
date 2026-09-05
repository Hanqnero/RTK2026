#!/usr/bin/env bash
# Запуск визуализации: X11, VNC, noVNC, DDS Router и RViz.
#
# Router поднимается раньше RViz. При discovery-trigger: reader обратный
# порядок тоже сработает, но первые секунды RViz показывал бы пустоту.

set -euo pipefail

# DDS Router принимает в адресах только IP: имя хоста он не резолвит и
# отвергает конфиг целиком. Поэтому имя разрешается здесь.
#
# Это существенно для Docker Desktop, где вторая сторона доступна как
# host.docker.internal, а не по фиксированному адресу.
resolve_address() {
    local value="$1"

    # Уже IP - оставляем как есть.
    if [[ "${value}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        printf '%s' "${value}"
        return
    fi

    # Запрашиваем именно IPv4: getent hosts может вернуть IPv6, а порты
    # Docker публикуются на IPv4, и соединение по IPv6-адресу не пройдёт.
    local resolved
    resolved="$(getent ahostsv4 "${value}" 2>/dev/null | awk '{print $1; exit}')"

    if [ -z "${resolved}" ]; then
        echo "не удалось разрешить имя ${value} в IP-адрес" >&2
        exit 1
    fi

    printf '%s' "${resolved}"
}

export DISPLAY="${DISPLAY:-:1}"
DDSROUTER_CONFIG="${DDSROUTER_CONFIG:-/check/ddsrouter/mac.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-/check/rviz/check.rviz}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

# setup.bash из Vulcanexus читает необъявленные переменные вроде
# COLCON_TRACE, поэтому проверку неустановленных переменных на время
# подключения окружения приходится снимать.
set +u
source /opt/vulcanexus/*/setup.bash
set -u

children=()
cleanup() {
    for pid in "${children[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

# Внешний X-сервер или собственный Xvfb.
#
# На Linux сокет хоста пробрасывается внутрь (/tmp/.X11-unix) и DISPLAY
# приходит снаружи - тогда окно RViz открывается прямо на рабочем столе,
# и вся связка Xvfb + x11vnc + noVNC не нужна: она существует ради macOS,
# где проброс X11 не работает.
#
# Отличаем одно от другого проверкой: если по указанному DISPLAY уже
# кто-то отвечает, свой сервер не поднимаем. Иначе Xvfb упёрся бы в
# "Server is already active for display 0".
if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
    echo "display = ${DISPLAY} (внешний X-сервер, noVNC не нужен)"
else
    Xvfb "${DISPLAY}" -screen 0 "${SCREEN_GEOMETRY:-1600x900x24}" -nolisten tcp &
    children+=($!)

    for _ in $(seq 50); do
        xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1 && break
        sleep 0.2
    done

    if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        echo "Xvfb не поднялся на ${DISPLAY}" >&2
        exit 1
    fi

    fluxbox >/dev/null 2>&1 &
    children+=($!)

    x11vnc -display "${DISPLAY}" -forever -shared -nopw -quiet &
    children+=($!)

    websockify --web /usr/share/novnc "${NOVNC_PORT}" localhost:5900 >/dev/null 2>&1 &
    children+=($!)

    echo "novnc = http://127.0.0.1:${NOVNC_PORT}/vnc.html"
fi

# Router нужен не всегда.
#
# Он заменяет автообнаружение одним TCP-соединением там, где multicast
# не проходит - то есть в Docker Desktop на macOS. На Linux контейнер
# запускается с network_mode: host, получает настоящую сеть машины, и
# DDS находит робота сам; посредник только добавил бы задержку.
#
# Признак - пустой PI_ADDRESS: адреса второй стороны нет, значит и
# соединять нечего.
if [ -z "${PI_ADDRESS:-}" ]; then
    echo "router = не запускается (PI_ADDRESS пуст, обнаружение прямое)"
else
    # Адрес другой стороны подставляется на запуске: держать его в образе
    # значило бы пересобирать образ при смене сети.
    resolved_address="$(resolve_address "${PI_ADDRESS}")"
    runtime_config=/tmp/ddsrouter.yaml
    sed "s/192\.168\.1\.50/${resolved_address}/g" "${DDSROUTER_CONFIG}" > "${runtime_config}"
    DDSROUTER_CONFIG="${runtime_config}"
    echo "resolved = ${PI_ADDRESS} -> ${resolved_address}"

    grep -E "ip:|port:|transport:" "${DDSROUTER_CONFIG}" | sed 's/^/  /'

    ddsrouter --config-path "${DDSROUTER_CONFIG}" &
    children+=($!)

    # Router должен успеть установить соединение до старта RViz.
    sleep 3
fi

if [ -f "${RVIZ_CONFIG}" ]; then
    echo "rviz_config = ${RVIZ_CONFIG}"
    rviz2 -d "${RVIZ_CONFIG}" &
else
    rviz2 &
fi
children+=($!)

wait -n
