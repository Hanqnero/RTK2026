#!/usr/bin/env bash
# Точка входа проверки транспорта.
#
# Роль контейнера задаётся первым аргументом, а не отдельным образом:
# так обе стороны заведомо одинаковы по версиям и настройкам, и расхождение
# между ними исключено как причина отказа.
#
#   router    поднять DDS Router с конфигом ROLE
#   talker    публиковать проверочные топики
#   listener  принимать и считать потери
#   shell     интерактивная оболочка

set -euo pipefail

# setup.bash из Vulcanexus читает необъявленные переменные вроде
# COLCON_TRACE, поэтому проверку неустановленных переменных на время
# подключения окружения приходится снимать.
set +u
source /opt/vulcanexus/*/setup.bash
set -u

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

ROLE="${ROLE:-pi}"
CONFIG="/check/ddsrouter/${ROLE}.yaml"

command="${1:-shell}"
shift || true

case "${command}" in
    router)
        if [ ! -f "${CONFIG}" ]; then
            echo "конфиг ${CONFIG} не найден, проверьте ROLE" >&2
            exit 1
        fi

        # Адрес Raspberry Pi подставляется на запуске: держать его в образе
        # значило бы пересобирать образ при смене сети.
        runtime_config="${CONFIG}"
        if [ -n "${PI_ADDRESS:-}" ]; then
            resolved_address="$(resolve_address "${PI_ADDRESS}")"
            runtime_config=/tmp/ddsrouter.yaml
            sed "s/192\.168\.1\.50/${resolved_address}/g" "${CONFIG}" > "${runtime_config}"
            echo "resolved = ${PI_ADDRESS} -> ${resolved_address}"
        fi

        echo "role = ${ROLE}"
        echo "config = ${runtime_config}"
        echo "pi_address = ${PI_ADDRESS:-из конфига}"
        grep -E "ip:|port:|transport:" "${runtime_config}" | sed 's/^/  /'
        echo

        exec ddsrouter --config-path "${runtime_config}" "$@"
        ;;

    talker)
        exec python3 /check/talker.py "$@"
        ;;

    listener)
        exec python3 /check/listener.py "$@"
        ;;

    shell)
        exec bash "$@"
        ;;

    *)
        exec "${command}" "$@"
        ;;
esac
