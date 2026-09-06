#!/usr/bin/env bash
# Синхронизация рабочего дерева на Raspberry Pi.
#
# Зачем не git clone
# ------------------
#
# Роботу нужна малая часть репозитория. Симуляционные миры, документация,
# датасеты YOLO и пакеты зрения с навигацией на нём не запускаются, но
# занимают место и путаются под ногами при разборе того, что там лежит.
# Здесь отправляется ровно то, что робот исполняет.
#
# Что НЕ трогается
# ----------------
#
# maps/ и records/ - рабочие каталоги самого робота: туда он сохраняет
# карты и записи прогонов. Скрипт их только создаёт, если нет, и никогда
# не удаляет содержимое. Всё остальное зеркалируется с --delete, поэтому
# лишний файл на Pi не переживёт следующей синхронизации.
#
# Использование
# -------------
#
#     pi/tools/sync_to_pi.sh                 # хост по умолчанию
#     pi/tools/sync_to_pi.sh 10.42.0.1       # через точку ROSSIYANE
#     CLEAN=1 pi/tools/sync_to_pi.sh         # ещё и убрать лишнее с Pi

set -euo pipefail

PI_HOST="${1:-pi.local}"
PI_USER="${PI_USER:-pi}"
PI_ROOT="RTK2026"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REMOTE="${PI_USER}@${PI_HOST}"

# Не позволяем rsync --delete превратить исправный vendor-каталог на Pi
# в пустой, если локальный Git-подмодуль забыли инициализировать.
if [ ! -f "${ROOT}/vendor/sllidar_ros2/package.xml" ]; then
    printf '%s\n' \
        'vendor/sllidar_ros2 не инициализирован.' \
        'Выполните: git submodule update --init --recursive vendor/sllidar_ros2' >&2
    exit 1
fi

# Пакеты ROS, которые робот действительно собирает. Список обязан
# совпадать с --packages-select в pi/docker/Dockerfile.ros: иначе сборка
# упадёт на отсутствующем пакете либо соберёт лишнее.
ROS_PACKAGES=(
    rtk2026_driver
    rtk2026_description
    rtk2026_localization
    rtk2026_slam
    rtk2026_observability
    rtk2026_bringup
    rtk2026_interfaces
)

# Каталоги, которые едут целиком.
#
# docker/dds_check нужен для DDS Router на стороне робота, остальные
# наборы образов в docker/ относятся к ноутбуку и симуляции.
TREES=(
    arduino
    pi
    protocol
    docker/dds_check
    vendor/sllidar_ros2
)

# Мусор сборки и окружений: он пересоздаётся на месте и по сети ехать
# не должен. build-pi исключён намеренно - это кэш кросс-компиляции
# самой Pi, и перезаписывать его копией с ноутбука нельзя.
#
# meshes/ и worlds/ исключены отдельно: это 6 МБ STL и миры, нужные
# только для картинки и симуляции. Описание робота разворачивается на Pi
# без геометрии, всё отображение считает ноутбук.
EXCLUDES=(
    --exclude ".git/"
    --exclude "__pycache__/"
    --exclude ".venv/"
    --exclude ".cache/"
    --exclude "build/"
    --exclude "build-pi/"
    --exclude "build-pi-test/"
    --exclude "install/"
    --exclude "log/"
    --exclude "*.pyc"
    --exclude "meshes/"
    --exclude "worlds/"
)

printf 'хост      = %s\n' "${REMOTE}"
printf 'источник  = %s\n\n' "${ROOT}"

ssh "${REMOTE}" "mkdir -p ${PI_ROOT}/src ${PI_ROOT}/maps ${PI_ROOT}/records ${PI_ROOT}/workspaces/robot_ws/src"

for tree in "${TREES[@]}"; do
    printf '  %s\n' "${tree}"
    ssh "${REMOTE}" "mkdir -p ${PI_ROOT}/$(dirname "${tree}")"
    rsync -az --delete "${EXCLUDES[@]}" \
        "${ROOT}/${tree}/" "${REMOTE}:${PI_ROOT}/${tree}/"
done

for package in "${ROS_PACKAGES[@]}"; do
    printf '  src/%s\n' "${package}"
    rsync -az --delete "${EXCLUDES[@]}" \
        "${ROOT}/src/${package}/" "${REMOTE}:${PI_ROOT}/src/${package}/"
done

# Уборка того, что осталось от прежнего полного клона.
#
# Отдельным шагом и по явному списку, а не rm -rf по всему дереву:
# maps/ и records/ содержат данные, которых нет в репозитории, и потерять
# их из-за широкой маски нельзя.
if [ "${CLEAN:-0}" = "1" ]; then
    printf '\nуборка лишнего\n'

    keep_pattern="$(printf '%s\\|' "${ROS_PACKAGES[@]}")"
    keep_pattern="${keep_pattern%\\|}"

    ssh "${REMOTE}" "
        cd ${PI_ROOT} || exit 0
        rm -rf yolo worlds docs pc runtime isaac_lab install build log
        cd src 2>/dev/null || exit 0
        for d in */; do
            name=\"\${d%/}\"
            echo \"\${name}\" | grep -qx '${keep_pattern}' || rm -rf \"\${name}\"
        done
    "
fi

printf '\nготово. Размер на роботе:\n'
ssh "${REMOTE}" "du -sh ${PI_ROOT}; du -sh ${PI_ROOT}/*/ 2>/dev/null | sort -rh | head -8"
