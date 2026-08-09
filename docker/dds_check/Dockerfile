# syntax=docker/dockerfile:1.7

# Образ проверки транспорта. Один и тот же для Raspberry Pi и ноутбука:
# роль задаётся конфигом DDS Router и командой, а не сборкой.
#
# Проекта здесь нет намеренно. Задача - доказать, что данные ходят между
# машинами, и отделить проблемы транспорта от проблем прикладного кода.
# Подключение к rtk2026 идёт следующим шагом, после того как транспорт
# признан рабочим.

# Vulcanexus - это ROS 2 плюс инструменты eProsima. Образы публикуются
# и для arm64, что подходит и Raspberry Pi, и Apple Silicon.
#
# Базовый вариант DDS Router не содержит, но репозиторий Vulcanexus в образе
# уже настроен, и Router ставится обычным apt. Это надёжнее, чем угадывать,
# в каком из готовых вариантов образа он есть, и заметно меньше по размеру,
# чем desktop-сборка.
ARG VULCANEXUS_TAG=jazzy
FROM eprosima/vulcanexus:${VULCANEXUS_TAG}

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    \
    # Ретранслятор DDS между изолированными сетями.
    vulcanexus-jazzy-ddsrouter \
    \
    # Проверка прохождения multicast между машинами до включения Router.
    ros-jazzy-ros2multicast \
    \
    # Публикация трансформов имитатором робота: в базовом образе
    # tf2 отсутствует.
    ros-jazzy-tf2-ros-py \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    \
    # Диагностика сети внутри контейнера.
    iproute2 \
    iputils-ping \
    net-tools

# Сборка падает сразу, если Router всё же не встал: иначе ошибка всплыла бы
# при запуске, когда образ уже собран.
RUN source /opt/vulcanexus/*/setup.bash && \
    command -v ddsrouter >/dev/null || { echo "ddsrouter не установился" >&2; exit 1; } && \
    ddsrouter --version

# RMW фиксируется явно и одинаково на обеих сторонах. Pub/sub между Fast DDS
# и Cyclone обычно работает, но реализация ROS 2 services содержит зависящие
# от middleware детали, и совпадение RMW убирает целый класс отказов.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0

WORKDIR /check

COPY talker.py listener.py fake_robot.py /check/
COPY ddsrouter /check/ddsrouter
COPY entrypoint.sh /usr/local/bin/entrypoint
RUN chmod +x /usr/local/bin/entrypoint

EXPOSE 11666

ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["shell"]
