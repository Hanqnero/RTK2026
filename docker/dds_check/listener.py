#!/usr/bin/env python3
"""Приёмник проверки транспорта: считает пропуски, разброс и обрывы.

Что именно измеряется
---------------------

``received``
    Сколько сообщений дошло.

``lost``
    Сколько номеров пропущено. Считается по разрывам последовательности,
    поэтому потеря отличима от замедления: при замедлении номера идут подряд,
    при потере в них дыры.

``jitter``
    Разброс интервалов между приходами. Абсолютная задержка не измеряется:
    часы машин не синхронизированы, а для оценки канала важен именно разброс.

``latched``
    Получено ли удержанное значение и через сколько после старта. Это проверка
    late-joiner: подписчик стартует позже публикации, но обязан получить
    последнее значение. Если оно не приходит, TRANSIENT_LOCAL не проходит
    через маршрут.

``gaps``
    Перерывы в потоке длиннее порога. Так виден обрыв связи и время
    восстановления: для проверки reconnect это главная величина.
"""

from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from talker import (
    LATCHED_TOPIC,
    SENSOR_TOPIC,
    VOLATILE_TOPIC,
    latched_qos,
    sensor_qos,
    volatile_qos,
)


class TopicStats:
    """Учёт одного топика."""

    def __init__(self, name: str, started_at: float) -> None:
        self.name = name
        # Момент старта узла. Все отметки времени в отчёте отсчитываются
        # от него, иначе "первое сообщение через N секунд" превращается
        # в бессмысленное абсолютное значение монотонных часов.
        self._started_at = started_at
        self.received = 0
        self.lost = 0
        self.out_of_order = 0
        self.first_at: float | None = None
        self.last_at: float | None = None
        self.intervals: list[float] = []
        self.gaps: list[tuple[float, float]] = []
        self._last_sequence: int | None = None

    def update(self, sequence: int, gap_threshold_s: float) -> None:
        now = time.monotonic()

        if self.first_at is None:
            self.first_at = now - self._started_at
        if self.last_at is not None:
            interval = now - self.last_at
            self.intervals.append(interval)

            # Перерыв заметно длиннее обычного интервала - признак обрыва.
            if interval > gap_threshold_s:
                self.gaps.append((self.last_at - self._started_at, interval))

        self.last_at = now
        self.received += 1

        if self._last_sequence is not None:
            step = sequence - self._last_sequence
            if step > 1:
                self.lost += step - 1
            elif step <= 0:
                # Перезапуск передатчика обнуляет счётчик; это не потеря.
                self.out_of_order += 1

        self._last_sequence = sequence

    def report(self) -> list[str]:
        lines = [f"{self.name}:"]

        if self.received == 0:
            lines.append("  received = 0")
            return lines

        expected = self.received + self.lost
        loss_ratio = self.lost / expected if expected else 0.0

        lines.append(f"  received = {self.received}")
        lines.append(f"  lost = {self.lost} ({100.0 * loss_ratio:.2f} %)")

        if self.out_of_order:
            lines.append(f"  sequence_restarts = {self.out_of_order}")

        if self.first_at is not None:
            lines.append(f"  first_message_after_s = {self.first_at:.3f}")

        if self.last_at is not None and self.first_at is not None:
            span = (self.last_at - self._started_at) - self.first_at
            if span > 0:
                lines.append(f"  rate_hz = {(self.received - 1) / span:.2f}")

        if len(self.intervals) > 1:
            lines.append(f"  interval_mean_ms = {1000.0 * statistics.fmean(self.intervals):.2f}")
            lines.append(f"  interval_max_ms = {1000.0 * max(self.intervals):.2f}")
            lines.append(f"  jitter_stdev_ms = {1000.0 * statistics.pstdev(self.intervals):.2f}")

        lines.append(f"  gaps = {len(self.gaps)}")
        for started_at, duration in self.gaps:
            lines.append(
                f"    gap at {started_at:.1f} s, duration {duration:.2f} s"
            )

        return lines


class Listener(Node):
    def __init__(self, gap_threshold_s: float, report_period_s: float) -> None:
        super().__init__("transport_check_listener")

        self._started_at = time.monotonic()
        self._gap_threshold_s = gap_threshold_s

        self._stats = {
            VOLATILE_TOPIC: TopicStats(VOLATILE_TOPIC, self._started_at),
            SENSOR_TOPIC: TopicStats(SENSOR_TOPIC, self._started_at),
        }

        self._latched_payload: str | None = None
        self._latched_at: float | None = None

        self.create_subscription(
            String, VOLATILE_TOPIC,
            lambda msg: self._on_message(VOLATILE_TOPIC, msg),
            volatile_qos(),
        )
        self.create_subscription(
            String, SENSOR_TOPIC,
            lambda msg: self._on_message(SENSOR_TOPIC, msg),
            sensor_qos(),
        )
        self.create_subscription(
            String, LATCHED_TOPIC, self._on_latched, latched_qos()
        )

        self.create_timer(report_period_s, self._report)

        self.get_logger().info(f"gap_threshold_s = {gap_threshold_s}")
        self.get_logger().info("Ctrl+C для итогового отчёта")

    def _on_message(self, topic: str, message: String) -> None:
        parts = message.data.split()
        if len(parts) < 2:
            return

        try:
            sequence = int(parts[1])
        except ValueError:
            return

        self._stats[topic].update(sequence, self._gap_threshold_s)

    def _on_latched(self, message: String) -> None:
        if self._latched_payload is not None:
            return

        self._latched_payload = message.data
        self._latched_at = time.monotonic() - self._started_at

        self.get_logger().info(
            f"latched_received_after_s = {self._latched_at:.3f}"
        )
        self.get_logger().info(f"latched_payload = {message.data}")

    def _report(self) -> None:
        elapsed = time.monotonic() - self._started_at

        volatile = self._stats[VOLATILE_TOPIC]
        sensor = self._stats[SENSOR_TOPIC]

        self.get_logger().info(
            f"elapsed_s={elapsed:.0f} "
            f"volatile={volatile.received}/{volatile.lost} "
            f"sensor={sensor.received}/{sensor.lost} "
            f"latched={'yes' if self._latched_payload else 'no'}"
        )

    def summary(self) -> str:
        elapsed = time.monotonic() - self._started_at

        lines = ["", f"elapsed_s = {elapsed:.1f}", ""]

        for stats in self._stats.values():
            lines.extend(stats.report())
            lines.append("")

        lines.append(f"{LATCHED_TOPIC}:")
        if self._latched_payload is None:
            # Отдельно отмечаем: это единственная проверка, которая ловит
            # неверную настройку remove-unused-entities у DDS Router.
            lines.append("  received = no")
            lines.append("  TRANSIENT_LOCAL не прошёл через маршрут")
        else:
            lines.append("  received = yes")
            lines.append(f"  after_s = {self._latched_at:.3f}")
            lines.append(f"  payload = {self._latched_payload}")

        return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Принимать проверочные топики и считать потери и обрывы"
    )
    parser.add_argument(
        "--gap-threshold",
        type=float,
        default=1.0,
        help="Перерыв длиннее этого считается обрывом, секунды",
    )
    parser.add_argument(
        "--report-period",
        type=float,
        default=5.0,
        help="Период промежуточных строк состояния, секунды",
    )
    arguments = parser.parse_args()

    rclpy.init()
    node = Listener(arguments.gap_threshold, arguments.report_period)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
