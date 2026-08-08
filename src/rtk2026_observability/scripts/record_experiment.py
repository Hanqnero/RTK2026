#!/usr/bin/env python3
"""Универсальная запись текущего ROS 2 graph в rosbag2."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import subprocess
from typing import Any

from ament_index_python.packages import get_package_share_directory
import yaml


def _default_config_path() -> Path:
    """Вернуть установленную конфигурацию записи экспериментов."""
    return (
        Path(get_package_share_directory("rtk2026_observability"))
        / "config"
        / "experiment_recording.yaml"
    )


def _arguments() -> argparse.Namespace:
    """Разобрать путь к конфигурации без параметров конкретного запуска."""
    parser = argparse.ArgumentParser(
        description=(
            "Записать доступные ROS 2 topics в автоматически именованный "
            "каталог rosbag2."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="YAML-конфигурация rosbag2.",
    )
    return parser.parse_args()


def _load_config(path: Path) -> dict[str, Any]:
    """Загрузить и проверить универсальную конфигурацию rosbag2."""
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError("Корень конфигурации должен быть YAML mapping")

    required = {
        "output_root",
        "directory_prefix",
        "storage_id",
        "use_sim_time",
        "record_all_topics",
    }
    missing = required.difference(config)
    if missing:
        raise ValueError(
            f"В конфигурации отсутствуют поля: {sorted(missing)}"
        )

    topics = config.get("topics", [])
    if not config["record_all_topics"] and not topics:
        raise ValueError(
            "При record_all_topics=false список topics не должен быть пустым"
        )
    if topics and not isinstance(topics, list):
        raise ValueError("Поле topics должно быть YAML list")

    return config


def _clock_is_available() -> bool:
    """Проверить наличие simulation clock в текущем ROS graph."""
    result = subprocess.run(
        ["ros2", "topic", "list"],
        check=True,
        capture_output=True,
        text=True,
    )
    return "/clock" in result.stdout.splitlines()


def _resolve_sim_time(value: Any) -> bool:
    """Преобразовать auto, true или false в итоговый режим времени."""
    if isinstance(value, bool):
        return value

    normalized = str(value).strip().lower()
    if normalized == "auto":
        return _clock_is_available()
    if normalized == "true":
        return True
    if normalized == "false":
        return False

    raise ValueError("use_sim_time должен быть auto, true или false")


def _write_recording_snapshot(
    path: Path,
    config: dict[str, Any],
    experiment_id: str,
    started_at: str,
    finished_at: str | None,
    status: str,
    use_sim_time: bool,
    return_code: int | None,
) -> None:
    """Сохранить параметры и результат конкретного запуска рекордера."""
    snapshot = {
        "schema": "rtk2026.experiment_recording.v1",
        "experiment_id": experiment_id,
        "status": status,
        "started_at_utc": started_at,
        "finished_at_utc": finished_at,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "use_sim_time": use_sim_time,
        "return_code": return_code,
        "recording_config": config,
    }
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            snapshot,
            stream,
            allow_unicode=True,
            sort_keys=False,
        )


def _build_command(
    config: dict[str, Any],
    bag_directory: Path,
    use_sim_time: bool,
) -> list[str]:
    """Собрать ros2 bag record только из значений конфигурации."""
    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        str(config["storage_id"]),
        "--output",
        str(bag_directory),
    ]

    preset = str(config.get("storage_preset_profile", "")).strip()
    if preset:
        command.extend(["--storage-preset-profile", preset])

    if use_sim_time:
        command.append("--use-sim-time")

    if config["record_all_topics"]:
        command.append("--all")
    else:
        command.append("--topics")
        command.extend(str(topic) for topic in config["topics"])

    return command


def main() -> None:
    """Создать каталог, запустить rosbag2 и корректно завершить MCAP."""
    arguments = _arguments()
    config = _load_config(arguments.config)

    output_root = Path(
        os.path.expandvars(
            os.path.expanduser(str(config["output_root"]))
        )
    )
    output_root.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S_%fZ"
    )
    experiment_id = f"{config['directory_prefix']}_{timestamp}"
    experiment_directory = output_root / experiment_id
    bag_directory = experiment_directory / "bag"
    snapshot_path = experiment_directory / "recording.yaml"
    experiment_directory.mkdir(exist_ok=False)

    use_sim_time = _resolve_sim_time(config["use_sim_time"])
    started_at = datetime.now(timezone.utc).isoformat()
    _write_recording_snapshot(
        path=snapshot_path,
        config=config,
        experiment_id=experiment_id,
        started_at=started_at,
        finished_at=None,
        status="recording",
        use_sim_time=use_sim_time,
        return_code=None,
    )

    command = _build_command(config, bag_directory, use_sim_time)
    print(f"Запись: {experiment_directory}")
    print("Корректное завершение и финализация MCAP: Ctrl+C")

    process = subprocess.Popen(command, start_new_session=True)
    return_code = 1
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        os.killpg(process.pid, signal.SIGINT)
        return_code = process.wait()
    finally:
        finished_at = datetime.now(timezone.utc).isoformat()
        status = (
            "finished"
            if return_code in (0, 130)
            else "failed"
        )
        _write_recording_snapshot(
            path=snapshot_path,
            config=config,
            experiment_id=experiment_id,
            started_at=started_at,
            finished_at=finished_at,
            status=status,
            use_sim_time=use_sim_time,
            return_code=return_code,
        )

        if bag_directory.exists():
            with (
                experiment_directory / "bag_info.txt"
            ).open("w", encoding="utf-8") as stream:
                subprocess.run(
                    ["ros2", "bag", "info", str(bag_directory)],
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    check=False,
                    text=True,
                )

    print(f"Сохранено: {experiment_directory}")
    if return_code not in (0, 130):
        raise SystemExit(return_code)


if __name__ == "__main__":
    main()
