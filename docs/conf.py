"""Конфигурация Sphinx для документации RTK2026."""

from __future__ import annotations

import inspect
import os
import sys
from pathlib import Path
from typing import Any


DOCS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = DOCS_DIR.parent

# Python-пакеты ROS и автономные диагностические скрипты документируются
# непосредственно из рабочего дерева, поэтому сборка документации не требует
# предварительного colcon install.
sys.path.insert(0, str(PROJECT_ROOT / "src" / "rtk2026_driver"))
sys.path.insert(0, str(PROJECT_ROOT / "src" / "rtk2026_description" / "scripts"))
sys.path.insert(0, str(PROJECT_ROOT / "arduino"))

project = "RTK2026"
author = "RTK2026 Team"
copyright = "2026, RTK2026 Team"
release = "0.1.0"
language = "ru"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.linkcode",
    "sphinx.ext.mathjax",
    "sphinx.ext.napoleon",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
]

autosummary_generate = True
autodoc_typehints = "description"
autodoc_member_order = "bysource"
autoclass_content = "both"
add_module_names = False

# Позволяет строить API-справочник на ПК без установленного ROS 2. Реальные
# импорты проекта protocol.py и transport.py при этом сохраняются там, где их
# зависимости доступны.
autodoc_mock_imports = [
    "ament_index_python",
    "geometry_msgs",
    "launch",
    "launch_ros",
    "nav_msgs",
    "rclpy",
    "sensor_msgs",
    "serial",
    "tf2_ros",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

rst_prolog = """
.. |cmd_vel| replace:: ``/cmd_vel``
.. |odom| replace:: ``/odom``
.. |wheel_odom| replace:: ``/wheel/odom``
.. |scan| replace:: ``/scan``
.. |base_footprint| replace:: ``base_footprint``
"""

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}

# Ссылки на исходники этой же ветки могут ещё не существовать на GitHub до
# публикации текущего коммита. Их корректность задаётся единым SOURCE_REF и не
# должна мешать проверке внешней документации через ``make linkcheck``.
linkcheck_ignore = [
    r"https://github\.com/Hanqnero/RTK2026/(?:blob|tree)/.*",
    # GitHub периодически отвечает linkcheck rate limit, хотя ссылка ведёт на
    # официальный шаблон robot_localization и остаётся полезной читателю.
    r"https://github\.com/cra-ros-pkg/robot_localization/blob/.*",
    r"http://127\.0\.0\.1:6080/.*",
    r"http://127\.0\.0\.1:6081/.*",
]

todo_include_todos = True

html_theme = "alabaster"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_title = "RTK2026 — техническая документация"
html_show_sourcelink = True
html_theme_options = {
    "description": "Прошивка, ROS 2, SLAM, описание робота и симуляция",
    "fixed_sidebar": True,
    "github_user": "Hanqnero",
    "github_repo": "RTK2026",
    "github_button": True,
}

REPOSITORY_URL = "https://github.com/Hanqnero/RTK2026"
SOURCE_REF = os.environ.get("RTK2026_DOCS_SOURCE_REF", "main")


def linkcode_resolve(domain: str, info: dict[str, str]) -> str | None:
    """Вернуть GitHub-ссылку на Python-объект, показанный autodoc."""

    if domain != "py" or not info.get("module"):
        return None

    module = sys.modules.get(info["module"])
    if module is None:
        return None

    obj: Any = module
    for component in info.get("fullname", "").split("."):
        obj = getattr(obj, component, None)
        if obj is None:
            return None

    try:
        source_file = inspect.getsourcefile(obj) or inspect.getsourcefile(module)
        source_lines, start_line = inspect.getsourcelines(obj)
    except (OSError, TypeError):
        return None

    if source_file is None:
        return None

    try:
        relative_path = Path(source_file).resolve().relative_to(PROJECT_ROOT)
    except ValueError:
        return None

    end_line = start_line + len(source_lines) - 1
    return (
        f"{REPOSITORY_URL}/blob/{SOURCE_REF}/{relative_path.as_posix()}"
        f"#L{start_line}-L{end_line}"
    )
