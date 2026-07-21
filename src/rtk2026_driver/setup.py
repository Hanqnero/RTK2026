from glob import glob
from setuptools import find_packages, setup

package_name = "rtk2026_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # маркер для ament — обязателен для обнаружения пакета
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # конфиг устанавливается в share — launch файл найдёт его через get_package_share_directory
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            # Имя команды ведёт на фактический модуль Arduino bridge.
            "arduino_bridge = rtk2026_driver.arduino_bridge:main",
        ],
    },
)
