from glob import glob

from setuptools import find_packages, setup

package_name = "rtk2026_city_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Граф ставится наравне с параметрами: на него по умолчанию
        # ссылается лаунч, и без geojson в маске путь указывал бы
        # на файл, которого в установке нет.
        (
            f"share/{package_name}/config",
            glob("config/*.yaml") + glob("config/*.geojson"),
        ),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="kamilisxakof@gmail.com",
    description="Движение по городу: полосы, маневры, выбор пути по знакам.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"city_nav_node = {package_name}.node:main",
            # Проверки до выезда. ROS им не нужен, поэтому запускаются и там,
            # где стек не поднят.
            f"city_nav_check = {package_name}.cli:main_check",
            f"city_nav_poses = {package_name}.cli:main_poses",
        ],
    },
)
