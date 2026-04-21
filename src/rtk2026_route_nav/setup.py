from glob import glob

from setuptools import setup

package_name = "rtk2026_route_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    py_modules=[
        "keepout_click_tool",
        "lane_decision_manager_v3",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rtk2026",
    maintainer_email="todo@todo.com",
    description="Nav2 route_server: конфиг, граф, launch.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keepout_click_tool = keepout_click_tool:main",
            "lane_decision_manager_v3 = lane_decision_manager_v3:main",
        ]
    },
)
