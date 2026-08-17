from setuptools import setup

package_name = "rtk2026_nav2_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/rtk2026_nav2_slam.launch.py",
                "launch/rtk2026_explorer.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/nav2_params_slam.yaml",
                "config/slam_toolbox_autorace.yaml",
            ],
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="kamilisxakof@gmail.com",
    description="Frontier-based autonomous exploration node for RTK2026 (Nav2 + SLAM).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explorer = rtk2026_nav2_explorer.explorer_node:main",
        ],
    },
)

