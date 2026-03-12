from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rtk2026_peripherals"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="user@example.com",
    description="2D lidar, stereo/depth camera, IMU for RTK2026",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_broadcaster_imu = rtk2026_peripherals.tf_broadcaster_imu:main",
            "fake_scan = rtk2026_peripherals.fake_scan_node:main",
            "odom_tf_broadcaster = rtk2026_peripherals.odom_tf_broadcaster_node:main",
            "clock_publisher = rtk2026_peripherals.clock_publisher_node:main",
            "static_map_publisher = rtk2026_peripherals.static_map_publisher_node:main",
            "static_odom_tf_publisher = rtk2026_peripherals.static_odom_tf_node:main",
            "trigger_nav2_bringup = rtk2026_peripherals.trigger_nav2_bringup_node:main",
        ],
    },
)
