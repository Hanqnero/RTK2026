import os
from glob import glob

from setuptools import find_packages, setup


package_name = "rtk2026_mearm"

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
]

vendor_files = glob(os.path.join("..", "..", "vendor", "mearm", "*.py"))
if vendor_files:
    data_files.append(
        (os.path.join("share", package_name, "vendor", "mearm"), vendor_files)
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026 Team",
    maintainer_email="kamilisxakof@gmail.com",
    description="ROS2 wrapper for the vendor meArm Python control library",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mearm_node = rtk2026_mearm.mearm_node:main",
        ],
    },
)
