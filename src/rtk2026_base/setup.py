from setuptools import find_packages, setup

package_name = "rtk2026_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/base_controller.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="user@example.com",
    description="Base controller: cmd_vel to motors, encoders to odometry",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_controller = rtk2026_base.base_controller_node:main",
        ],
    },
)
