from setuptools import find_packages, setup

package_name = "rtk2026_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/arduino_bridge.yaml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="user@example.com",
    description="Arduino serial bridge for RTK2026",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arduino_bridge = rtk2026_driver.arduino_bridge_node:main",
            "fake_encoder = rtk2026_driver.fake_encoder_node:main",
        ],
    },
)
