from glob import glob

from setuptools import find_packages, setup

package_name = "rtk2026_cv"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", glob("*.onnx")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="kamilisxakof@gmail.com",
    description="Traffic sign and bus detection adapter for RTK2026.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_sign_adapter = rtk2026_cv.yolo_sign_adapter_node:main",
            "onnx_sign_detector = rtk2026_cv.onnx_sign_detector_node:main",
        ]
    },
)
