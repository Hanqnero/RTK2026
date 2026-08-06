from glob import glob

from setuptools import setup

package_name = "rtk2026_vector_objects"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    py_modules=["keepout_click_tool"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="kamilisxakof@gmail.com",
    description="Обход препятствий через Nav2 VectorObjectServer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keepout_click_tool = keepout_click_tool:main",
        ]
    },
)
