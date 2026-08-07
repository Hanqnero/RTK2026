from setuptools import find_packages, setup

package_name = "rtk2026_city_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RTK2026",
    maintainer_email="kamilisxakof@gmail.com",
    description="Движение по городу: полосы, маневры, выбор пути по знакам.",
    license="Apache-2.0",
)
