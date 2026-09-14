from glob import glob
from setuptools import find_packages, setup


package_name = "evolo_bearing"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["evolo_bearing"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axel Ericson Holmgren",
    maintainer_email="axelholmgren@users.noreply.github.com",
    description="Calibrated gimbal yaw correction and bearing-ray visualisation for Evolo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bearing_marker_ids_node = evolo_bearing.bearing_marker_ids_node:main",
            "bearing_marker_node = evolo_bearing.bearing_marker_node:main",
        ],
    },
)
