from glob import glob
from setuptools import find_packages, setup


package_name = "evolo_reference_markers"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axel Ericson Holmgren",
    maintainer_email="axelholmgren@users.noreply.github.com",
    description="Fixed and Smarcduino position marker publishers for Evolo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fixed_position_marker_node = evolo_reference_markers.fixed_position_marker_node:main",
            "smarcduino_marker_node = evolo_reference_markers.smarcduino_marker_node:main",
            "smarcduino_waraps_position_marker_node = evolo_reference_markers.smarcduino_waraps_position_marker_node:main",
        ],
    },
)
