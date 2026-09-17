from glob import glob

from setuptools import setup


package_name = "evolo_bearing_config"


setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
        (
            f"share/{package_name}/launch",
            glob("launch/*.launch.py"),
        ),
        (
            f"share/{package_name}/config/experiments",
            glob("config/experiments/*.yaml"),
        ),
        (
            f"share/{package_name}/config/rviz/rviz",
            glob("config/rviz/rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axel Ericson Holmgren",
    maintainer_email="axelholmgren@users.noreply.github.com",
    description=(
        "Higher-level configuration for Evolo bearing tracking."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
)
