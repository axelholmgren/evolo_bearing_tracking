from setuptools import find_packages, setup


package_name = "evolo_bearing_error"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axel Ericson Holmgren",
    maintainer_email="axelholmgren@users.noreply.github.com",
    description="Bearing-error geometry and CSV logging for Evolo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bearing_error_node = evolo_bearing_error.bearing_error_node:main",
        ],
    },
)
