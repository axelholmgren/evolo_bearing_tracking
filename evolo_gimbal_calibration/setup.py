from setuptools import find_packages, setup


package_name = "evolo_gimbal_calibration"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name]),
    package_data={package_name: ["*.csv", "*.txt", "*.png", "*.zip"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Axel Ericson Holmgren",
    maintainer_email="axelholmgren@users.noreply.github.com",
    description="Gimbal yaw calibration model and experiment artifacts for Evolo.",
    license="Apache-2.0",
)
