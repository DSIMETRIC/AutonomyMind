from setuptools import find_packages, setup

package_name = "autonomymind_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dsimetric",
    maintainer_email="dev@dsimetric.com",
    description="Camera and LiDAR adapters for AutonomyMind Observation contracts.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_detection_adapter = autonomymind_perception.camera_detection_adapter:main",
            "lidar_scan_adapter = autonomymind_perception.lidar_scan_adapter:main",
        ],
    },
)
