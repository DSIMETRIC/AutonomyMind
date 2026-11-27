from setuptools import find_packages, setup

package_name="autonomymind_sensor_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Dsimetric",
    maintainer_email="dev@dsimetric.com",
    description="Camera-LiDAR fusion for AutonomyMind.",
    license="Apache-2.0",
    entry_points={
        "console_scripts":[
            "camera_lidar_fusion = autonomymind_sensor_fusion.camera_lidar_fusion:main",
        ],
    },
)
