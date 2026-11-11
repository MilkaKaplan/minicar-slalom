from setuptools import find_packages, setup

package_name = "minicar_publisher"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MilkaKaplan",
    description="Reads the top-view photo of the slalom track and publishes it as sensor_msgs/Image.",
    license="MIT",
    entry_points={"console_scripts": [
        "lidar_replay_node = minicar_publisher.lidar_replay_node:main",
    ]},
)
