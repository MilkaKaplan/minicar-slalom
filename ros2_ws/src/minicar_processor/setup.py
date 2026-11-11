from setuptools import find_packages, setup

package_name = "minicar_processor"

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
    description="Finds the posts on the photo, plans the slalom path (there, U-turn, back) and checks it.",
    license="MIT",
    entry_points={"console_scripts": [
        "lidar_mapper_node = minicar_processor.lidar_mapper_node:main",
    ]},
)
