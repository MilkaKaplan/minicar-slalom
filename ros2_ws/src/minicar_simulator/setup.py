from setuptools import find_packages, setup

package_name = "minicar_simulator"

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
    description="Simulated car (kinematic bicycle + pure pursuit) that drives the planned slalom path.",
    license="MIT",
    entry_points={"console_scripts": ["simulator_node = minicar_simulator.simulator_node:main"]},
)
