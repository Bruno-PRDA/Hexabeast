from setuptools import find_packages, setup

package_name = "hexapod_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bruno Pereira Azevedo",
    maintainer_email="icato0912@gmail.com",
    description="Tripod gait and leg IK for the hexapod, as a ROS 2 node.",
    license="All rights reserved",
    entry_points={"console_scripts": ["gait_node = hexapod_gait.gait_node:main"]},
)
