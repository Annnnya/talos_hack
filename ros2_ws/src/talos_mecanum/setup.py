from setuptools import find_packages, setup

package_name = "talos_mecanum"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/robot.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Talos team",
    maintainer_email="robotics@example.com",
    description="ROS 2 mecanum base driver and keyboard teleoperation for Talos.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "drive_node = talos_mecanum.drive_node:main",
            "keyboard_teleop = talos_mecanum.keyboard_teleop_node:main",
        ],
    },
)
