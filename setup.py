from glob import glob
import os

from setuptools import find_packages, setup

package_name = "ros2_robot_basics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yanxin Chen",
    maintainer_email="Yanxin.chen@rwth-aachen.de",
    description=(
        "Minimal ROS2 project demonstrating topic communication, "
        "PD control, joint simulation, rosbag2 recording, and plotting."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "target_joint_publisher = publisher_node.target_joint_publisher:main",
            "joint_pd_simulator = controller_node.joint_pd_simulator:main",
        ],
    },
)
