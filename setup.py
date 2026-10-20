import os
from glob import glob

from setuptools import setup

package_name = "fold_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="ROS 2 bridge for agx_motor_sdk ArmJointGroup (fold, arm protocol)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fold_node = fold_ros.fold_node:main",
        ],
    },
)
