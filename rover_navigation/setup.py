from setuptools import setup
import os
from glob import glob

package_name = "rover_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Antony Juza",
    maintainer_email="antonyjuza@gmail.com",
    description="Ultrasonic navigation for fire-rescue rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ultrasonic_sensor_node = rover_navigation.ultrasonic_sensor_node:main",
            "navigation_node        = rover_navigation.navigation_node:main",
        ],
    },
)