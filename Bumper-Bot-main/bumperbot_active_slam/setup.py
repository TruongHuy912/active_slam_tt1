from glob import glob
import os

from setuptools import find_packages, setup


package_name = "bumperbot_active_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bumper-Bot Active SLAM Maintainer",
    maintainer_email="user@example.com",
    description="Phase 1 Active SLAM frontier detection baseline for Bumper-Bot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "active_slam_explorer = bumperbot_active_slam.active_slam_node:main",
        ],
    },
)
