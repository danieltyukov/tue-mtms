from setuptools import find_packages, setup
import os
from glob import glob

package_name = "movement_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="matthijs",
    maintainer_email="matthijs_smulders@kpnmail.nl",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "movement_manager = movement_manager.movement_manager:main",
            "drive_manager = movement_manager.drive_manager:main",
            "morph_manager = movement_manager.morph_manager:main",
        ],
    },
)
