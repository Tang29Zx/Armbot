from glob import glob
import os

from setuptools import find_packages, setup


package_name = "vla_runtime"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tang29Zx",
    maintainer_email="212705023+Tang29Zx@users.noreply.github.com",
    description="DDS-to-OpenPI bridge for the Armbot VLA runtime",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vla_bridge = vla_runtime.vla_bridge_node:main",
        ],
    },
)
