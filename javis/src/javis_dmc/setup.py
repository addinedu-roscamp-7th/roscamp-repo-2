import os
from glob import glob

from setuptools import find_packages, setup

package_name = "javis_dmc"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Test GUI plugin
        (os.path.join("share", package_name, "test_gui"), glob("test_gui/*.py")),
        #     (os.path.join('share', package_name),
        #         ['test_gui/plugin.xml']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kim jong myung",
    maintainer_email="jongbob1918@gmail.com",
    description="Dobby Main Controller",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dmc_node = javis_dmc.dmc_node:main",
            # 'test_gui = javis_dmc.test_gui:main',
        ],
    },
)
