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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "test_gui"), glob("test_gui/*.py")),
    ],
    # 실행에 필요한 패키지를 추가합니다
    install_requires=["setuptools", "rclpy"],
    zip_safe=True,
    maintainer="kim jong myung",
    maintainer_email="jongbob1918@gmail.com",
    description="Dobby Main Controller",
    license="Apache-2.0",
    # package.xml과 테스트 의존성을 일치시킵니다
    tests_require=[
        "pytest",
        "pytest-cov",
        "pytest-mock",
        "ament_copyright",
        "ament_flake8",
        "ament_pep257",
    ],
    entry_points={
        "console_scripts": [
            "dmc_node = javis_dmc.dmc_node:main",
        ],
    },
)