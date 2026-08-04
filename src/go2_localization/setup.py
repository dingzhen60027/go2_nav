from glob import glob
from setuptools import find_packages, setup


package_name = "go2_localization"

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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wjg",
    maintainer_email="wjg@example.com",
    description="Dual-EKF localization pipeline for Go2 and MID360.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sport_state_adapter = go2_localization.sport_state_adapter:main",
            "icp_fusion_bridge = go2_localization.icp_fusion_bridge:main",
        ],
    },
)
