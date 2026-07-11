from setuptools import find_packages, setup

package_name = 'go2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjg',
    maintainer_email='wjg@todo',
    description='cmd_vel to Go2 sport API bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'go2_bridge = go2_bridge.bridge:main',
        ],
    },
)
