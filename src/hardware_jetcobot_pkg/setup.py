from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hardware_jetcobot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetcobot_control = hardware_jetcobot_pkg.jetcobot_hardware_control:main',
            'set_target_pos = hardware_jetcobot_pkg.set_target_pos:main',
            'joint_control = hardware_jetcobot_pkg.joint_control:main',
            'joint_state_switcher = hardware_jetcobot_pkg.joint_state_switcher:main',
        ],
    },
)
