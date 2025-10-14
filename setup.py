from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lg_teleop_device'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamgrit',
    maintainer_email='teamgrit@example.com',
    description='Description of the lg_teleop_device package',
    license='License declaration',
#    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_media_node = lg_teleop_device.media.topic_media_node:main',
            'lidar_node = lg_teleop_device.lidar.lidar_node:main',
            'arm_control_node = lg_teleop_device.control.arm_control_node:main',
            'head_control_node = lg_teleop_device.control.head_control_node:main',
            'twist_control_node = lg_teleop_device.control.twist_control_node:main',
            'gamepad_control_node = lg_teleop_device.control.gamepad_control_node:main',
            'skeleton_gamepad_control_node = lg_teleop_device.control.skeleton_gamepad_control_node:main',
            'joint_states_node = lg_teleop_device.sensor.joint_states_node:main',
            'joint_states_subscription_node = lg_teleop_device.sensor.joint_states_subscription_node:main',
            'network_node = lg_teleop_device.network.network:main',
            'waist_control_node = lg_teleop_device.control.waist_control_node:main',
        ],
    },
)
