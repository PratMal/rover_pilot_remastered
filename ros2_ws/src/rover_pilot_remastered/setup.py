from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_pilot_remastered'

setup(
    name=package_name,
    version='2.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/wheels', glob('config/wheels/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruce',
    maintainer_email='diveshkumar_23ee078@dtu.ac.in',
    description='Rover drive control: PS5 teleop with a latched e-stop, /cmd_vel to the drive ESP32.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drive_node = rover_pilot_remastered.drive_node:main',
            'base_teleop = rover_pilot_remastered.base_teleop:main',
            'fake_esp = rover_pilot_remastered.fake_esp:main',
        ],
    },
)
