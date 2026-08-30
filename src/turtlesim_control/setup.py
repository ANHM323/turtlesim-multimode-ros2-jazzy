from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlesim_control'

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
    maintainer='andyh',
    maintainer_email='andyh@example.com',
    description='Controlador multi-modo para turtlesim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mode_controller = turtlesim_control.mode_controller:main',
        ],
    },
)
