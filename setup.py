import os
from glob import glob

from setuptools import setup

package_name = 'rosetta_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Blankenau',
    maintainer_email='isaac.blankenau@gmail.com',
    description='Bringup launch files for the Rosetta simulation stack',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
