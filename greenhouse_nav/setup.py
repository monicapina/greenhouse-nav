from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'greenhouse_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monica',
    maintainer_email='monicapinanavarro@gmail.com',
    description='Greenhouse boustrophedon navigation with TurtleBot3 and Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = greenhouse_nav.mission_node:main',
        ],
    },
)