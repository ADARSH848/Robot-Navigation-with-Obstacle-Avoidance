from setuptools import setup
import os
from glob import glob

package_name = 'navigation_assignment'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Path Smoothing and Control Assignment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_generator = navigation_assignment.path_generator_node:main',
            'controller = navigation_assignment.controller_node:main',
            'site_loader = navigation_assignment.construction_site_loader:main',
        ],
    },
)
