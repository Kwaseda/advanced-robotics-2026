from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'r7021e_mpc_bringup'

# The parameter files are real files inside this package's own config/ directory, not a
# symlink to a shared one. A shared directory reached by a symlink can silently install
# empty, and no unit test notices: only a launch plus "ros2 param get" does.
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='domadd-2@student.ltu.se',
    description='R7021E Lab 2 launch files, parameters and RViz configuration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
