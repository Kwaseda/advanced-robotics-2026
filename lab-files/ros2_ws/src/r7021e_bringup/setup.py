from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'r7021e_bringup'

# Parameter files live at the repository root, in config/, not inside this package --
# one source of truth for a workspace that may hold more packages later. config/ here
# is a symlink to that directory; this glob follows it, so ament_index finds the files
# through the installed package share without knowing where the repo is checked out.
# Needs a filesystem with symlinks (not /mnt/c on WSL).

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
    description='R7021E launch files, parameters and RViz configuration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
