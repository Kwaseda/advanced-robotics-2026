from setuptools import find_packages, setup

package_name = 'r7021e_rrt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='domadd-2@student.ltu.se',
    description='R7021E Lab 3: RRT* planning and frontier-based exploration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # One node, one entry point, one file that exists. Last year's lab2_pkg
            # registered five entry points for files that were not there, which builds
            # clean and fails at run time.
            'navigation_node = r7021e_rrt.navigation_node:main',
        ],
    },
)
