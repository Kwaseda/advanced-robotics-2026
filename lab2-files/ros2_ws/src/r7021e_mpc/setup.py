from setuptools import find_packages, setup

package_name = 'r7021e_mpc'

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
    description='R7021E Lab 2: Model Predictive Control for the TurtleBot3 Burger',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = r7021e_mpc.mpc_node:main',
            'trajectory_node = r7021e_mpc.trajectory_node:main',
            'goal_marker_node = r7021e_mpc.goal_marker_node:main',
        ],
    },
)
