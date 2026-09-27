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
            # Every entry point here names a file that exists.
            #
            # The follower is ours rather than the course's, and the launch file
            # takes `follower:=course` to run the supplied one instead. Both are
            # kept so the two can be compared in a single run of the stack.
            'navigation_node = r7021e_rrt.navigation_node:main',
            'path_follower_node = r7021e_rrt.path_follower_node:main',
        ],
    },
)
