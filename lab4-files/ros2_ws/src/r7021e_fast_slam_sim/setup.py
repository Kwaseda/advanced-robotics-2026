from setuptools import setup

package_name = 'r7021e_fast_slam_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/maze_sim.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/maze.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='addodominique@gmail.com',
    description='Gazebo maze for Lab 4.',
    license='MIT',
)
