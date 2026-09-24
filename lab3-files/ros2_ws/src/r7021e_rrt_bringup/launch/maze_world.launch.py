"""Gazebo side of Lab 3: a generated maze, a Burger in it, and the ros_gz bridge.

Separate from lab3.launch.py so that the simulator can be started, inspected and
killed on its own. `gz sim`'s server and GUI do not die on Ctrl-C, confirmed on
every run tested, so being able to bring the world up without the whole stack is
what makes a clean teardown checkable:

    ros2 launch r7021e_rrt_bringup maze_world.launch.py
    ps aux | grep -E "[g]z sim|[g]zserver" | grep -v grep

Both worlds were produced by the course's maze generator, `map_gen.py`, with its
hardcoded output directory pointed at this package's worlds/ instead:

    lab3_maze_small.world   -s 5,5 -o=-2,-2 -l 0.8   4.0 by 4.0 m, 25 cells
    lab3_maze.world         -s 9,9 -o=-3.6,-3.6 -l 0.8   7.2 by 7.2 m, 81 cells

Both are sized so the maze cell containing the world origin spans -0.4 to 0.4 m in
both axes, which is why the robot spawns at (0, 0) with about 0.35 m of clearance
on every side. Get the origin wrong and the robot spawns inside a wall.

The 0.8 m cell is a guess at the Concept Lab maze. Ask the TA for the real cell
size and outer dimensions before the session and regenerate at those numbers; the
commands are above.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'r7021e_rrt_bringup'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    # FindPackageShare, not get_package_share_directory, for the two packages that
    # live outside this workspace. The substitution resolves when the action runs
    # rather than when the file is parsed, so a machine with no simulator can still
    # launch the rest of the stack.
    turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='lab3_maze_small',
            description='world file in this package, without the .world suffix. '
                        'lab3_maze_small for iteration, lab3_maze for the run'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='start the Gazebo GUI. false for a headless parameter sweep'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='spawn x. Both generated mazes have a free cell at the origin'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0', description='spawn y'),
    ]

    world = PathJoinSubstitution(
        [share, 'worlds', [LaunchConfiguration('world'), '.world']])

    # The maze's own models are boxes defined inline, but the world includes the
    # Fuel ground plane, and the Burger's SDF references meshes under
    # turtlebot3_gazebo/models. Without this the robot spawns as an invisible
    # collision hull and the laser reads a world with no floor.
    resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([turtlebot3_gazebo, 'models']))

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Reused rather than reimplemented: these two publish the Burger's URDF and
    # start the ros_gz bridge that carries /scan, /odom, /cmd_vel and the clock.
    # The bridge is the reason `use_sim_time` works at all downstream.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'])),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )

    # Deliberately no second clock bridge here. spawn_turtlebot3.launch.py already
    # bridges /clock via turtlebot3_burger_bridge.yaml, and adding another
    # parameter_bridge for it puts two publishers on /clock, each with its own view
    # of simulator time. A node on sim time then sees the clock step backwards, tf
    # reports "Detected jump back in time. Clearing TF buffer", and the SLAM map
    # comes out smeared into doubled walls, which looks exactly like a scan
    # matching problem and is not.

    return LaunchDescription(
        arguments + [resources, server, gui, robot_state_publisher, spawn])
