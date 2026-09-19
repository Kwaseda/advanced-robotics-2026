"""One launch file for all four tasks of Lab 1.

    ros2 launch r7021e_bringup lab1.launch.py                      task 1, terminal setpoints
    ros2 launch r7021e_bringup lab1.launch.py trajectory:=true     tasks 2 and 3, the eight
    ros2 launch r7021e_bringup lab1.launch.py mode:=wall_following task 4

Arguments, all of them things that change between a desk and a lab bench:

    mode          position_control | wall_following. Which node drives /cmd_vel.
    trajectory    start the figure of eight generator. Ignored in wall_following mode.
    domain_id     ROS_DOMAIN_ID for every node started here. The lab dictates 3<robot
                  number>, so turtle4 is 34. A flag, never an edit to a file.
    use_sim_time  true in Gazebo, false on the robot. Set once, here, for every node.
    rviz          start RViz2 with the saved configuration.
    sim           include the Gazebo world. Off by default.
    world_launch  which world. Defaults to turtlebot3_dqn_stage1.launch.py, the stock
                  walled square with the robot at its centre. See the note below on
                  box.launch.py, which the tutorial names and this installation lacks.
    config_dir    where the parameter files are. Defaults to this package's share.

Why mode is an argument and not two launch files: controller_node and
wall_follower_node both publish to /cmd_vel, and two publishers on one topic do not
error. They interleave, and the robot does something that looks like a tuning problem
and is not. Making the choice an argument means it cannot be made by accident.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'r7021e_bringup'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    arguments = [
        DeclareLaunchArgument(
            'mode', default_value='position_control',
            choices=['position_control', 'wall_following'],
            description='which node drives /cmd_vel'),
        DeclareLaunchArgument(
            'trajectory', default_value='false',
            choices=['true', 'false'],
            description='publish the figure of eight on /new_position'),
        DeclareLaunchArgument(
            'domain_id', default_value='34',
            description='ROS_DOMAIN_ID, the lab uses 3<robot number>. turtle4 is 34'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            choices=['true', 'false'],
            description='true in Gazebo, false on the robot'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            choices=['true', 'false'],
            description='start RViz2 with the saved Lab 1 configuration'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            choices=['true', 'false'],
            description='include the Gazebo world named by world_launch'),
        DeclareLaunchArgument(
            'world_launch', default_value='turtlebot3_dqn_stage1.launch.py',
            description='launch file inside turtlebot3_gazebo that starts the world. '
                        'The tutorial names box.launch.py, which ships with the course '
                        'virtual machine and not with the upstream package. See the note '
                        'further down this file'),
        DeclareLaunchArgument(
            'config_dir', default_value=os.path.join(share, 'config'),
            description='directory holding robot.yaml and the per-node parameter files'),
    ]

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('config_dir')

    # ROS_DOMAIN_ID is set for the processes this file starts. It does not reach the
    # shell the launch was typed in, so a "ros2 topic list" in another terminal still
    # needs its own export. Step one of the lab-day checklist in the package README.
    domain = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

    def parameters(node_yaml):
        """robot.yaml first, then the node's own file.

        Order matters. Later files win on a repeated key, so a node file could
        override a physical limit if it declared one. None of them do, and none of
        them should: config/robot.yaml owns those numbers. Loading it first and
        listing it on every node is what makes one definition serve four nodes.
        """
        return [
            PathJoinSubstitution([config_dir, 'robot.yaml']),
            PathJoinSubstitution([config_dir, node_yaml]),
            {'use_sim_time': use_sim_time},
        ]

    driving = IfCondition(PythonExpression(['"', mode, '" == "position_control"']))
    following = IfCondition(PythonExpression(['"', mode, '" == "wall_following"']))
    eight = IfCondition(PythonExpression([
        '"', mode, '" == "position_control" and "',
        LaunchConfiguration('trajectory'), '" == "true"',
    ]))

    nodes = [
        Node(
            package='r7021e_control', executable='controller_node',
            name='controller_node', output='screen',
            parameters=parameters('controller.yaml'), condition=driving),
        Node(
            package='r7021e_control', executable='trajectory_node',
            name='trajectory_node', output='screen',
            parameters=parameters('trajectory.yaml'), condition=eight),
        Node(
            package='r7021e_control', executable='wall_follower_node',
            name='wall_follower_node', output='screen',
            parameters=parameters('wall_follower.yaml'), condition=following),
        # The scan monitor runs in both modes. Task 3 wants the closest wall point
        # plotted while the eight runs, and the wall following segments have to be
        # marked on the same plot, so it has to be recording during task 4 as well.
        Node(
            package='r7021e_control', executable='scan_monitor_node',
            name='scan_monitor_node', output='screen',
            parameters=parameters('scan_monitor.yaml')),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'lab1.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # The simulator.
    #
    # The tutorial, slide 23, says to launch box.launch.py from turtlebot3_gazebo, and
    # describes the result as "the robot in the center with red-colored walls around
    # it". That file does not exist in this installation and is not in the upstream
    # ROBOTIS turtlebot3_simulations jazzy branch, which is what is built in
    # ~/turtlebot3_ws. The tutorial is written for the course virtual machine, which
    # ships its own checkout.
    #
    # The stock world that matches the description is turtlebot3_dqn_stage1: a ground
    # plane plus turtlebot3_dqn_world, which is a 5 m square of walls whose inner faces
    # sit at +/- 2.35 m, with the robot spawned at the origin. Read out of the model
    # SDF on 2026-09-08. The walls are white rather than red, which is the only
    # difference that matters and it is cosmetic.
    #
    # world_launch is therefore an argument, not a constant, so switching to
    # box.launch.py is a flag on the command line if the lab machines turn out to have
    # it.
    # ASSUMPTION: turtlebot3_dqn_stage1 is an acceptable stand-in for the box world.
    
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # FindPackageShare, not get_package_share_directory. The function would
            # look the package up while this file is being read, so a machine without
            # the simulator could not launch the nodes at all. The substitution is
            # resolved only when the action runs, which the condition below stops it
            # from doing.
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch',
                LaunchConfiguration('world_launch'),
            ])
        ),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription(arguments + [domain] + nodes + [rviz, world])
