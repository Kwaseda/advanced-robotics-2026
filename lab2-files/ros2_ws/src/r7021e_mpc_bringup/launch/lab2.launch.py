"""One launch file for all four tasks of Lab 2, Model Predictive Control.

    ros2 launch r7021e_mpc_bringup lab2.launch.py                 task 1, setpoint tracking
    ros2 launch r7021e_mpc_bringup lab2.launch.py task:=2         one static obstacle
    ros2 launch r7021e_mpc_bringup lab2.launch.py task:=3         two static obstacles
    ros2 launch r7021e_mpc_bringup lab2.launch.py task:=4         circle plus one obstacle

Add sim:=true for Gazebo and rviz:=true to watch. For tasks 1 to 3 the setpoint is
published by hand, repeated rather than once, because a single --once publish can be lost
to discovery before a bag recorder has matched the new publisher:

    ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose \\
      "{position: {x: 0.8, y: 0.5, z: 0.0}}"

Task 4 needs no typed setpoint: trajectory_node publishes the circle on /new_position and
its future on /reference_path.

Arguments:

    task          1 | 2 | 3 | 4. Selects config/mpc_task<N>.yaml and, for 4, starts the
                  circular trajectory generator.
    t_step        Overrides mpc_node's t_step parameter (default from mpc.yaml, 0.1 s).
    n_horizon     Overrides mpc_node's n_horizon parameter (default from mpc.yaml, 20).
    domain_id     ROS_DOMAIN_ID for every node started here. The lab dictates
                  3<robot number>, so turtle4 is 34.
    use_sim_time  true in Gazebo, false on the robot.
    rviz          start RViz2 with the saved Lab 2 configuration.
    sim           include the Gazebo world named by world_launch.
    world_launch  which world, inside turtlebot3_gazebo.
    config_dir    where the parameter files are. Defaults to this package's share.

The task is an argument rather than four launch files because the obstacle set, the
boundary box and the trajectory all have to agree with each other, and four files each
repeating three of those numbers is four places for them to drift apart.
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'r7021e_mpc_bringup'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    arguments = [
        DeclareLaunchArgument(
            'task', default_value='1',
            choices=['1', '2', '3', '4'],
            description='1 setpoint tracking, 2 one obstacle, 3 two obstacles, '
                        '4 circular trajectory with an obstacle'),
        DeclareLaunchArgument(
            't_step', default_value='0.1',
            description="Overrides mpc_node's t_step parameter, seconds. Default matches "
                        'mpc.yaml'),
        DeclareLaunchArgument(
            'n_horizon', default_value='20',
            description="Overrides mpc_node's n_horizon parameter, steps. Default matches "
                        'mpc.yaml'),
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
            description='start RViz2 with the saved Lab 2 configuration'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            choices=['true', 'false'],
            description='include the Gazebo world named by world_launch'),
        DeclareLaunchArgument(
            'world_launch', default_value='turtlebot3_dqn_stage1.launch.py',
            description='launch file inside turtlebot3_gazebo that starts the world'),
        DeclareLaunchArgument(
            'config_dir', default_value=os.path.join(share, 'config'),
            description='directory holding robot.yaml and the per-task parameter files'),
    ]

    task = LaunchConfiguration('task')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('config_dir')

    # Set for the processes this file starts. A "ros2 topic list" in another terminal
    # still needs its own export.
    domain = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

    # Four entries, in this order, because later ones win on a repeated key: robot.yaml
    # (physical limits), mpc.yaml (shared defaults), mpc_task<N>.yaml (deltas), then the
    # t_step/n_horizon override dict last. All four target the /** wildcard scope (see
    # robot.yaml), not /mpc_node explicitly: ROS 2 lets an exact node-name match win over
    # /** regardless of file order, so if the override dict here (always written to a /**
    # params file by launch_ros) disagreed in scope with the others it would silently lose
    # even though it is last. The overlay can widen the boundary and add obstacles; it
    # cannot contradict a physical limit, because the node takes the min() of the lab bound
    # and the robot bound rather than whichever it read last.
    mpc_parameters = [
        PathJoinSubstitution([config_dir, 'robot.yaml']),
        PathJoinSubstitution([config_dir, 'mpc.yaml']),
        PathJoinSubstitution([config_dir, ['mpc_task', task, '.yaml']]),
        {
            'use_sim_time': use_sim_time,
            't_step': ParameterValue(LaunchConfiguration('t_step'), value_type=float),
            'n_horizon': ParameterValue(LaunchConfiguration('n_horizon'), value_type=int),
        },
    ]

    circling = IfCondition(PythonExpression(['"', task, '" == "4"']))

    nodes = [
        Node(
            package='r7021e_mpc', executable='mpc_node',
            name='mpc_node', output='screen',
            parameters=mpc_parameters),
        # Task 4 only. For tasks 1 to 3 the setpoint is typed in a terminal and this node
        # would only fight it.
        Node(
            package='r7021e_mpc', executable='trajectory_node',
            name='trajectory_node', output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, 'robot.yaml']),
                PathJoinSubstitution([config_dir, 'circle.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            condition=circling),
        # /new_position has no header, so RViz cannot draw it directly. Runs
        # unconditionally: with nothing on /new_position it has nothing to draw.
        Node(
            package='r7021e_mpc', executable='goal_marker_node',
            name='goal_marker_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'lab2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # FindPackageShare, not get_package_share_directory: resolved only when the action
    # runs, so a machine without the simulator can still launch the nodes.
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch',
                LaunchConfiguration('world_launch'),
            ])
        ),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription(arguments + [domain] + nodes + [rviz, world])
