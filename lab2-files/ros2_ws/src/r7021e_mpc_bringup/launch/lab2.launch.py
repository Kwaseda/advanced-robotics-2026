"""Lab 2 launch file for all four tasks.

    ros2 launch r7021e_mpc_bringup lab2.launch.py task:=2 sim:=true rviz:=true   Gazebo
    ros2 launch r7021e_mpc_bringup lab2.launch.py task:=2 rviz:=true             real robot

Arguments: task (1-4), sim, rviz, t_step, n_horizon, domain_id (3<robot number>).
Simulation time is used exactly when sim:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('r7021e_mpc_bringup')
    config = os.path.join(share, 'config')

    task = LaunchConfiguration('task')
    sim = LaunchConfiguration('sim')
    use_sim_time = ParameterValue(sim, value_type=bool)
    horizon = {
        't_step': ParameterValue(LaunchConfiguration('t_step'), value_type=float),
        'n_horizon': ParameterValue(LaunchConfiguration('n_horizon'), value_type=int),
    }

    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='1', choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('t_step', default_value='0.1'),
        DeclareLaunchArgument('n_horizon', default_value='20'),
        DeclareLaunchArgument('domain_id', default_value='34'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),

        # Later files win: shared defaults, then the task's changes, then the launch arguments.
        Node(
            package='r7021e_mpc', executable='mpc_node', name='mpc_node', output='screen',
            parameters=[
                os.path.join(config, 'mpc.yaml'),
                PathJoinSubstitution([config, ['mpc_task', task, '.yaml']]),
                {'use_sim_time': use_sim_time, **horizon},
            ]),
        Node(
            package='r7021e_mpc', executable='trajectory_node', name='trajectory_node',
            output='screen',
            parameters=[os.path.join(config, 'circle.yaml'),
                        {'use_sim_time': use_sim_time, **horizon}],
            condition=IfCondition(PythonExpression(['"', task, '" == "4"']))),
        Node(
            package='r7021e_mpc', executable='goal_marker_node', name='goal_marker_node',
            output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'lab2.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch',
                'turtlebot3_dqn_stage1.launch.py'])),
            condition=IfCondition(sim)),
    ])
