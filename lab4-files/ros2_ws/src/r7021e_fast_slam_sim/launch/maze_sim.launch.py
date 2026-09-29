"""Lab 4 simulator: the course maze, a Burger in it, and the ros_gz bridge.

    ros2 launch r7021e_fast_slam_sim maze_sim.launch.py ground_truth:=true

ground_truth:=true bridges Gazebo's model pose to /ground_truth
(transforms[0] is the robot). The SLAM node never reads it; it is recorded so
the clean /odom used as ground truth can be checked against the real pose.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'r7021e_fast_slam_sim'
## The <world name=...> inside maze.world; Gazebo namespaces its topics by it.
WORLD_NAME = 'maze_world'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    args = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='start the Gazebo GUI'),
        DeclareLaunchArgument('ground_truth', default_value='false',
                              choices=['true', 'false'],
                              description='bridge the simulator pose to /ground_truth'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
    ]

    world = PathJoinSubstitution([share, 'worlds', 'maze.world'])

    ## Without this the Burger spawns as an invisible collision hull.
    resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([turtlebot3_gazebo, 'models']))

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r -s -v2 ', world],
                          'on_exit_shutdown': 'true'}.items())

    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
        condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([turtlebot3_gazebo, 'launch',
                                  'robot_state_publisher.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

    ## Also starts the bridge, /clock included; do not bridge /clock twice.
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([turtlebot3_gazebo, 'launch',
                                  'spawn_turtlebot3.launch.py'])),
        launch_arguments={'x_pose': LaunchConfiguration('x_pose'),
                          'y_pose': LaunchConfiguration('y_pose')}.items())

    ground_truth = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ground_truth_bridge', output='log',
        arguments=[f'/world/{WORLD_NAME}/dynamic_pose/info'
                   '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[(f'/world/{WORLD_NAME}/dynamic_pose/info', '/ground_truth')],
        condition=IfCondition(LaunchConfiguration('ground_truth')))

    return LaunchDescription(args + [resources, server, gui, robot_state_publisher,
                                     spawn, ground_truth])
