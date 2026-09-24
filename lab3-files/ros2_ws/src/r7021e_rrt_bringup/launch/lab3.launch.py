"""One launch file for Lab 3, autonomous exploration with sampling-based planning.

    ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true rviz:=true
    ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true world:=lab3_maze
    ros2 launch r7021e_rrt_bringup lab3.launch.py use_sim_time:=false domain_id:=34

The first line is the desk case: maze, Burger, SLAM, frontier detector, follower,
planner, RViz. The last is the lab bench: the robot is already running
`turtlebot3_bringup robot.launch.py` over ssh, so no simulator and a real clock.

Arguments, all of them things that change between a desk and a bench:

    sim           bring up Gazebo and spawn the Burger. Off by default, because
                  the graded run is on hardware.
    world         which generated maze. lab3_maze_small (4.0 m, iteration) or
                  lab3_maze (7.2 m, the recorded run). Ignored unless sim.
    gui           Gazebo's own GUI. Set false for a headless parameter sweep.
    rviz          start RViz2 with the saved Lab 3 configuration.
    slam          start slam_toolbox. false when something else already maps.
    domain_id     ROS_DOMAIN_ID for every node started here. The lab dictates
                  3<robot number>, so turtle4 is 34. A flag, never an edit.
    use_sim_time  true in Gazebo, false on the robot. Set once, here, for every
                  node, including slam_toolbox.
    config_dir    where the parameter files are. Defaults to this package's share.
    gain_mode     reduced_range or cluster_size. The I(p) comparison the report
                  carries is two runs differing only in this flag.
    inflation     obstacle inflation radius in metres. An argument only because
                  the report carries a sweep over it; the shipped value lives in
                  lab3.yaml with its justification, and this overrides it.

Why gain_mode and inflation are arguments when every other number is not
------------------------------------------------------------------------
Because runs are compared across them. A comparison made by editing a YAML file
between two runs cannot be shown to have differed in only one place. Each appears
once below as an override appended after the file, so the file stays the single
statement of what the shipped configuration is.

What this file deliberately does not start
------------------------------------------
Lab 1's controller_node and Lab 2's mpc_node. Both publish /cmd_vel, and so does
the path follower started here. Two publishers on one /cmd_vel do not error, they
interleave, and the robot does something that looks like a tuning problem.

The frontier detector and the path follower come from the course's own
r7021e_exploration package, unmodified, which is what the lab intends: "Your
focus in this lab is only on the path planner, and on the exploration method."
That package must be installed in the workspace alongside this one; the lab
instructions say to extract it into ~/ros2_ws/src, and this file assumes it has
been.

The frontier topic is remapped here. frontier_detector_node publishes on
`frontiers`, plural, and navigation_node subscribes to the same name, so the
remapping below is an identity that exists to be read: the course's own
navigation template declares `frontier`, singular, and its launch file carries no
remapping, so as shipped its subscription never receives anything. Leaving a
visible remapping here means the next person to wire this up sees the topic name
in the launch file rather than discovering it from silence.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'r7021e_rrt_bringup'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    arguments = [
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='bring up the Gazebo maze and spawn the Burger'),
        DeclareLaunchArgument(
            'world', default_value='lab3_maze_small',
            description='lab3_maze_small (4.0 m) or lab3_maze (7.2 m). sim only'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description="Gazebo's own GUI. false for a headless sweep"),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='start RViz2 with the saved Lab 3 configuration'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='start slam_toolbox in async mapping mode'),
        DeclareLaunchArgument(
            'domain_id', default_value='34',
            description='ROS_DOMAIN_ID, the lab uses 3<robot number>. turtle4 is 34'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='true in Gazebo, false on the robot'),
        DeclareLaunchArgument(
            'config_dir', default_value=os.path.join(share, 'config'),
            description='directory holding robot.yaml, lab3.yaml and slam_lab3.yaml'),
        DeclareLaunchArgument(
            'gain_mode', default_value='reduced_range',
            choices=['reduced_range', 'cluster_size'],
            description='I(p) form. The report compares one run of each'),
        DeclareLaunchArgument(
            'inflation', default_value='0.105',
            description='obstacle inflation radius, metres. Swept in the report'),
        # The follower's own max_w, set from here rather than by editing its code.
        # Lowering it has been tried and measured and made things worse; see the
        # design notes before changing it.
        DeclareLaunchArgument(
            'follower_max_w', default_value='1.0',
            description="path follower's angular speed limit, rad/s"),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('config_dir')

    domain = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

    # Two files per node, in this order, because later files win on a repeated key:
    #
    #   robot.yaml    the Burger's physical facts, one definition for everything
    #   lab3.yaml     this lab's own numbers, each with its reason
    #
    # then the two sweep overrides as dictionaries, which win over both. An
    # override that is not exercised is identical to the file's own value, so a
    # default launch and a launch with the defaults typed out behave the same.
    navigation_parameters = [
        PathJoinSubstitution([config_dir, 'robot.yaml']),
        PathJoinSubstitution([config_dir, 'lab3.yaml']),
        {
            'use_sim_time': use_sim_time,
            'gain.mode': LaunchConfiguration('gain_mode'),
            'inflation_radius': LaunchConfiguration('inflation'),
        },
    ]

    ours = Node(
        package='r7021e_rrt', executable='navigation_node',
        name='navigation_node', output='screen',
        parameters=navigation_parameters,
    )

    # Course-provided, unmodified. See the module docstring on the remapping.
    frontier_detector = Node(
        package='r7021e_exploration', executable='frontier_detector_node',
        name='frontier_detector', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('frontiers', 'frontiers')],
    )

    path_follower = Node(
        package='r7021e_exploration', executable='path_follower_node',
        name='path_follower', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_w': ParameterValue(LaunchConfiguration('follower_max_w'),
                                    value_type=float),
        }],
    )

    # slam_toolbox's own launch file rather than a hand-rolled LifecycleNode plus
    # configure and activate events. It does the same lifecycle dance, it is
    # maintained by the package, and it takes the parameter file as an argument,
    # which is what lets use_sim_time reach it from here instead of being pinned
    # to false inside the launch file the way the course's copy pins it.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_dir, 'slam_lab3.yaml']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'maze_world.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'lab3.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        arguments + [domain, world, slam, frontier_detector, path_follower, ours, rviz])
