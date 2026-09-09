import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('agrios_description')
    bringup = get_package_share_directory('agrios_bringup')
    rviz_cfg = os.path.join(desc, 'rviz', 'agrios.rviz')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc, 'launch', 'sim.launch.py')),
        launch_arguments={'lock_steering': 'false'}.items())

    loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'localization.launch.py')))

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'nav2.launch.py')))

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        sim,
        TimerAction(period=15.0, actions=[loc]),
        TimerAction(period=20.0, actions=[nav]),
        TimerAction(period=30.0, actions=[rviz]),
    ])