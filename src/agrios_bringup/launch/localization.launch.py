import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agrios_bringup')
    ekf_local_cfg = os.path.join(pkg, 'config', 'ekf_local.yaml')
    ekf_global_cfg = os.path.join(pkg, 'config', 'ekf_global.yaml')
    navsat_cfg     = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        Node(
            package='agrios_control',
            executable='odom_cov_relay',
            name='odom_cov_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='agrios_control',
            executable='rtk_sim',
            name='rtk_sim',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_cfg, {'use_sim_time': True}],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_cfg, {'use_sim_time': True}],
            remappings=[
                ('imu', '/rtk/heading'),
                ('gps/fix', '/rtk/fix'),
                ('odometry/filtered', '/odometry/global'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_cfg, {'use_sim_time': True}],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
    ])