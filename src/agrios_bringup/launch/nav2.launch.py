import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agrios_bringup')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(pkg, 'behavior_trees', 'navigate_no_spin.xml')

    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[params, {'use_sim_time': True}]),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=[params, {'use_sim_time': True}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             output='screen', parameters=[params, {'use_sim_time': True}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen',
             parameters=[params,
                         {'use_sim_time': True},
                         {'default_nav_to_pose_bt_xml': bt_xml},
                         {'default_nav_through_poses_bt_xml': bt_xml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True},
                         {'autostart': True},
                         {'node_names': ['controller_server', 'planner_server',
                                         'behavior_server', 'bt_navigator']}]),
    ])