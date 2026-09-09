import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agrios_description')
    xacro_file = os.path.join(pkg, 'urdf', 'agrios.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'agrios_empty.sdf')

    lock = LaunchConfiguration('lock_steering')
    robot_desc = Command(['xacro ', xacro_file, ' lock_steering:=', lock])

    sim_env = os.environ.copy()
    sim_env['__EGL_VENDOR_LIBRARY_FILENAMES'] = \
        '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '4', world_file],
        env=sim_env,
        output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'agrios',
                   '-z', '0.5'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('lock_steering', default_value='true'),
        gazebo,
        rsp,
        bridge,
        TimerAction(period=5.0, actions=[spawn]),
        # TimerAction(period=15.0, actions=[rviz]),
    ])