import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def spawn_robot(context, *args, **kwargs):
    pkg = get_package_share_directory('agrios_description')
    xacro_file = os.path.join(pkg, 'urdf', 'agrios.urdf.xacro')
    lock = LaunchConfiguration('lock_steering').perform(context)

    urdf = subprocess.check_output(
        ['xacro', xacro_file, f'lock_steering:={lock}']).decode()
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False)
    tmp.write(urdf)
    tmp.close()

    spawn = ExecuteProcess(
        cmd=['ign', 'service', '-s', '/world/empty/create',
             '--reqtype', 'ignition.msgs.EntityFactory',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '5000',
             '--req', f'sdf_filename: "{tmp.name}", name: "agrios", '
                      f'pose: {{position: {{z: 0.5}}}}'],
        output='screen')
    return [spawn]


def generate_launch_description():
    pkg = get_package_share_directory('agrios_description')
    xacro_file = os.path.join(pkg, 'urdf', 'agrios.urdf.xacro')
    lock = LaunchConfiguration('lock_steering')
    robot_desc = Command(['xacro ', xacro_file, ' lock_steering:=', lock])
    sim_env = os.environ.copy()
    sim_env['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'

    world_file = os.path.join(pkg, 'worlds', 'agrios_empty.sdf')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '-v', '4', world_file],
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
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen') 

    # lidar_tf = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     arguments=['0','0','0','0','0','0',
    #                'lidar_link', 'agrios::base_footprint::lidar'],
    #     parameters=[{'use_sim_time': True}])


    rviz = Node(
        package='rviz2', executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('lock_steering', default_value='true'),
        gazebo,
        rsp,
        bridge,
        # lidar_tf,
        # SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
        #                 '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        TimerAction(period=15.0, actions=[OpaqueFunction(function=spawn_robot)]),
        # TimerAction(period=60.0, actions=[rviz]),
    ]) 