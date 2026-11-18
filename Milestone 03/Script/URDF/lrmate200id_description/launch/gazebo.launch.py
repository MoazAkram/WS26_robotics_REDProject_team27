import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lrmate200id_description')
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'lrmate200id_gazebo.urdf.xacro')).toxml()

    resource_path = os.path.dirname(pkg)
    old = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if old:
        resource_path = resource_path + os.pathsep + old

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'lrmate200id'],
                 output='screen')

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/model/lrmate200id/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                  remappings=[('/model/lrmate200id/pose', '/end_effector_pose')],
                  output='screen')

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    pos = Node(package='controller_manager', executable='spawner',
               arguments=['position_controller'], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[pos])),
    ])
