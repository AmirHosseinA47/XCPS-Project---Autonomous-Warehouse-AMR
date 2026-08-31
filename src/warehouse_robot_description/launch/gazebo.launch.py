from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_path = get_package_share_directory(
        'warehouse_robot_description'
    )

    ros_gz_sim_path = get_package_share_directory(
        'ros_gz_sim'
    )

    urdf_path = os.path.join(
        package_path,
        'urdf',
        'robot.urdf'
    )

    world_path = os.path.join(
        package_path,
        'worlds',
        'warehouse_world.sdf'
    )

    bridge_config_path = os.path.join(
        package_path,
        'config',
        'gz_bridge.yaml'
    )

    ekf_config_path = os.path.join(
        package_path,
        'config',
        'ekf.yaml'
    )

    gazebo_launch_path = os.path.join(
        ros_gz_sim_path,
        'launch',
        'gz_sim.launch.py'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # Gazebo SERVER ONLY
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_path
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )


    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_path}'
        ]
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {
                'use_sim_time': True
            }
        ]
    )


    lidar_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id',
            'lidar_link',
            '--child-frame-id',
            'warehouse_robot/base_link/lidar_sensor'
        ]
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_warehouse_robot',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'warehouse_robot',
            '-x',
            '-7.0',
            '-y',
            '-5.0',
            '-z',
            '0.075',
            '-Y',
            '0.0'
        ]
    )


    delayed_spawn = TimerAction(
        period=3.0,
        actions=[
            spawn_robot
        ]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        gazebo_bridge,
        ekf_node,
        lidar_sensor_tf,
        delayed_spawn
    ])
