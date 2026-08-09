from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    warehouse_package = get_package_share_directory(
        'warehouse_robot_description'
    )

    nav2_package = get_package_share_directory(
        'nav2_bringup'
    )

    # ------------------------------------------------------
    # Paths
    # ------------------------------------------------------

    gazebo_launch = os.path.join(
        warehouse_package,
        'launch',
        'gazebo.launch.py'
    )

    nav2_bringup_launch = os.path.join(
        nav2_package,
        'launch',
        'bringup_launch.py'
    )

    nav2_rviz_config = os.path.join(
        nav2_package,
        'rviz',
        'nav2_default_view.rviz'
    )

    nav2_params = os.path.join(
        warehouse_package,
        'config',
        'nav2_params.yaml'
    )

    map_file = os.path.expanduser(
        '~/warehouse_ws/maps/warehouse_map.yaml'
    )


    # ======================================================
    # 1. GAZEBO + ROBOT + EKF
    # ======================================================

    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch
        )
    )


    # ======================================================
    # 2. NAV2
    #
    # slam=false:
    # use saved map + AMCL
    #
    # autostart=true:
    # activate planner/controller/bt/etc automatically
    # ======================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_bringup_launch
        ),

        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True'
        }.items()
    )


    # ======================================================
    # 3. RVIZ
    # ======================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',

        arguments=[
            '-d',
            nav2_rviz_config
        ],

        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # ======================================================
    # DELAYS
    #
    # Give Gazebo enough time to start before Nav2.
    # Give Nav2 a little time before RViz starts.
    # ======================================================

    delayed_nav2 = TimerAction(
        period=6.0,
        actions=[
            nav2
        ]
    )

    delayed_rviz = TimerAction(
        period=9.0,
        actions=[
            rviz
        ]
    )


    return LaunchDescription([

        gazebo_robot,

        delayed_nav2,

        delayed_rviz

    ])
