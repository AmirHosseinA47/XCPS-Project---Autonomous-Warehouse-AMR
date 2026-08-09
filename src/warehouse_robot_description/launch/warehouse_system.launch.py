import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler
)

from launch.conditions import (
    IfCondition,
    UnlessCondition
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    warehouse_pkg = get_package_share_directory(
        'warehouse_robot_description'
    )

    nav2_bringup_pkg = get_package_share_directory(
        'nav2_bringup'
    )

    warehouse_launch_dir = os.path.join(
        warehouse_pkg,
        'launch'
    )

    nav2_launch_dir = os.path.join(
        nav2_bringup_pkg,
        'launch'
    )

    # =========================================================
    # FILE PATHS
    # =========================================================

    default_map = os.path.join(
        os.path.expanduser('~'),
        'warehouse_ws',
        'maps',
        'warehouse_map.yaml'
    )

    default_params = os.path.join(
        warehouse_pkg,
        'config',
        'nav2_params.yaml'
    )

    default_rviz_config = os.path.join(
        nav2_bringup_pkg,
        'rviz',
        'nav2_default_view.rviz'
    )

    auto_initial_pose_script = os.path.join(
        warehouse_launch_dir,
        'auto_initial_pose.py'
    )

    # The dynamic worker SDF is currently stored in the
    # source workspace.
    #
    # We deliberately use the exact file that has already
    # been tested successfully in Gazebo.
    warehouse_worker_sdf = os.path.join(
        os.path.expanduser('~'),
        'warehouse_ws',
        'src',
        'warehouse_robot_description',
        'models',
        'moving_agv.sdf'
    )

    # =========================================================
    # LAUNCH CONFIGURATIONS
    # =========================================================

    headless = LaunchConfiguration(
        'headless'
    )

    use_rviz = LaunchConfiguration(
        'rviz'
    )

    use_dashboard = LaunchConfiguration(
        'dashboard'
    )

    dynamic_obstacle = LaunchConfiguration(
        'dynamic_obstacle'
    )

    map_file = LaunchConfiguration(
        'map'
    )

    params_file = LaunchConfiguration(
        'params_file'
    )

    rviz_config = LaunchConfiguration(
        'rviz_config'
    )

    # =========================================================
    # LAUNCH ARGUMENTS
    # =========================================================

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Run Gazebo without GUI'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Start RViz2'
    )

    declare_dashboard = DeclareLaunchArgument(
        'dashboard',
        default_value='True',
        description='Start mission dashboard'
    )

    declare_dynamic_obstacle = DeclareLaunchArgument(
        'dynamic_obstacle',
        default_value='False',
        description=(
            'Spawn the moving warehouse worker '
            'dynamic obstacle'
        )
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Warehouse map YAML'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Nav2 parameters file'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file'
    )

    # =========================================================
    # GAZEBO HEADLESS
    # =========================================================

    gazebo_headless = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                warehouse_launch_dir,
                'gazebo_headless.launch.py'
            )
        ),

        condition=IfCondition(
            headless
        )
    )

    # =========================================================
    # GAZEBO GUI
    # =========================================================

    gazebo_gui = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                warehouse_launch_dir,
                'gazebo.launch.py'
            )
        ),

        condition=UnlessCondition(
            headless
        )
    )

    # =========================================================
    # NAV2 LOCALIZATION
    #
    # Starts:
    #
    # map_server
    # amcl
    # lifecycle_manager_localization
    #
    # Navigation is intentionally NOT started yet.
    # =========================================================

    localization = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'localization_launch.py'
            )
        ),

        launch_arguments={
            'namespace': '',
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items()
    )

    # =========================================================
    # AUTOMATIC INITIAL POSE
    #
    # Sequence:
    #
    # Gazebo clock available
    #        |
    #        v
    # AMCL ACTIVE
    #        |
    #        v
    # /set_initial_pose
    #        |
    #        v
    # /amcl_pose received
    #        |
    #        v
    # process exits
    # =========================================================

    auto_initial_pose = ExecuteProcess(

        cmd=[
            'python3',
            auto_initial_pose_script,
            '--ros-args',
            '-p',
            'use_sim_time:=true'
        ],

        output='screen'
    )

    # =========================================================
    # NAV2 NAVIGATION
    #
    # Uses our stable configuration:
    #
    # SmacPlanner2D
    # Regulated Pure Pursuit
    # Local / Global Costmaps
    # Collision Monitor
    # Behavior Server
    # BT Navigator
    # =========================================================

    navigation = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'navigation_launch.py'
            )
        ),

        launch_arguments={
            'namespace': '',
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items()
    )

    # =========================================================
    # DYNAMIC WAREHOUSE WORKER
    #
    # This is the actor already tested successfully.
    #
    # Initial Gazebo world position:
    #
    # x = -3.5
    # y =  1.5
    # z =  0.0
    #
    # Its internal scripted trajectory moves it along
    # the warehouse aisle continuously.
    #
    # Spawn only when:
    #
    # dynamic_obstacle:=True
    # =========================================================

    warehouse_worker = Node(

        package='ros_gz_sim',
        executable='create',

        name='spawn_warehouse_worker',

        output='screen',

        arguments=[
            '-file',
            warehouse_worker_sdf,

            '-name',
            'warehouse_worker',

            '-x',
            '-3.5',

            '-y',
            '1.5',

            '-z',
            '0.0'
        ],

        condition=IfCondition(
            dynamic_obstacle
        )
    )

    # =========================================================
    # START NAVIGATION AFTER LOCALIZATION
    #
    # When auto_initial_pose exits:
    #
    # 1. Start Nav2 navigation
    #
    # 2. If dynamic_obstacle=True,
    #    spawn the moving warehouse worker
    #
    # No arbitrary sleep or TimerAction.
    # =========================================================

    start_after_localization = RegisterEventHandler(

        OnProcessExit(

            target_action=auto_initial_pose,

            on_exit=[
                navigation,
                warehouse_worker
            ]
        )
    )

    # =========================================================
    # RVIZ
    # =========================================================

    rviz = Node(

        package='rviz2',
        executable='rviz2',
        name='rviz2',

        output='screen',

        arguments=[
            '-d',
            rviz_config
        ],

        parameters=[
            {
                'use_sim_time': True
            }
        ],

        condition=IfCondition(
            use_rviz
        )
    )

    # =========================================================
    # MISSION DASHBOARD
    # =========================================================

    dashboard = Node(

        package='warehouse_robot_missions',
        executable='mission_dashboard',
        name='mission_dashboard',

        output='screen',

        condition=IfCondition(
            use_dashboard
        )
    )

    # =========================================================
    # COMPLETE SYSTEM
    # =========================================================

    return LaunchDescription([

        # -----------------------------------------------------
        # Arguments
        # -----------------------------------------------------

        declare_headless,
        declare_rviz,
        declare_dashboard,
        declare_dynamic_obstacle,

        declare_map,
        declare_params,
        declare_rviz_config,

        # -----------------------------------------------------
        # Simulation
        # -----------------------------------------------------

        gazebo_headless,
        gazebo_gui,

        # -----------------------------------------------------
        # Localization
        # -----------------------------------------------------

        localization,

        # -----------------------------------------------------
        # Register startup sequence
        # -----------------------------------------------------

        start_after_localization,

        # -----------------------------------------------------
        # Automatic initial localization
        # -----------------------------------------------------

        auto_initial_pose,

        # -----------------------------------------------------
        # Operator interfaces
        # -----------------------------------------------------

        rviz,
        dashboard,
    ])


