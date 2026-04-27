import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_robot_description():
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )
    result = subprocess.run(
        ['xacro', urdf_file, 'namespace:='],
        capture_output=True, text=True
    )
    return result.stdout

def generate_launch_description():
    pkg_sim = get_package_share_directory('coopexplorer_sim')
    scene_path = os.path.join(pkg_sim, 'scenes', 'coopexplorer.ttt')

    robot_description = get_robot_description()

    slam_params = os.path.join(
        get_package_share_directory('coopexplorer_bringup'),
        'config', 'slam_toolbox_tb3_0.yaml'
    )

    nav2_params_tb3_0 = os.path.join(
        get_package_share_directory('coopexplorer_bringup'),
        'config', 'nav2_params_tb3_0.yaml'
    )

    nav2_params_tb3_1 = os.path.join(
        get_package_share_directory('coopexplorer_bringup'),
        'config', 'nav2_params_tb3_1.yaml'
    )

    map_file = os.path.join(
        get_package_share_directory('coopexplorer_bringup'),
        'maps', 'map_tb3_0.yaml'
    )

    rsp_tb3_0 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_0',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': 'tb3_0/'
        }],
        output='screen'
    )

    rsp_tb3_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_1',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': 'tb3_1/'
        }],
        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true'
        }.items()
    )

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        parameters=[{
            'use_sim_time': True,
            'base_frame_id': 'tb3_1/base_footprint',
            'odom_frame_id': 'tb3_1/odom',
            'global_frame_id': 'map',
            'scan_topic': '/tb3_1/scan',
            'set_initial_pose': True,
            'initial_pose.x': 0.035,
            'initial_pose.y': 0.75,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0
        }],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        output='screen'
    )

    nav2_tb3_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_tb3_0
        }.items()
    )

    frontier_detector = Node(
        package='coopexplorer_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        parameters=[{
            'robot_namespace': 'tb3_0',
            'min_frontier_size': 5,
            'use_sim_time': False
        }],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),

        ExecuteProcess(
            cmd=['coppeliaSim.sh', scene_path],
            output='screen'
        ),

        rsp_tb3_0,
        rsp_tb3_1,

        TimerAction(period=5.0, actions=[slam_toolbox]),

        TimerAction(period=6.0, actions=[
            map_server,
            amcl,
            lifecycle_manager,
        ]),

        TimerAction(period=22.0, actions=[nav2_tb3_0]),

        TimerAction(period=35.0, actions=[frontier_detector]),

        TimerAction(period=20.0, actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', os.path.join(
                    get_package_share_directory('coopexplorer_bringup'),
                    'rviz', 'coopexplorer.rviz'
                )],
                output='screen'
            )
        ]),
    ])
