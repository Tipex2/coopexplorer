import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sim = get_package_share_directory('coopexplorer_sim')
    scene_path = os.path.join(pkg_sim, 'scenes', 'coopexplorer.ttt')

    return LaunchDescription([
        # Garantiza que CoppeliaSim y ROS2 se ven en el mismo dominio
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),

        # Abre CoppeliaSim con la escena de los dos robots
        ExecuteProcess(
            cmd=['coppeliaSim.sh', scene_path],
            output='screen'
        ),
    ])
