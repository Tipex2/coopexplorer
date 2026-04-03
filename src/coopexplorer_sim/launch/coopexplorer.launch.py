import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('coopexplorer_sim')
    scene_path = os.path.join(pkg_dir, 'scenes', 'coopexplorer.ttt')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['coppeliaSim.sh', scene_path],
            output='screen'
        ),
    ])
