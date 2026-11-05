# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
# Static weights vs the Diff-WMPC policy in RViz: the track map, RViz and replay_compare.py (see its docstring).
#
#   ros2 launch src/rl_framework/experiments/diff_weight_mpc/utils/replay_compare.launch.py speed:=0.25 s0:=1.711
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[4]


def generate_launch_description():
    map_file = os.path.join(get_package_share_directory('scenarios'), 'ground_vehicle_racing_rl', 'map', 'map.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('s0', default_value='1.711', description='Start arc length [m]'),
        DeclareLaunchArgument('speed', default_value='0.25', description='Playback speed (1 = real time)'),
        DeclareLaunchArgument('config', default_value='nominal car', description='A baseline_suite.py configuration'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_file}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(HERE.parent / 'replay_compare.rviz')]),
        ExecuteProcess(
            cmd=['python3', str(HERE / 'replay_compare.py'),
                 '--s0', LaunchConfiguration('s0'), '--speed', LaunchConfiguration('speed'),
                 '--config', LaunchConfiguration('config')],
            cwd=str(WORKSPACE), output='screen'),
    ])
