from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the installed project_utils config directory
    util_dir = get_package_share_directory('project_utils')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(util_dir, 'config', 'vehicle_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_map_dir_cmd = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.join(util_dir,"maps"),
        description='Full path to the ROS2 map dir')
          
    # prefix = ["gdbserver localhost:3000"],
    # emulate_tty=True
    load_nodes = GroupAction(
        actions = [
            Node(
                package='visualizer',
                executable='visualizer_node',
                name='visualizer_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file'),
                            { 'map_dir': LaunchConfiguration('map_dir') }
                            ],
                )
            ]  
    )
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_dir_cmd)
    ld.add_action(load_nodes)
    
    return ld