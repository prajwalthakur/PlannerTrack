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
        'configuration_file',
        default_value=os.path.join(util_dir, 'config', 'sim_config.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    map_dir = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(util_dir,"/maps"),
        description='Full path to the ROS2 map dir')
          
    # prefix = ["gdbserver localhost:3000"],
    # emulate_tty=True
    load_nodes = GroupAction(
        actions = [
            Node(
                package='vehicle_interface',
                executable='vehicle_interface_node',
                name='vehicle_interface_node',
                output='screen',
                parameters=[LaunchConfiguration('configuration_file')],
                )
            ]  
    )
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(load_nodes)
    
    return ld