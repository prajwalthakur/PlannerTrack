from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml



def generate_launch_description():

    pkg_dir = get_package_share_directory('lane_change_example')

    config_file_path = os.path.join(pkg_dir, 'config', 'sim_config.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'laneChangeRviz.rviz')


    # Package that contains child launch file
    gym_pkg_dir = get_package_share_directory('gym_ros_cpp')

    child_launch = os.path.join(
        gym_pkg_dir,
        'launch',
        'multi_vehicle_interface.launch.py'
    )
    nodes = []

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(child_launch),
            launch_arguments={
                'configuration_file': config_file_path
            }.items()
        )
    )

    nodes.append(
        Node(
            package='lane_change_example',
            executable='lane_change_example_node',
            name='lane_change_example_node',
            output='screen',
            parameters=[config_file_path]
        )
    )


    nodes.append(
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    )
    return LaunchDescription(nodes)
   