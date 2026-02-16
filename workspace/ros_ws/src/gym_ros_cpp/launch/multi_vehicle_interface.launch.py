from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context, *args, **kwargs):

    util_dir = get_package_share_directory('project_utils')
    default_path = os.path.join(util_dir, 'config', 'sim_config.yaml')

    # Resolve argument
    config_arg = LaunchConfiguration('configuration_file').perform(context)

    # If empty → use default
    if config_arg == '':
        config_file_path = default_path
    else:
        config_file_path = config_arg

    # Now safe to open
    with open(config_file_path, 'r') as f:
        config = yaml.safe_load(f)

    ros_params = config['/**']['ros__parameters']
    sim_config = ros_params['simulation']
    num_vehicles = sim_config['num_vehicles']

    nodes = []

    # Backend node
    nodes.append(
        Node(
            package='gym_ros_cpp',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
            parameters=[config_file_path]
        )
    )

    xacro_file = os.path.join(util_dir, 'config', 'car.xacro')

    # Spawn robot_state_publishers
    for i in range(1, num_vehicles + 1):

        veh_config = ros_params[f"vehicle{i}_param"]
        body_color = veh_config['body_color']
        wheel_color = veh_config['wheel_color']

        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=f"vehicle_{i}",
                parameters=[{
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ',
                            xacro_file,
                            f' car_name:=vehicle_{i}',
                            f' body_r:={body_color[0]}',
                            f' body_g:={body_color[1]}',
                            f' body_b:={body_color[2]}',
                            f' body_a:={body_color[3]}',
                            f' wheel_r:={wheel_color[0]}',
                            f' wheel_g:={wheel_color[1]}',
                            f' wheel_b:={wheel_color[2]}',
                            f' wheel_a:={wheel_color[3]}',
                        ]),
                        value_type=str
                    )
                }]
            )
        )

    return nodes


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'configuration_file',
            default_value='',
            description='Optional path to simulation YAML file'
        ),

        OpaqueFunction(function=launch_setup)
    ])
