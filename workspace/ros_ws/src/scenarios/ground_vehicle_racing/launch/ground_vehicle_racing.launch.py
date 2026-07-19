# Author Prajwal Thakur
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    scenario_dir = os.path.join(
        get_package_share_directory('scenarios'),
        'ground_vehicle_racing'
    )

    params_dir = os.path.join(scenario_dir, 'params')
    params_config = os.path.join(params_dir, 'params.yaml')
    agents_config = os.path.join(params_dir, 'agents.yaml')
    sim_config = os.path.join(params_dir, 'sim.yaml')
    rviz_config = os.path.join(params_dir, 'multi_rviz.rviz')

    return LaunchDescription([
        Node(
            package='agent_sim',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
            parameters=[
                params_config,
                {
                    'sim_config_file': sim_config,
                    'agents_config_file': agents_config,
                },
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
