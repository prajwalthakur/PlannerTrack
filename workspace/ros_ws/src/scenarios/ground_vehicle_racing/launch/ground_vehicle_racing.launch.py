# Author Prajwal Thakur
import os
import sys

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ros2 launch loads this file by path, not as an installed Python package, so
# sibling modules under controllers/ need this on sys.path to be importable.
sys.path.append(os.path.dirname(__file__))
from controllers.pure_pursuit import build_nodes as build_pure_pursuit_nodes 
from controllers.mppi import build_nodes as build_mppi_nodes  


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

    # Read agents.yaml at launch-generation time (same file agent_sim itself
    # consumes at runtime) so the controller count/numbering can never drift
    # from the actual agent count -- no hardcoded N here. Each agent's own
    # "controller:" field picks which builder below runs for that agent.
    with open(agents_config) as f:
        agents_yaml = yaml.safe_load(f)['agents']
    agent_names = list(agents_yaml.keys())

    default_map = os.path.join(scenario_dir, 'map', 'map.yaml')
    default_waypoints = os.path.join(scenario_dir, 'map', 'global_waypoints.json')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file to load with map_server'
    )

    # Node/param wiring for each controller type lives in its own module
    # under controllers/ (see build_nodes() there). To move an agent between
    # controllers, edit its "controller:" field in agents.yaml -- nothing
    # here needs to change. To add a new controller type, add one
    # controllers/<name>.py with a build_nodes(i, sim_config, agents_config)
    # and one entry below.
    CONTROLLER_BUILDERS = {
        'pure_pursuit': build_pure_pursuit_nodes,
        'mppi_controller': build_mppi_nodes,
    }

    controller_nodes = []
    for i, agent_name in enumerate(agent_names, start=1):
        controller_type = agents_yaml[agent_name].get('controller', 'pure_pursuit')
        builder = CONTROLLER_BUILDERS.get(controller_type)
        if builder is None:
            raise ValueError(
                f"{agent_name}: unknown controller '{controller_type}', "
                f"expected one of {list(CONTROLLER_BUILDERS)}"
            )
        controller_nodes.extend(builder(i, sim_config, agents_config))

    return LaunchDescription([
        map_arg,
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
        # map_server is a lifecycle node -- it comes up unconfigured/inactive
        # and won't publish /map on its own. lifecycle_manager below drives
        # it through configure -> activate on startup.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }]
        ),
        # agent_sim publishes everything relative to "odom" (see
        # agent_interface.fixed_frame in params.yaml); map_server publishes
        # the OccupancyGrid in "map". No localization node in this sim
        # corrects one against the other, so bridge them as coincident
        # frames with an identity transform.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        Node(
            package='scenarios',
            executable='global_waypoints_publisher.py',
            name='global_waypoints_publisher',
            output='screen',
            parameters=[{
                'waypoints_file': default_waypoints,
                'frame_id': 'map',
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        *controller_nodes,
    ])
