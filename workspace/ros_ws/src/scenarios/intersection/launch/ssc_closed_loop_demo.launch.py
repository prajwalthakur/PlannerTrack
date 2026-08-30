# Author Prajwal Thakur
#
import os
import sys

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(os.path.dirname(__file__)))
from controllers.pure_pursuit import build_nodes as build_pure_pursuit_nodes
from controllers.mppi import build_nodes as build_mppi_nodes

EGO_AGENT_NUMBER = 1
EGO_TRAJECTORY_TOPIC = f'/agent_{EGO_AGENT_NUMBER}/planner/ssc/bezier_trajectory'


def generate_launch_description():
    scenario_dir = os.path.join(
        get_package_share_directory('scenarios'),
        'intersection'
    )

    params_dir = os.path.join(scenario_dir, 'params')
    params_config = os.path.join(params_dir, 'params.yaml')
    agents_config = os.path.join(params_dir, 'agents.yaml')
    sim_config = os.path.join(params_dir, 'sim.yaml')
    ssc_config = os.path.join(params_dir, 'ssc_planner.yaml')
    rviz_config = os.path.join(params_dir, 'ssc_corridor_debug.rviz')

    default_map = os.path.join(scenario_dir, 'map', 'map.yaml')
    route_graph_file = os.path.join(scenario_dir, 'map', 'route_graph.geojson')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file to load with map_server'
    )

    with open(agents_config) as f:
        agents_yaml = yaml.safe_load(f)['agents']
    agent_names = list(agents_yaml.keys())

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
        if i == EGO_AGENT_NUMBER:
            controller_nodes.extend(
                builder(i, sim_config, agents_config,
                        reference_trajectory_topic=EGO_TRAJECTORY_TOPIC,
                        controller_param_overrides={
                            'regulated_pp.optimizer.set_external_target_speed': False,
                        }))
        else:
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
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}]
        ),
        Node(
            package='mpl_route_planner',
            executable='route_server_node',
            name='route_server',
            output='screen',
            parameters=[{
                'graph_filepath': route_graph_file,
                'route_frame': 'map',
                'path_density': 0.05,
                'smooth_corners': True,
                'smoothing_radius': 0.8,
                'smoothing_angle_threshold': 2.9,
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'route_server'],
                'bond_timeout': 0.0,
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        Node(
            package='scenarios',
            executable='publish_lane_markers.py',
            name='lane_marker_publisher',
            output='screen',
            parameters=[{'params_file': params_config, 'frame_id': 'map'}]
        ),
        Node(
            package='scenarios',
            executable='publish_route_graph_markers.py',
            name='route_graph_marker_publisher',
            output='screen',
            parameters=[{
                'graph_file': route_graph_file,
                'frame_id': 'map',
            }]
        ),
        # Every agent's reference_trajectory publishes immediately, ego
        # included. SscPlanner subscribes to every OTHER agent's
        # reference_trajectory as its own occupancy/corridor-construction
        # input, not just as something a controller tracks -- an earlier
        # version of this launch gated non-ego agents' publish behind ego's
        # own SSC output (to keep other agents' scripted routes from
        # finishing before ego's ~15s-to-solve trajectory was even ready),
        # but that starved SSC of other agents' data at the exact moment it
        # needed it ("[SscMap]: Trajectory is empty", "wrong status for
        # vehicle id N") -- confirmed live, a "solved" Bezier trajectory
        # drove straight through another agent it never saw. Removed.
        Node(
            package='scenarios',
            executable='intersection_global_waypoints_publisher.py',
            name='global_waypoints_publisher',
            output='screen',
            parameters=[{
                'agents_config_file': agents_config,
                'frame_id': 'map',
                'cruise_speed_mps': 0.4,
                'decel_distance_m': 0.6,
            }]
        ),
        Node(
            package='planner_base',
            executable='planner_node_exe',
            name='planner',
            namespace='agent_1',
            output='screen',
            parameters=[{
                'agent_number': 1,
                'agents_config_file': agents_config,
                'graph_file': route_graph_file,
                'ssc_config_file': ssc_config,
                'planner_plugin': 'SscPlanner',
                'planner_frequency': 2.0,
            }],
            remappings=[
                ('~/input/reference_path', '/agent_1/reference_path'),
                ('~/input/reference_trajectory', '/agent_1/reference_trajectory'),
                ('~/input/odom', '/agent_1/odom'),
            ],
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
