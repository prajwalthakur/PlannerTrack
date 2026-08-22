# Author Prajwal Thakur
#
# Standalone debug/demo launch for the SSC corridor pipeline -- deliberately
# separate from intersection.launch.py (computeBezierTrajectory() is still a
# stub, so this planner isn't production-ready yet). Ego (agent_1) stays
# static: no controller nodes are launched for any agent, since
# SscPlanner::projectAgentsToFrenet() reads only published reference
# trajectories, never live odom-stepped poses (see ssc_planner.cpp -- the
# geomModel->step(...) calls in both the ego and other-agent odom callbacks
# are commented out). That makes this the minimal real exercise of the
# corridor code path.
#
# Also brings up the same visual-context nodes intersection.launch.py uses
# (lane markers, route graph markers, map<->odom static transform) so the
# corridor (published on both an abstract debug frame and, via
# buildCorridorMarkersCartesian, overlaid in "map") can be seen alongside
# the real scene -- not just verified in isolation.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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

    return LaunchDescription([
        map_arg,
        # Spawns odom/TF for every agent in agents.yaml unconditionally --
        # needed here only for agent_1's own odom (InputData::mEgoPose).
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
        # map_server/route_server are lifecycle nodes; route_server is a
        # hard dependency of global_waypoints_publisher's ComputeRoute
        # action call below.
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
                # See intersection.launch.py's own comment on this same
                # param -- route_server never creates a bond heartbeat, so
                # bond monitoring must be disabled or bringup times out.
                'bond_timeout': 0.0,
            }]
        ),
        # agent_sim publishes everything relative to "odom"; map_server
        # publishes in "map". No localization node corrects one against the
        # other, so bridge them as coincident frames -- needed here since
        # the Cartesian corridor overlay is published in "map".
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
        # Calls route_server's ComputeRoute action once per agent, publishes
        # both /agent_<i>/reference_path and /agent_<i>/reference_trajectory,
        # latched, for every agent -- including agent_1/ego, which feeds
        # PlannerNode's InputData::mEgoPath/mEgoInitTraj below, and every
        # other agent, which feeds SscPlanner's own internal occupancy
        # subscriptions (receivePredictedTrajectoryStatic()).
        Node(
            package='scenarios',
            executable='intersection_global_waypoints_publisher.py',
            name='global_waypoints_publisher',
            output='screen',
            parameters=[{
                'agents_config_file': agents_config,
                'frame_id': 'map',
                'cruise_speed_mps': 1.0,
                'decel_distance_m': 0.5,
            }]
        ),
        # The SSC corridor generator itself, ego (agent_1) only -- SscMap's
        # corridor is a single-vehicle, single-route structure (one seed
        # trajectory -> one corridor, no per-behavior loop), so no other
        # agent needs its own planner instance.
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
    ])
