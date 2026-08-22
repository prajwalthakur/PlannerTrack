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
        'intersection'
    )

    params_dir = os.path.join(scenario_dir, 'params')
    params_config = os.path.join(params_dir, 'params.yaml')
    agents_config = os.path.join(params_dir, 'agents.yaml')
    sim_config = os.path.join(params_dir, 'sim.yaml')
    rviz_config = os.path.join(params_dir, 'multi_rviz.rviz')

    default_map = os.path.join(scenario_dir, 'map', 'map.yaml')
    route_graph_file = os.path.join(scenario_dir, 'map', 'route_graph.geojson')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file to load with map_server'
    )

    # Read agents.yaml at launch-generation time (same file agent_sim itself
    # consumes at runtime) so the controller count/numbering can never drift
    # from the actual agent count -- no hardcoded N here. Each agent's own
    # "controller:" field picks which builder below runs for that agent.
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
        # map_server and route_server are both lifecycle nodes -- they come
        # up unconfigured/inactive; lifecycle_manager below drives both
        # through configure -> activate on startup.
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
                # ~0.67-0.76m is this vehicle's real minimum turning radius
                # (s_max=0.4189 rad, wheelbase 0.30-0.34m -- see
                # pure_pursuit_controller.yaml/agents.yaml); 0.8m leaves a
                # little margin. Left turns fit this; right turns are
                # capped tighter by the intersection's own geometry and
                # will fall back to an unsmoothed corner -- see the design
                # discussion this scenario's route graph was built from.
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
                # route_server is a plain rclcpp_lifecycle::LifecycleNode, not
                # nav2::LifecycleNode -- it never creates the bond heartbeat
                # nav2_lifecycle_manager otherwise waits on after activating
                # each node (map_server has one built in; route_server would
                # need bondcpp wired in deliberately to get one, which isn't
                # worth it just for the auto-restart-on-heartbeat-loss safety
                # net in a project this size). 0.0 disables bond monitoring
                # for all managed nodes -- without this, lifecycle_manager
                # times out waiting on route_server's bond after 4s and
                # aborts the entire bringup (map_server included).
                'bond_timeout': 0.0,
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        Node(
            package='scenarios',
            executable='reset_state.py',
            name='reset_state_watchdog',
            output='screen',
            parameters=[{'params_file': params_config}]
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
        # Calls route_server's ComputeRoute action once per agent (each
        # agent's "route: {start_id, goal_id}" in agents.yaml), turns the
        # dense path into a Trajectory, publishes it latched on
        # /agent_<i>/reference_trajectory -- what the controllers below
        # subscribe to. wait_for_server() inside handles startup ordering
        # against route_server's lifecycle activation.
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
        *controller_nodes,
    ])
