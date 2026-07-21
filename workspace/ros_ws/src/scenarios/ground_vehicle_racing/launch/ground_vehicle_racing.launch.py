# Author Prajwal Thakur
import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    controller_config = os.path.join(params_dir, 'controller.yaml')
    rviz_config = os.path.join(params_dir, 'multi_rviz.rviz')

    # Read agents.yaml at launch-generation time (same file agent_sim itself
    # consumes at runtime) so the controller count/numbering can never drift
    # from the actual agent count -- no hardcoded N here.
    with open(agents_config) as f:
        agent_names = list(yaml.safe_load(f)['agents'].keys())

    default_map = os.path.join(scenario_dir, 'map', 'map.yaml')
    default_waypoints = os.path.join(scenario_dir, 'map', 'global_waypoints.json')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file to load with map_server'
    )

    # One controller_node_exe per agent (which algorithm it actually runs --
    # regulated_pp, dummy, mpcc -- is picked inside controller.yaml's
    # hybrid_mode/lateral_controller_mode/longitudinal_controller_mode, not
    # here). Namespaced per agent (name kept as "controller" in every
    # namespace, not "controller_agent_N") so controller.yaml's single
    # "controller:" ros__parameters key applies unmodified to every
    # instance, and so each instance's ~/... private topics land under
    # /agent_N/controller/... without needing explicit remaps.
    #
    # Remap split:
    #   - global (same absolute topic for every instance): the shared
    #     raceline from global_waypoints_publisher.
    #   - per-agent (absolute, index-specific): this instance's own
    #     odometry in, control command out.
    # ~/output/control_cmd is still project_utils_msgs/Control; agent_sim's
    # control subscriber expects EigenVector -- the remap below won't
    # actually deliver data until that conversion lands on the controller
    # side (separate, already-planned follow-up), but the topic wiring is
    # forward-looking so nothing else needs to change once it does.
    controller_nodes = [
        Node(
            package='trajectory_follower_node',
            executable='controller_node_exe',
            name='controller',
            namespace=f'agent_{i}',
            output='screen',
            parameters=[
                controller_config,
                {
                    'sim_config_file': sim_config,
                    'agents_config_file': agents_config,
                    'agent_number': i,
                },
            ],
            remappings=[
                ('~/input/reference_trajectory', '/global_planner/iqp_trajectory'),
                ('~/input/current_odometry', f'/agent_{i}/odom'),
                ('~/input/current_steering',f'/agent_{i}/steering_report'),
                ('~/output/control_cmd', f'/agent_{i}/amr_control'),
            ],
        )
        for i, _ in enumerate(agent_names, start=1)
    ]

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
