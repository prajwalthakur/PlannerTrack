# Author Prajwal Thakur
# RL variant of ground_vehicle_racing: same track, agent_1 runs the augmented
# single-track plant SingleTrackAugDynModel (state [..., steer, acc], input
# [jerk, steer_rate]) used by rl_framework/plannertrack_py, and the reference
# defaults to the C++ minimum-curvature raceline (MinCurvPlanner).
#
#   ros2 launch scenarios ground_vehicle_racing_rl.launch.py                    # pure pursuit drives
#   ros2 launch scenarios ground_vehicle_racing_rl.launch.py controller:=none   # external node on /agent_1/control
import copy
import os
import sys
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
        'ground_vehicle_racing_rl'
    )

    params_dir = os.path.join(scenario_dir, 'params')
    params_config = os.path.join(params_dir, 'params.yaml')
    agents_config = os.path.join(params_dir, 'agents.yaml')
    sim_config = os.path.join(params_dir, 'sim.yaml')
    min_curv_planner_config = os.path.join(params_dir, 'min_curv_planner.yaml')
    rviz_config = os.path.join(params_dir, 'multi_rviz.rviz')

    # Read agents.yaml at launch-generation time (same file agent_sim itself
    # consumes at runtime) so the controller count/numbering can never drift
    # from the actual agent count -- no hardcoded N here. Each agent's own
    # "controller:" field picks which builder below runs for that agent.
    with open(agents_config) as f:
        agents_doc = yaml.safe_load(f)
    agents_yaml = agents_doc['agents']
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
        # No built-in controller: an external node (e.g. the learned-weight
        # MPC) publishes the agent's input on /agent_<i>/control.
        'none': lambda *args, **kwargs: [],
    }
    # Both built-in controllers end in pid_controller, which sends
    # [acceleration, steer_rate] via DynamicModel::packAccelSteerRate(). A jerk
    # plant takes that only with command_mode: acceleration -- set by the launch
    # in a derived copy of agents.yaml (see ros_agents_config below), because
    # rl_framework builds its jerk-driven plant from agents.yaml itself.
    JERK_PLANTS = {'SingleTrackAugDynModel'}
    PID_CONTROLLERS = {'pure_pursuit', 'mppi_controller'}

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='agents_yaml',
        choices=['agents_yaml', *CONTROLLER_BUILDERS],
        description="Controller for every agent; 'agents_yaml' uses each agent's "
                    "controller: field, 'none' leaves /agent_<i>/control to an external node"
    )
    accel_time_constant_arg = DeclareLaunchArgument(
        'accel_time_constant',
        default_value='0.05',
        description='Drive-actuator lag [s] of a jerk plant driven by pid_controller '
                    '(dynamics_params.accelTimeConstant, command_mode: acceleration)'
    )

    # Which reference trajectory the controllers follow:
    #   json             -- /global_planner/iqp_trajectory from
    #                       global_waypoints_publisher.py (precomputed by the
    #                       Python global planner)
    #   min_curv_planner -- /min_curv_planner/trajectory, computed at startup
    #                       by the C++ MinCurvPlanner plugin
    reference_source_arg = DeclareLaunchArgument(
        'reference_source',
        default_value='min_curv_planner',
        choices=['json', 'min_curv_planner'],
        description='Reference trajectory source for the controllers'
    )
    reference_topics = {
        'json': '/global_planner/iqp_trajectory',
        'min_curv_planner': '/min_curv_planner/trajectory',
    }

    def ros_agents_config(controllers, accel_time_constant):
        """agents.yaml for the ROS nodes: the scenario file itself, or -- if a pid-based
        controller drives a jerk plant -- a copy in a temp dir with that agent's plant
        switched to command_mode: acceleration. agent_sim and pid_controller must read
        the same file (pid_controller packs its command with its own model copy)."""
        derived = copy.deepcopy(agents_doc)
        switched = []
        for agent_name, controller_type in controllers.items():
            agent = derived['agents'][agent_name]
            dynamics_params = agent.setdefault('dynamics_params', {})
            if (agent.get('dynamics_plugin') in JERK_PLANTS and controller_type in PID_CONTROLLERS
                    and dynamics_params.get('command_mode', 'jerk') != 'acceleration'):
                dynamics_params['command_mode'] = 'acceleration'
                dynamics_params['accelTimeConstant'] = accel_time_constant
                switched.append(agent_name)
        if not switched:
            return agents_config, switched
        path = os.path.join(tempfile.mkdtemp(prefix='ground_vehicle_racing_rl_'), 'agents.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(derived, f, sort_keys=False)
        return path, switched

    def build_reference_and_controllers(context):
        reference_source = LaunchConfiguration('reference_source').perform(context)
        controller_override = LaunchConfiguration('controller').perform(context)
        accel_time_constant = float(LaunchConfiguration('accel_time_constant').perform(context))

        controllers = {}
        for agent_name in agent_names:
            controller_type = (agents_yaml[agent_name].get('controller', 'pure_pursuit')
                               if controller_override == 'agents_yaml' else controller_override)
            if controller_type not in CONTROLLER_BUILDERS:
                raise ValueError(
                    f"{agent_name}: unknown controller '{controller_type}', "
                    f"expected one of {list(CONTROLLER_BUILDERS)}"
                )
            controllers[agent_name] = controller_type
        ros_agents, switched = ros_agents_config(controllers, accel_time_constant)

        nodes = []
        if switched:
            nodes.append(LogInfo(msg=(
                f"{', '.join(switched)}: jerk plant driven by pid_controller -> "
                f"command_mode: acceleration (accelTimeConstant {accel_time_constant} s) "
                f"in {ros_agents}")))
        nodes.append(Node(
            package='agent_sim',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
            parameters=[
                params_config,
                {
                    'sim_config_file': sim_config,
                    'agents_config_file': ros_agents,
                },
            ]
        ))
        if reference_source == 'min_curv_planner':
            # Global raceline from the centerline in global_waypoints.json,
            # computed once with agent_1's vehicle parameters (agents.yaml).
            nodes.append(Node(
                package='planner_base',
                executable='planner_node_exe',
                name='min_curv_planner',
                output='screen',
                parameters=[{
                    'agent_number': 1,
                    'agents_config_file': ros_agents,
                    'planner_config_file': min_curv_planner_config,
                    'planner_plugin': 'MinCurvPlanner',
                    'planner_frequency': 1.0,
                }],
            ))
        for i, agent_name in enumerate(agent_names, start=1):
            builder = CONTROLLER_BUILDERS[controllers[agent_name]]
            nodes.extend(builder(i, sim_config, ros_agents,
                                 reference_topic=reference_topics[reference_source]))
        return nodes

    return LaunchDescription([
        map_arg,
        reference_source_arg,
        controller_arg,
        accel_time_constant_arg,
        # agent_sim is started in build_reference_and_controllers: it must read the
        # same (possibly derived) agents config as the controllers.
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
        OpaqueFunction(function=build_reference_and_controllers),
    ])
