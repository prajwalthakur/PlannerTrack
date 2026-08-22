# Author Prajwal Thakur
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# controller_node_exe (hybrid_mode_active/hybrid_mode inside mppi_controller.yaml
# selects MPPIController, trajectory_follower_node/controller_mode.hpp) computes
# a velocity/steering setpoint on ~/output/control_cmd; pid_controller closes the
# loop against this agent's own odom/steering_report feedback and is the one
# that actually owns /agent_N/control (EigenVectorStamped), which is what
# agent_sim's control subscriber expects -- same pattern as pure_pursuit.py.
def build_nodes(i, sim_config, agents_config):
    params_dir = os.path.join(
        get_package_share_directory('scenarios'), 'ground_vehicle_racing', 'params')
    mppi_config = os.path.join(params_dir, 'mppi_controller.yaml')
    pid_controller_config = os.path.join(params_dir, 'pid_controller.yaml')

    # RewrittenYaml (the standard nav2 pattern for
    # per-namespace costmap overrides) rewrites the key wherever it appears
    # in the yaml tree, so it actually reaches
    # /**/local_costmap/local_costmap:robot_base_frame. No leading "/" --
    # must match agent_sim's actual TF frame id (agent_N/base_link, see
    # vehicle_interface_class.cpp).
    mppi_config_for_agent = RewrittenYaml(
        source_file=mppi_config,
        param_rewrites={'robot_base_frame': f'agent_{i}/base_link'},
        convert_types=True,
    )

    return [
        Node(
            package='trajectory_follower_node',
            executable='controller_node_exe',
            name='controller',
            namespace=f'agent_{i}',
            output='screen',
            parameters=[
                mppi_config_for_agent,
                {
                    'sim_config_file': sim_config,
                    'agents_config_file': agents_config,
                    'agent_number': i,
                },
            ],
            remappings=[
                ('~/input/reference_trajectory', f'/agent_{i}/reference_trajectory'),
                ('~/input/current_odometry', f'/agent_{i}/odom'),
                ('~/input/current_steering', f'/agent_{i}/steering_report'),
                ('~/output/control_cmd', f'/agent_{i}/amr_control'),
                ('local_costmap/scan', f'/agent_{i}/scan'),
            ],
        ),
        Node(
            package='pid_controller',
            executable='pid_controller_node_exe',
            name='pid_controller',
            namespace=f'agent_{i}',
            output='screen',
            parameters=[
                pid_controller_config,
                {
                    'sim_config_file': sim_config,
                    'agents_config_file': agents_config,
                    'agent_number': i,
                },
            ],
            remappings=[
                ('~/input/current_odometry', f'/agent_{i}/odom'),
                ('~/input/current_steering', f'/agent_{i}/steering_report'),
                ('~/input/control_cmd', f'/agent_{i}/amr_control'),
                ('~/output/control_cmd', f'/agent_{i}/control'),
            ],
        ),
    ]
