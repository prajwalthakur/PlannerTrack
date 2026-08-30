# Author Prajwal Thakur
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# controller_node_exe (hybrid_mode inside pure_pursuit_controller.yaml picks
# regulated_pp/dummy/mpcc) computes a velocity/steering setpoint on
# ~/output/control_cmd; pid_controller closes the loop against this agent's
# own odom/steering_report feedback and is the one that actually owns
# /agent_N/control (EigenVectorStamped), which is what agent_sim's control
# subscriber expects.
def build_nodes(i, sim_config, agents_config, reference_trajectory_topic=None,
                 controller_param_overrides=None):
    """reference_trajectory_topic overrides what this agent's controller
    tracks -- defaults to its own raw /agent_i/reference_trajectory. Used by
    ssc_closed_loop_demo.launch.py to point agent_1's controller at
    /agent_1/planner/ssc/bezier_trajectory instead, so the SSC-computed plan
    actually drives the simulated vehicle rather than just being visualized
    alongside it.

    controller_param_overrides: optional dict layered after
    pure_pursuit_controller.yaml (later ROS2 parameter sources win ties) --
    e.g. {'set_external_target_speed': False} to override this scenario's
    own config on a specific agent without touching the file.
    """
    if reference_trajectory_topic is None:
        reference_trajectory_topic = f'/agent_{i}/reference_trajectory'

    params_dir = os.path.join(
        get_package_share_directory('scenarios'), 'intersection', 'params')
    pure_pursuit_config = os.path.join(params_dir, 'pure_pursuit_controller.yaml')
    pid_controller_config = os.path.join(params_dir, 'pid_controller.yaml')

    controller_params = [
        pure_pursuit_config,
        {
            'sim_config_file': sim_config,
            'agents_config_file': agents_config,
            'agent_number': i,
        },
    ]
    if controller_param_overrides:
        controller_params.append(controller_param_overrides)

    return [
        Node(
            package='trajectory_follower_node',
            executable='controller_node_exe',
            name='controller',
            namespace=f'agent_{i}',
            output='screen',
            parameters=controller_params,
            remappings=[
                ('~/input/reference_trajectory', reference_trajectory_topic),
                ('~/input/current_odometry', f'/agent_{i}/odom'),
                ('~/input/current_steering', f'/agent_{i}/steering_report'),
                ('~/output/control_cmd', f'/agent_{i}/amr_control'),
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
