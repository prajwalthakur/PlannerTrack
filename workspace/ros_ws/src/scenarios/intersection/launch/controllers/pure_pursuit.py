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
def build_nodes(i, sim_config, agents_config):
    params_dir = os.path.join(
        get_package_share_directory('scenarios'), 'ground_vehicle_racing', 'params')
    pure_pursuit_config = os.path.join(params_dir, 'pure_pursuit_controller.yaml')
    pid_controller_config = os.path.join(params_dir, 'pid_controller.yaml')

    return [
        Node(
            package='trajectory_follower_node',
            executable='controller_node_exe',
            name='controller',
            namespace=f'agent_{i}',
            output='screen',
            parameters=[
                pure_pursuit_config,
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
