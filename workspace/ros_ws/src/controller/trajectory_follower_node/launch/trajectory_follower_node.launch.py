import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     config_dir = os.path.join(get_package_share_directory('project_utils'), 'config')

#     pkg_dir = get_package_share_directory("trajectory_follower_node")

#     params_file = os.path.join(
#         config_dir,
#         "params.yaml"
#     )

#     traj_follower_node = Node(
#         package="trajectory_follower_node",
#         executable="controller_node_exe",   # name from add_executable
#         name="controller_node",
#         output="screen",
#         parameters=[params_file],
#         arguments=['--ros-args', '--log-level', 'info']
#     )
#     container = ComposableNodeContainer(
#         name='controller_container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container_mt',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='trajectory_follower_node',
#                 plugin='mpl::control::trajectory_follower_node::Controller',
#                 name='mppi_controller'
#             )
#         ],
#     )
#     return LaunchDescription([
#         traj_follower_node
#         container
#     ])

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('project_utils'),
        'config'
    )

    params_file = os.path.join(config_dir, "params.yaml")

    container = ComposableNodeContainer(
        name='controller_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='trajectory_follower_node',
                plugin='mpl::control::trajectory_follower_node::Controller',
                name='controller',
                parameters=[params_file],
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        container
    ])