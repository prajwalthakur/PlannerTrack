#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
The trajectory generator is deliberately simple: constant cruise speed,
decelerating linearly to a stop over the last `decel_distance_m` before the
goal. No curvature-feedforward, no lateral velocity/heading-rate terms --
those TrajectoryPoint fields are left at zero. time_from_start is a
constant-velocity-per-segment integration along the path, not a dynamics
simulation.
"""
import math
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Duration
from nav_msgs.msg import Path
from project_utils_msgs.action import ComputeRoute
from project_utils_msgs.msg import Trajectory, TrajectoryPoint


def path_to_trajectory(path: Path, cruise_speed_mps: float, decel_distance_m: float) -> Trajectory:
    """Constant-speed trajectory with a linear decel ramp into the goal."""
    traj = Trajectory()
    traj.header = path.header

    n = len(path.poses)
    if n == 0:
        return traj

    # Cumulative arc length up to each point, and remaining distance to goal.
    dist_from_start = [0.0] * n
    for i in range(1, n):
        dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x
        dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y
        dist_from_start[i] = dist_from_start[i - 1] + math.hypot(dx, dy)
    total_length = dist_from_start[-1]

    time_from_start = 0.0
    for i in range(n):
        remaining = total_length - dist_from_start[i]
        if remaining < decel_distance_m and decel_distance_m > 1e-6:
            speed = cruise_speed_mps * (remaining / decel_distance_m)
        else:
            speed = cruise_speed_mps
        speed = max(speed, 0.05)  # avoid a literal zero-speed point stalling a controller

        if i > 0:
            seg_dist = dist_from_start[i] - dist_from_start[i - 1]
            avg_speed = 0.5 * (speed + prev_speed)
            if avg_speed > 1e-6:
                time_from_start += seg_dist / avg_speed
        prev_speed = speed

        pt = TrajectoryPoint()
        pt.pose = path.poses[i].pose
        pt.longitudinal_velocity_mps = 0.0 if i == n - 1 else speed
        pt.time_from_start = Duration(
            sec=int(time_from_start), nanosec=int((time_from_start % 1.0) * 1e9))
        traj.points.append(pt)

    return traj


class GlobalWaypointsPublisher(Node):
    def __init__(self):
        super().__init__('global_waypoints_publisher')

        self.declare_parameter('agents_config_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cruise_speed_mps', 1.0)
        self.declare_parameter('decel_distance_m', 0.5)

        agents_config_file = self.get_parameter('agents_config_file').get_parameter_value().string_value
        if not agents_config_file:
            raise RuntimeError('agents_config_file parameter is required')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.cruise_speed_mps = self.get_parameter('cruise_speed_mps').get_parameter_value().double_value
        self.decel_distance_m = self.get_parameter('decel_distance_m').get_parameter_value().double_value

        with open(agents_config_file) as f:
            agents_yaml = yaml.safe_load(f)['agents']

        self._routes_needed = []
        for i, (name, agent) in enumerate(agents_yaml.items(), start=1):
            route = agent.get('route')
            if route is None:
                self.get_logger().warn(f'{name} has no "route" field -- skipping')
                continue
            self._routes_needed.append((i, name, route['start_id'], route['goal_id']))

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._traj_pubs = {
            i: self.create_publisher(Trajectory, f'/agent_{i}/reference_trajectory', latched_qos)
            for i, _, _, _ in self._routes_needed
        }
        # Same dense path route_server already computed, published as-is
        # (no trajectory/velocity fields) for a plain RViz Path display --
        # nothing downstream is meant to subscribe to this, it's viz-only.
        self._path_pubs = {
            i: self.create_publisher(Path, f'/agent_{i}/reference_path', latched_qos)
            for i, _, _, _ in self._routes_needed
        }

        self._action_client = ActionClient(self, ComputeRoute, '/compute_route')

    def run(self):
        self.get_logger().info('Waiting for /compute_route action server...')
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/compute_route action server not available, giving up')
            return

        for i, name, start_id, goal_id in self._routes_needed:
            path = self._compute_route(name, start_id, goal_id)
            if path is None:
                continue
            self._path_pubs[i].publish(path)
            traj = path_to_trajectory(path, self.cruise_speed_mps, self.decel_distance_m)
            self._traj_pubs[i].publish(traj)
            self.get_logger().info(
                f'{name}: published {len(traj.points)}-point trajectory '
                f'(start_id={start_id}, goal_id={goal_id}) on /agent_{i}/reference_trajectory '
                f'and /agent_{i}/reference_path')

    def _compute_route(self, agent_name, start_id, goal_id, max_attempts=10, retry_delay_sec=0.5):
        goal = ComputeRoute.Goal()
        goal.start_id = start_id
        goal.goal_id = goal_id

        # wait_for_server() only confirms the action *server object* exists,
        # which route_server creates in on_configure() -- goals sent before
        # it reaches on_activate() (is_active_ still false) get rejected.
        # Retry rather than race the lifecycle transition.
        goal_handle = None
        for attempt in range(1, max_attempts + 1):
            send_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            self.get_logger().warn(
                f'{agent_name}: route goal rejected (attempt {attempt}/{max_attempts}), '
                f'retrying -- route_server may still be activating')
            goal_handle = None
            time.sleep(retry_delay_sec)

        if goal_handle is None:
            self.get_logger().error(f'{agent_name}: route goal rejected after {max_attempts} attempts')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != ComputeRoute.Result.NONE:
            self.get_logger().error(
                f'{agent_name}: route failed (error_code={result.error_code}): {result.error_msg}')
            return None

        return result.path


def main():
    rclpy.init()
    node = GlobalWaypointsPublisher()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
