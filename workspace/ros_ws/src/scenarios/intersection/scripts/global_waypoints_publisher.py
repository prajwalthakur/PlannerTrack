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
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from project_utils_msgs.action import ComputeRoute
from project_utils_msgs.msg import Trajectory, TrajectoryPoint

# Sentinel route (start_id, goal_id) meaning "stationary, don't call
# route_server at all" -- e.g. a parked/blocking vehicle the SSC corridor
# should route around rather than an agent with somewhere to go.
STATIONARY_ROUTE_ID = -100


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


def stationary_path_and_trajectory(
        x: float, y: float, yaw: float, frame_id: str, stamp,
        duration_s: float = 15.0, dt_s: float = 0.1) -> tuple[Path, Trajectory]:
    """A frozen-in-place Path/Trajectory pair for a stationary agent -- the
    same fixed pose repeated at dt_s spacing out to duration_s, instead of
    anything computed via route_server.

    This project doesn't rasterize static obstacles directly (SscMap has no
    FillStaticPart equivalent -- see ssc_map.hpp's own header comment on
    why); occupancy only ever comes from *published reference
    trajectories*. A stationary agent still needs one, just synthesized
    from its own fixed pose instead of a route.

    Must be densely sampled, not just a start/end pair:
    SscMap::fillMapWithFsVehicleTraj rasterizes one occupancy-grid
    time-slice PER trajectory point, so a sparse trajectory would leave
    most of the grid's time-slices with no occupancy entry at all and the
    corridor would inflate straight through this vehicle undetected. dt_s
    matches ssc_planner.yaml's map_resolution.z (0.10) so every time-slice
    the grid actually has gets covered.
    """
    quat = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
    num_points = int(duration_s / dt_s) + 1

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp

    traj = Trajectory()
    traj.header.frame_id = frame_id
    traj.header.stamp = stamp

    for i in range(num_points):
        t = i * dt_s

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = quat
        path.poses.append(pose)

        pt = TrajectoryPoint()
        pt.pose.position.x = x
        pt.pose.position.y = y
        pt.pose.orientation = quat
        pt.longitudinal_velocity_mps = 0.0
        pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1.0) * 1e9)))
        traj.points.append(pt)

    return path, traj


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
        self._stationary_agents = []
        for i, (name, agent) in enumerate(agents_yaml.items(), start=1):
            route = agent.get('route')
            if route is None:
                self.get_logger().warn(f'{name} has no "route" field -- skipping')
                continue
            start_id, goal_id = route['start_id'], route['goal_id']
            if start_id == STATIONARY_ROUTE_ID and goal_id == STATIONARY_ROUTE_ID:
                dyn = agent.get('dynamics_params', {})
                # float(...) explicitly -- YAML parses e.g. "-3" (no decimal
                # point) as a Python int, and assigning an int straight into
                # a geometry_msgs float64 field passes Python-side but
                # aborts the whole process at publish time (C-level
                # PyFloat_Check assertion in the rosidl-generated
                # converter) -- crashed this entire node, not just the
                # stationary agent, since this runs before anything else in
                # run().
                self._stationary_agents.append((
                    i, name,
                    float(dyn.get('initXPose', 0.0)), float(dyn.get('initYPose', 0.0)),
                    float(dyn.get('initYaw', 0.0))))
                continue
            self._routes_needed.append((i, name, start_id, goal_id))

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        all_agent_ids = [i for i, *_ in self._routes_needed] + [i for i, *_ in self._stationary_agents]
        self._traj_pubs = {
            i: self.create_publisher(Trajectory, f'/agent_{i}/reference_trajectory', latched_qos)
            for i in all_agent_ids
        }
        # Same dense path route_server already computed, published as-is
        # (no trajectory/velocity fields) for a plain RViz Path display --
        # nothing downstream is meant to subscribe to this, it's viz-only.
        self._path_pubs = {
            i: self.create_publisher(Path, f'/agent_{i}/reference_path', latched_qos)
            for i in all_agent_ids
        }

        self._action_client = ActionClient(self, ComputeRoute, '/compute_route')

    def run(self):
        # No route_server dependency at all -- publish these first so they
        # aren't held up by (or lost to) the wait below.
        self._publish_stationary(self._stationary_agents)

        if not self._routes_needed:
            return

        self.get_logger().info('Waiting for /compute_route action server...')
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/compute_route action server not available, giving up')
            return
        self._publish_routed(self._routes_needed)

    def _publish_stationary(self, agents):
        for i, name, x, y, yaw in agents:
            path, traj = stationary_path_and_trajectory(x, y, yaw, self.frame_id, self.get_clock().now().to_msg())
            self._path_pubs[i].publish(path)
            self._traj_pubs[i].publish(traj)
            self.get_logger().info(
                f'{name}: published stationary {len(traj.points)}-point trajectory '
                f'(x={x}, y={y}, yaw={yaw}) on /agent_{i}/reference_trajectory '
                f'and /agent_{i}/reference_path')

    def _publish_routed(self, agents):
        for i, name, start_id, goal_id in agents:
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
