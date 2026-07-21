#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Reads global_waypoints.json (F1TENTH/race_stack global-raceline export) and
publishes three things, each once, latched via transient-local QoS so late
subscribers (RViz, a controller node started afterwards) still receive them:

  /global_planner/centerline          nav_msgs/Path
      x, y, psi (as quaternion) per centerline waypoint. Directly
      RViz-visualizable (Path display) and a standard type any controller
      already knows how to consume.

  /global_planner/iqp_trajectory      project_utils_msgs/Trajectory
      The IQP-optimized raceline: pose + longitudinal_velocity_mps +
      track_kappa_radpm per point -- this is the topic an LQR/PID/MPCC
      controller should subscribe to. RViz has no built-in display for a
      custom message type, so this topic will NOT show up in RViz by
      itself -- see the next topic.

  /global_planner/iqp_trajectory_markers   visualization_msgs/MarkerArray
      Visualization-only companion to the topic above: the same path as a
      single speed-colored LINE_STRIP (blue = slow, red = fast), so the
      IQP trajectory is visible in RViz. Not meant to be subscribed to by
      a controller -- subscribe to /global_planner/iqp_trajectory instead.

  /global_planner/track_boundaries    visualization_msgs/MarkerArray
      The two track-edge point groups from trackbounds_markers (kept as
      two separate POINTS markers, colored to match the two colors already
      used in the source JSON -- order within each group is whatever the
      source file had, not re-sorted along the boundary, so POINTS is used
      instead of LINE_STRIP to avoid drawing a zig-zag if it isn't already
      arc-ordered). marker.points is a plain geometry_msgs/Point[], so this
      is directly usable by a controller for boundary constraints, not
      just for display.

None of s_m/d_m/d_left/d_right are carried into Trajectory/TrajectoryPoint
-- that message has no fields for them.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from project_utils_msgs.msg import Trajectory, TrajectoryPoint


def yaw_to_quaternion(yaw):
	return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def lerp_color(t):
	# t in [0, 1]: 0 -> blue (slow), 1 -> red (fast)
	t = max(0.0, min(1.0, t))
	return (t, 0.0, 1.0 - t)


class GlobalWaypointsPublisher(Node):
	def __init__(self):
		super().__init__('global_waypoints_publisher')

		self.declare_parameter('waypoints_file', '')
		self.declare_parameter('frame_id', 'map')

		waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
		if not waypoints_file:
			from ament_index_python.packages import get_package_share_directory
			waypoints_file = get_package_share_directory('scenarios') + \
				'/ground_vehicle_racing/map/global_waypoints.json'
		self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

		self.get_logger().info(f'Loading global waypoints from: {waypoints_file}')
		with open(waypoints_file) as f:
			data = json.load(f)

		latched_qos = QoSProfile(
			depth=1,
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
			reliability=QoSReliabilityPolicy.RELIABLE,
		)

		self.centerline_pub = self.create_publisher(Path, '/global_planner/centerline', latched_qos)
		self.iqp_traj_pub = self.create_publisher(
			Trajectory, '/global_planner/iqp_trajectory', latched_qos)
		self.iqp_traj_markers_pub = self.create_publisher(
			MarkerArray, '/global_planner/iqp_trajectory_markers', latched_qos)
		self.boundaries_pub = self.create_publisher(
			MarkerArray, '/global_planner/track_boundaries', latched_qos)

		stamp = self.get_clock().now().to_msg()
		self.centerline_pub.publish(self._build_centerline(data, stamp))
		self.iqp_traj_pub.publish(self._build_iqp_trajectory(data, stamp))
		self.iqp_traj_markers_pub.publish(self._build_iqp_trajectory_markers(data, stamp))
		self.boundaries_pub.publish(self._build_track_boundaries(data, stamp))

		n_center = len(data['centerline_waypoints']['wpnts'])
		n_iqp = len(data['global_traj_wpnts_iqp']['wpnts'])
		n_bound = len(data['trackbounds_markers']['markers'])
		self.get_logger().info(
			f'Published {n_center} centerline points, {n_iqp} IQP trajectory points, '
			f'{n_bound} boundary points.')

	def _build_centerline(self, data, stamp):
		path = Path()
		path.header.stamp = stamp
		path.header.frame_id = self.frame_id
		for wp in data['centerline_waypoints']['wpnts']:
			pose = PoseStamped()
			pose.header.stamp = stamp
			pose.header.frame_id = self.frame_id
			pose.pose.position.x = wp['x_m']
			pose.pose.position.y = wp['y_m']
			qx, qy, qz, qw = yaw_to_quaternion(wp['psi_rad'])
			pose.pose.orientation.x = qx
			pose.pose.orientation.y = qy
			pose.pose.orientation.z = qz
			pose.pose.orientation.w = qw
			path.poses.append(pose)
		return path

	def _build_iqp_trajectory(self, data, stamp):
		traj = Trajectory()
		traj.header.stamp = stamp
		traj.header.frame_id = self.frame_id
		for wp in data['global_traj_wpnts_iqp']['wpnts']:
			pt = TrajectoryPoint()
			pt.pose.position.x = wp['x_m']
			pt.pose.position.y = wp['y_m']
			qx, qy, qz, qw = yaw_to_quaternion(wp['psi_rad'])
			pt.pose.orientation.x = qx
			pt.pose.orientation.y = qy
			pt.pose.orientation.z = qz
			pt.pose.orientation.w = qw
			pt.longitudinal_velocity_mps = float(wp['vx_mps'])
			pt.acceleration_mps2 = float(wp['ax_mps2'])
			pt.track_kappa_radpm = float(wp['kappa_radpm'])
			traj.points.append(pt)
		return traj

	def _build_iqp_trajectory_markers(self, data, stamp):
		wpnts = data['global_traj_wpnts_iqp']['wpnts']
		speeds = [wp['vx_mps'] for wp in wpnts]
		vmin, vmax = min(speeds), max(speeds)
		vspan = (vmax - vmin) or 1.0

		marker = Marker()
		marker.header.stamp = stamp
		marker.header.frame_id = self.frame_id
		marker.ns = 'iqp_trajectory'
		marker.id = 0
		marker.type = Marker.LINE_STRIP
		marker.action = Marker.ADD
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.05
		for wp in wpnts:
			p = Point()
			p.x = wp['x_m']
			p.y = wp['y_m']
			marker.points.append(p)
			r, g, b = lerp_color((wp['vx_mps'] - vmin) / vspan)
			from std_msgs.msg import ColorRGBA
			c = ColorRGBA()
			c.r, c.g, c.b, c.a = r, g, b, 1.0
			marker.colors.append(c)

		markers = MarkerArray()
		markers.markers.append(marker)
		return markers

	def _build_track_boundaries(self, data, stamp):
		bound_markers = data['trackbounds_markers']['markers']

		groups = {}
		for m in bound_markers:
			key = (round(m['color']['r'], 2), round(m['color']['g'], 2), round(m['color']['b'], 2))
			groups.setdefault(key, []).append(m['pose']['position'])

		markers = MarkerArray()
		palette = [(0.5, 1.0, 0.0), (0.5, 0.0, 0.5)]
		for idx, (color_key, points) in enumerate(groups.items()):
			marker = Marker()
			marker.header.stamp = stamp
			marker.header.frame_id = self.frame_id
			marker.ns = f'track_boundary_{idx}'
			marker.id = idx
			marker.type = Marker.POINTS
			marker.action = Marker.ADD
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.05
			marker.scale.y = 0.05
			r, g, b = palette[idx % len(palette)]
			marker.color.r = r
			marker.color.g = g
			marker.color.b = b
			marker.color.a = 1.0
			for pos in points:
				p = Point()
				p.x = pos['x']
				p.y = pos['y']
				marker.points.append(p)
			markers.markers.append(marker)
		return markers


def main():
	rclpy.init()
	node = GlobalWaypointsPublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
