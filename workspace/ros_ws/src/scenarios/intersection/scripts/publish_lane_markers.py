#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Publishes the "Lanes" centerlines from params.yaml as dashed black lines in
RViz -- a LINE_LIST marker per lane, since Marker has no native dashed
line style; the dash is faked by emitting alternating on/off segments
along each lane instead of one continuous LINE_STRIP.

Published once, latched (transient_local), since the lanes are static for
the lifetime of the scenario -- matches global_waypoints_publisher.py's
pattern for the same reason.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def load_ros_params(params_file):
    with open(params_file) as f:
        raw = yaml.safe_load(f)
    return raw.get('/**', {}).get('ros__parameters', raw)


def dashed_segments(start, end, dash_length, gap_length):
    """[(p0, p1), ...] on-segments along start->end, alternating with gaps."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    if length < 1e-9:
        return []
    ux = (end[0] - start[0]) / length
    uy = (end[1] - start[1]) / length

    segments = []
    period = dash_length + gap_length
    t = 0.0
    while t < length:
        dash_end = min(t + dash_length, length)
        p0 = (start[0] + ux * t, start[1] + uy * t)
        p1 = (start[0] + ux * dash_end, start[1] + uy * dash_end)
        segments.append((p0, p1))
        t += period
    return segments


class LaneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('lane_marker_publisher')

        self.declare_parameter('params_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('dash_length', 0.15)
        self.declare_parameter('gap_length', 0.10)
        self.declare_parameter('line_width', 0.02)

        params_file = self.get_parameter('params_file').get_parameter_value().string_value
        if not params_file:
            from ament_index_python.packages import get_package_share_directory
            params_file = get_package_share_directory('scenarios') + \
                '/intersection/params/params.yaml'
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        dash_length = self.get_parameter('dash_length').get_parameter_value().double_value
        gap_length = self.get_parameter('gap_length').get_parameter_value().double_value
        line_width = self.get_parameter('line_width').get_parameter_value().double_value

        ros_params = load_ros_params(params_file)
        lanes = ros_params['Lanes']
        num_lanes = int(lanes['num_lanes'])

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(MarkerArray, '/intersection/lane_markers', latched_qos)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lanes'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = line_width
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        n_segments = 0
        for i in range(1, num_lanes + 1):
            lane = lanes[f'lane_{i}']
            for p0, p1 in dashed_segments(lane['start'], lane['end'], dash_length, gap_length):
                marker.points.append(Point(x=p0[0], y=p0[1], z=0.0))
                marker.points.append(Point(x=p1[0], y=p1[1], z=0.0))
                n_segments += 1

        markers = MarkerArray()
        markers.markers.append(marker)
        self.pub.publish(markers)
        self.get_logger().info(
            f'Published {num_lanes} lane(s) as {n_segments} dash segments on '
            f'/intersection/lane_markers (frame: {frame_id})')


def main():
    rclpy.init()
    node = LaneMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
