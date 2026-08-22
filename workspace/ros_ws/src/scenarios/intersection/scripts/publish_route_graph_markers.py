#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Publishes route_graph.geojson as RViz markers -- one colored LINE_LIST per
edge class, plus small dots for every node.

Edge position comes from looking up each edge's startid/endid against the
node coordinates, exactly like nav2_route's own GeoJsonGraphFileLoader
does (confirmed by reading geojson_graph_file_loader.cpp: it never reads
an edge feature's own "geometry" field, only properties.startid/endid) --
NOT from the edge feature's own geometry.coordinates. That matters for
hand-authored/GUI-edited graphs, which commonly emit
"geometry": {"type": "MultiLineString"} with no coordinates at all and
rely entirely on the startid/endid + node lookup, same as here.

Published once, latched (transient_local), since the graph is static for
the lifetime of the scenario.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# RGB per edge "class" metadata value written by generate_route_graph.py.
CLASS_COLORS = {
    'approach': (0.2, 0.4, 1.0),
    'departure': (0.0, 0.8, 0.8),
    'straight_through': (1.0, 0.85, 0.0),
    'left_turn': (1.0, 0.15, 0.15),
    'right_turn': (0.15, 0.8, 0.15),
}
DEFAULT_COLOR = (0.6, 0.6, 0.6)


class RouteGraphMarkerPublisher(Node):
    def __init__(self):
        super().__init__('route_graph_marker_publisher')

        self.declare_parameter('graph_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('line_width', 0.02)
        self.declare_parameter('node_size', 0.04)

        graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        if not graph_file:
            from ament_index_python.packages import get_package_share_directory
            graph_file = get_package_share_directory('scenarios') + \
                '/intersection/map/route_graph.geojson'
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        line_width = self.get_parameter('line_width').get_parameter_value().double_value
        node_size = self.get_parameter('node_size').get_parameter_value().double_value

        with open(graph_file) as f:
            graph = json.load(f)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(
            MarkerArray, '/intersection/route_graph_markers', latched_qos)

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Pass 1: every node's coordinates, keyed by its "id" property --
        # this is the only reliable way to place an edge (see module
        # docstring: edge geometry itself is not trustworthy/present).
        node_coords = {}
        node_points = []
        for feature in graph['features']:
            if feature['geometry']['type'] == 'Point':
                x, y = feature['geometry']['coordinates']
                node_id = feature['properties']['id']
                node_coords[node_id] = (x, y)
                node_points.append(Point(x=x, y=y, z=0.01))

        # Pass 2: every edge (anything with startid/endid), positioned via
        # the node lookup above.
        edge_markers = {}
        edge_count = {}
        skipped = 0
        for feature in graph['features']:
            props = feature['properties']
            if 'startid' not in props or 'endid' not in props:
                continue
            start = node_coords.get(props['startid'])
            end = node_coords.get(props['endid'])
            if start is None or end is None:
                skipped += 1
                continue
            cls = props.get('metadata', {}).get('class', 'unknown')
            if cls not in edge_markers:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = f'route_graph_{cls}'
                marker.id = 0
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = line_width
                r, g, b = CLASS_COLORS.get(cls, DEFAULT_COLOR)
                marker.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
                edge_markers[cls] = marker
                edge_count[cls] = 0
            edge_markers[cls].points.append(Point(x=start[0], y=start[1], z=0.01))
            edge_markers[cls].points.append(Point(x=end[0], y=end[1], z=0.01))
            edge_count[cls] += 1

        markers.markers.extend(edge_markers.values())
        if skipped:
            self.get_logger().warn(f'{skipped} edge(s) reference a startid/endid not found '
                                    f'among the graph\'s nodes -- skipped.')

        node_marker = Marker()
        node_marker.header.frame_id = frame_id
        node_marker.header.stamp = stamp
        node_marker.ns = 'route_graph_nodes'
        node_marker.id = 0
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.pose.orientation.w = 1.0
        node_marker.scale.x = node_size
        node_marker.scale.y = node_size
        node_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        node_marker.points = node_points
        markers.markers.append(node_marker)

        self.pub.publish(markers)
        summary = ', '.join(f'{cls}={n}' for cls, n in edge_count.items())
        self.get_logger().info(
            f'Published route graph: {len(node_points)} nodes, edges [{summary}] '
            f'on /intersection/route_graph_markers (frame: {frame_id})')


def main():
    rclpy.init()
    node = RouteGraphMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
