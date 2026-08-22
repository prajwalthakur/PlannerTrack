#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Generates the approach/departure portion of a nav2_route-compatible
GeoJSON route graph from the "Lanes" block in params.yaml -- the same
lane centerlines map_generator already draws (as non-obstacle
references) and reset_state.py already uses for its frame bound. Keeping
this derived from that one source means it can never drift out of sync
with the lane geometry.

Each of the 8 approach/departure lanes is uniformly subdivided into
"--segments-per-lane" (default 4) straight edges, i.e. 3 new intermediate
nodes per lane. start/end coordinates are taken exactly as authored in
params.yaml -- only the subdivision is new.

Intersection turn connectors (linking an incoming lane's boundary node to
an outgoing lane's) are deliberately NOT generated here -- those are
authored by hand on top of this file's output. See
generate_route_graph.py for the earlier auto-generated-connector version
kept as reference (route_graph_old.geojson).

Output matches the schema documented in nav2_route/README.md ("File
Formats"): Point features for nodes (id, frame, metadata), LineString
features for edges (id, startid, endid, metadata), ids unique across both
nodes and edges (matching nav2_route/graphs/sample_graph.geojson).

nav2_route is not built yet in this workspace -- this only produces the
graph file for when it is; nothing here depends on nav2_route itself.
"""
import argparse
import json
import math
import os

import yaml


def load_ros_params(params_file):
    with open(params_file) as f:
        raw = yaml.safe_load(f)
    return raw.get('/**', {}).get('ros__parameters', raw)


def classify_lane(start, end):
    """(arm, direction) purely from geometry -- no hardcoded lane table.

    direction: 'incoming' if the lane's end is closer to the origin than
    its start (i.e. flows toward the intersection), else 'outgoing'.
    arm: whichever endpoint is farther from the origin determines the
    compass arm the lane belongs to.
    """
    d_start = math.hypot(start[0], start[1])
    d_end = math.hypot(end[0], end[1])
    direction = 'incoming' if d_end < d_start else 'outgoing'
    far_point = start if d_start > d_end else end

    if abs(end[1] - start[1]) > abs(end[0] - start[0]):
        arm = 'north' if far_point[1] > 0 else 'south'
    else:
        arm = 'east' if far_point[0] > 0 else 'west'
    return arm, direction


def lerp(p0, p1, t):
    return (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)


class GraphBuilder:
    def __init__(self, frame):
        self.frame = frame
        self._next_id = 0
        self.node_features = []
        self.edge_features = []

    def add_node(self, point, metadata=None):
        node_id = self._next_id
        self._next_id += 1
        props = {'id': node_id, 'frame': self.frame}
        if metadata:
            props['metadata'] = metadata
        self.node_features.append({
            'type': 'Feature',
            'properties': props,
            'geometry': {'type': 'Point', 'coordinates': [round(point[0], 6), round(point[1], 6)]},
        })
        return node_id

    def add_edge(self, start_id, end_id, start_pt, end_pt, metadata=None):
        edge_id = self._next_id
        self._next_id += 1
        props = {'id': edge_id, 'startid': start_id, 'endid': end_id}
        if metadata:
            props['metadata'] = metadata
        self.edge_features.append({
            'type': 'Feature',
            'properties': props,
            'geometry': {
                'type': 'LineString',
                'coordinates': [
                    [round(start_pt[0], 6), round(start_pt[1], 6)],
                    [round(end_pt[0], 6), round(end_pt[1], 6)],
                ],
            },
        })
        return edge_id


def build_lane(gb, lane_idx, lane, n_segments):
    start, end = lane['start'], lane['end']
    arm, direction = classify_lane(start, end)
    points = [lerp(start, end, i / n_segments) for i in range(n_segments + 1)]

    node_ids = []
    for i, pt in enumerate(points):
        role = 'terminal' if i in (0, n_segments) else 'interior'
        node_ids.append(gb.add_node(pt, {
            'lane': lane_idx, 'arm': arm, 'direction': direction,
            't': round(i / n_segments, 3), 'role': role,
        }))

    edge_class = 'approach' if direction == 'incoming' else 'departure'
    for i in range(n_segments):
        gb.add_edge(node_ids[i], node_ids[i + 1], points[i], points[i + 1], {
            'class': edge_class, 'arm': arm, 'lane': lane_idx,
            'speed_limit': 100.0,
        })


def generate_lane_graph(params_file, output_file, segments_per_lane):
    ros_params = load_ros_params(params_file)
    lanes = ros_params['Lanes']
    num_lanes = int(lanes['num_lanes'])

    frame = 'map'
    gb = GraphBuilder(frame)

    for i in range(1, num_lanes + 1):
        lane = lanes[f'lane_{i}']
        build_lane(gb, i, lane, segments_per_lane)

    geojson = {
        'type': 'FeatureCollection',
        'name': 'intersection_lane_graph',
        'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
        'features': gb.node_features + gb.edge_features,
    }

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(geojson, f, indent=2)

    print(f'{len(gb.node_features)} nodes, {len(gb.edge_features)} edges -> {output_file}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate the approach/departure-only nav2_route GeoJSON graph '
                     'from params.yaml Lanes (no intersection connectors -- authored by hand)')
    parser.add_argument(
        '--params_file',
        default=None,
        help='Path to intersection/params/params.yaml (default: package share dir)')
    parser.add_argument(
        '--output',
        default=None,
        help='Output .geojson path (default: intersection/map/lane_graph.geojson)')
    parser.add_argument('--segments-per-lane', type=int, default=4)
    args = parser.parse_args()

    params_file = args.params_file
    output_file = args.output
    if params_file is None or output_file is None:
        from ament_index_python.packages import get_package_share_directory
        scenario_dir = os.path.join(get_package_share_directory('scenarios'), 'intersection')
        if params_file is None:
            params_file = os.path.join(scenario_dir, 'params', 'params.yaml')
        if output_file is None:
            output_file = os.path.join(scenario_dir, 'map', 'lane_graph.geojson')

    generate_lane_graph(params_file, output_file, args.segments_per_lane)


if __name__ == '__main__':
    main()
