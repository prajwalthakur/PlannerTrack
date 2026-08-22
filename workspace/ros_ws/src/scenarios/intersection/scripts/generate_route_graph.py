#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Generates a nav2_route-compatible GeoJSON route graph from the "Lanes"
block in params.yaml -- the same lane centerlines map_generator already
draws (as non-obstacle references) and reset_state.py already uses for its
frame bound. Keeping the route graph derived from that one source means it
can never drift out of sync with the lane geometry.

Graph construction:
  1. Each of the 8 approach/departure lanes is uniformly subdivided into
     "--segments-per-lane" (default 4) straight edges, i.e. 3 new
     intermediate nodes per lane. start/end coordinates are taken exactly
     as authored in params.yaml -- only the subdivision is new.
  2. Every legal turning movement through the intersection (straight,
     left, right -- 3 per incoming arm, 12 total for a 4-way) is added as
     a connector. Straight-through is a direct line. Left/right turns are
     a tight corner fillet: straight lead-in from the incoming lane's
     boundary node, a small circular arc of radius "--turn-radius"
     (default 0.15m, capped to the natural tangent-line distance) right
     at the corner, straight lead-out to the outgoing lane's boundary
     node -- sampled into "--segments-per-turn" (default 2) arc edges,
     denser toward the exit end.

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


# Clockwise compass order -- used to classify a turn from the incoming arm's
# index offset to the outgoing arm's index: +1 = left, +2 = straight,
# +3 (== -1) = right. See the reasoning in the design discussion this
# script implements: standing at the south approach heading north, turning
# toward west (clockwise-next after south) is a left turn, toward east
# (clockwise-previous) is a right turn.
ARMS_CW = ['north', 'east', 'south', 'west']
MOVEMENT_BY_OFFSET = {1: 'left_turn', 2: 'straight_through', 3: 'right_turn'}


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


def unit_heading(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    n = math.hypot(dx, dy) or 1.0
    return (dx / n, dy / n)


def corner_fillet_points(p_in, heading_in, p_out, heading_out, radius, n_segments):
    """A tight corner fillet: straight lead-in from p_in, a small circular
    arc of the given radius right at the corner, straight lead-out to
    p_out. Same tangent-line-intersection "corner" construction
    nav2_route's own CornerArc uses at runtime (see corner_smoothing.hpp),
    but with an explicit, caller-chosen radius instead of the (much
    larger) radius forced by using the full distance to that corner --
    that full-distance radius is geometrically valid but, for a 90 degree
    turn between two lane offsets a fraction of a metre apart, produces
    an arc far bigger than a real curb-corner fillet. Capped to never
    exceed the natural tangent-line distance, so it degrades gracefully
    back to that full arc if the requested radius is large.

    Every turn in this crossroad pairs one axis-aligned arm with a
    perpendicular one, so the two tangent lines (through p_in along
    heading_in, through p_out along heading_out) are always
    perpendicular and meet at exactly one corner point, equidistant from
    p_in and p_out by this intersection's symmetry. Only valid for a 90
    degree turn; straight-through (parallel headings) is handled by the
    caller without calling this.

    Arc points are sampled denser toward the exit than the entry.
    """
    if abs(heading_in[0]) > abs(heading_in[1]):
        corner = (p_out[0], p_in[1])
    else:
        corner = (p_in[0], p_out[1])

    d_in = math.hypot(p_in[0] - corner[0], p_in[1] - corner[1])
    d_out = math.hypot(p_out[0] - corner[0], p_out[1] - corner[1])
    r = min(radius, d_in, d_out)

    tangent_in = (corner[0] - heading_in[0] * r, corner[1] - heading_in[1] * r)
    tangent_out = (corner[0] - heading_out[0] * r, corner[1] - heading_out[1] * r)

    theta_start = math.atan2(tangent_in[1] - corner[1], tangent_in[0] - corner[0])
    theta_end = math.atan2(tangent_out[1] - corner[1], tangent_out[0] - corner[0])

    delta = theta_end - theta_start
    # Normalize to the shorter (90 degree) sweep direction.
    if delta > math.pi:
        delta -= 2 * math.pi
    elif delta < -math.pi:
        delta += 2 * math.pi

    arc_points = []
    for i in range(n_segments + 1):
        u = i / n_segments
        t = 1.0 - (1.0 - u) ** 2  # denser spacing as u -> 1 (toward exit)
        arc_points.append((
            corner[0] + r * math.cos(theta_start + delta * t),
            corner[1] + r * math.sin(theta_start + delta * t),
        ))

    points = []
    if math.hypot(p_in[0] - tangent_in[0], p_in[1] - tangent_in[1]) > 1e-9:
        points.append(p_in)
    points.extend(arc_points)
    if math.hypot(p_out[0] - tangent_out[0], p_out[1] - tangent_out[1]) > 1e-9:
        points.append(p_out)
    return points


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

    boundary_idx = n_segments if direction == 'incoming' else 0
    boundary_node_id = node_ids[boundary_idx]
    boundary_point = points[boundary_idx]
    # Heading in direction of travel, regardless of incoming/outgoing.
    heading = unit_heading(start, end)
    return arm, direction, boundary_node_id, boundary_point, heading


def build_connector(gb, arm_in, arm_out, boundary_in, boundary_out, n_segments, turn_radius):
    node_in_id, point_in, heading_in = boundary_in
    node_out_id, point_out, heading_out = boundary_out

    i_in = ARMS_CW.index(arm_in)
    i_out = ARMS_CW.index(arm_out)
    movement = MOVEMENT_BY_OFFSET[(i_out - i_in) % 4]

    if movement == 'straight_through':
        points = [lerp(point_in, point_out, i / n_segments) for i in range(n_segments + 1)]
    else:
        points = corner_fillet_points(
            point_in, heading_in, point_out, heading_out, turn_radius, n_segments)
    node_ids = [node_in_id]
    for pt in points[1:-1]:
        node_ids.append(gb.add_node(pt, {
            'connector': f'{arm_in}_to_{arm_out}', 'class': movement,
        }))
    node_ids.append(node_out_id)

    speed_limit = 100.0 if movement == 'straight_through' else 40.0
    for i in range(len(points) - 1):
        gb.add_edge(node_ids[i], node_ids[i + 1], points[i], points[i + 1], {
            'class': movement, 'from_arm': arm_in, 'to_arm': arm_out,
            'speed_limit': speed_limit,
        })


def generate_route_graph(params_file, output_file, segments_per_lane, segments_per_turn, turn_radius):
    ros_params = load_ros_params(params_file)
    lanes = ros_params['Lanes']
    num_lanes = int(lanes['num_lanes'])

    frame = 'map'
    gb = GraphBuilder(frame)

    # boundary[(arm, direction)] = (node_id, point, heading)
    boundary = {}
    for i in range(1, num_lanes + 1):
        lane = lanes[f'lane_{i}']
        arm, direction, node_id, point, heading = build_lane(gb, i, lane, segments_per_lane)
        boundary[(arm, direction)] = (node_id, point, heading)

    for arm_in in ARMS_CW:
        if (arm_in, 'incoming') not in boundary:
            continue
        for offset in (1, 2, 3):
            arm_out = ARMS_CW[(ARMS_CW.index(arm_in) + offset) % 4]
            if (arm_out, 'outgoing') not in boundary:
                continue
            build_connector(
                gb, arm_in, arm_out,
                boundary[(arm_in, 'incoming')], boundary[(arm_out, 'outgoing')],
                segments_per_turn, turn_radius)

    geojson = {
        'type': 'FeatureCollection',
        'name': 'intersection_route_graph',
        'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
        'features': gb.node_features + gb.edge_features,
    }

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(geojson, f, indent=2)

    print(f'{len(gb.node_features)} nodes, {len(gb.edge_features)} edges -> {output_file}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate a nav2_route GeoJSON graph from params.yaml Lanes')
    parser.add_argument(
        '--params_file',
        default=None,
        help='Path to intersection/params/params.yaml (default: package share dir)')
    parser.add_argument(
        '--output',
        default=None,
        help='Output .geojson path (default: intersection/map/route_graph.geojson)')
    parser.add_argument('--segments-per-lane', type=int, default=4)
    parser.add_argument('--segments-per-turn', type=int, default=4)
    parser.add_argument(
        '--turn-radius', type=float, default=0.15,
        help='Corner fillet radius (m) for left/right turns, capped to the natural '
             'tangent-line distance (0.25m for a right turn, 0.75m for a left turn '
             'in this intersection) if set larger than that.')
    args = parser.parse_args()

    params_file = args.params_file
    output_file = args.output
    if params_file is None or output_file is None:
        from ament_index_python.packages import get_package_share_directory
        scenario_dir = os.path.join(get_package_share_directory('scenarios'), 'intersection')
        if params_file is None:
            params_file = os.path.join(scenario_dir, 'params', 'params.yaml')
        if output_file is None:
            output_file = os.path.join(scenario_dir, 'map', 'route_graph.geojson')

    generate_route_graph(
        params_file, output_file, args.segments_per_lane, args.segments_per_turn,
        args.turn_radius)


if __name__ == '__main__':
    main()
