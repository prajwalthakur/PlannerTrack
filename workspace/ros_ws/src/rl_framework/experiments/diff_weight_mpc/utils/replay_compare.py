#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Static weights vs the Diff-WMPC policy, side by side in RViz (a recording, not a live controller).

Both controllers first drive one lap in the Python simulator from the same start (the plant, MPC and evaluation of
baseline_suite.py, so the replay shows the same behaviour as the result tables). The saved states are then published
as markers on /replay/markers in slow motion, on a loop: both cars, their trails, the point of largest |n| so far and
the current values. Poses are interpolated between the 50 Hz simulation steps, so 0.25× does not look choppy.

Start it with the launch file (map, RViz and this node) from the workspace root, after `source install/setup.bash`:

    ros2 launch src/rl_framework/experiments/diff_weight_mpc/utils/replay_compare.launch.py
    ros2 launch src/rl_framework/experiments/diff_weight_mpc/utils/replay_compare.launch.py speed:=0.1 s0:=3.42
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import torch
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # experiments/: the diff_weight_mpc package
from baseline_suite import CONFIGS  # (same folder)
from geometry_msgs.msg import Point
from plannertrack_py import ReferenceTrack, compute_raceline
from plannertrack_py.batch_plant import BatchPlant
from diff_weight_mpc.closed_loop import run_episodes
from diff_weight_mpc.learning.policy_network import WeightPolicy
from diff_weight_mpc.mpc.controller import MpcController
from diff_weight_mpc.mpc.ocp import OcpConfig
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from train_mpc import REF_COLOR, SERIES, _to_theta  # (same folder)
from visualization_msgs.msg import Marker, MarkerArray

RL_FRAMEWORK = Path(__file__).resolve().parents[3]
SRC = RL_FRAMEWORK.parent
DATA = RL_FRAMEWORK / "data"
FRAME = "map"
RATE = 30.0  # marker updates per wall-clock second
END_HOLD = 1.5  # wall-clock seconds to hold the final picture before the loop restarts
TRAIL_WIDTH, EDGE_WIDTH = 0.015, 0.02
WHEEL_LENGTH, WHEEL_WIDTH = 0.10, 0.035
INK = "#1f1f1d"
Z_MAP, Z_CAR, Z_TRAIL, Z_TEXT = 0.01, 0.03, 0.05, 0.1  # draw order in the top-down view


def rgba(color, alpha=1.0):
    r, g, b = (int(color[i:i + 2], 16) / 255.0 for i in (1, 3, 5))
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def static_theta(run, update):
    """θ (6,) of a static run's checkpoint."""
    ckpt = torch.load(DATA / run / "checkpoints" / f"update_{update:04d}.pt", weights_only=False)  # our own file
    with torch.no_grad():
        return _to_theta(ckpt["w"], ckpt["weight_param"]).numpy()


def record(plant, mpc, vehicle, s0, seed, **weights):
    """One lap from s0 with fixed weights (theta=) or a policy (theta_fn=): the per-step values the replay needs."""
    log = run_episodes(plant, mpc, s0=[s0], seed=seed, vehicle_params=vehicle, **weights)
    m = log.active[:, 0]
    state = log.state[m, 0]
    return {
        "x": state[:, 0], "y": state[:, 1], "yaw": np.unwrap(state[:, 2]), "steer": state[:, 6],
        "s": log.s[m, 0], "n": log.n[m, 0], "dv": log.dv[m, 0], "margin": log.margin[m, 0],
        "steer_rate": log.u[m, 0, 1], "dt": log.dt, "end_reason": log.end_reason[0],
    }


def line(ns, points, color, width, marker_id=0, kind=Marker.LINE_STRIP, z=Z_MAP):
    marker = Marker(ns=ns, id=marker_id, type=kind, action=Marker.ADD, color=color)
    marker.header.frame_id = FRAME
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width)
    marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y in points]
    return marker


def box(ns, marker_id, x, y, yaw, length, width, color, z=Z_CAR):
    marker = Marker(ns=ns, id=marker_id, type=Marker.CUBE, action=Marker.ADD, color=color)
    marker.header.frame_id = FRAME
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(x), float(y), float(z)
    marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    marker.scale.x, marker.scale.y, marker.scale.z = float(length), float(width), 0.01
    return marker


def text(ns, marker_id, x, y, value, size, color):
    marker = Marker(ns=ns, id=marker_id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, color=color, text=value)
    marker.header.frame_id = FRAME
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(x), float(y), Z_TEXT
    marker.pose.orientation.w = 1.0
    marker.scale.z = float(size)
    return marker


def track_markers(reference, track_margin):
    """Raceline, real track edges and the MPC's tightened limits (lines at a car-side offset of 0 and track_margin)."""
    s = np.linspace(0.0, reference.length, 600)
    r = reference.sample(s)
    normal = np.c_[-np.sin(r["psi"]), np.cos(r["psi"])]
    xy = np.c_[r["x"], r["y"]]
    left, right = xy + normal * r["w_left"][:, None], xy - normal * r["w_right"][:, None]
    markers = [line("raceline", xy, rgba(REF_COLOR, 0.8), 0.008),
               line("track_edges", left, rgba(INK), EDGE_WIDTH, 0),
               line("track_edges", right, rgba(INK), EDGE_WIDTH, 1)]
    dashes = np.arange(len(s) - 1)[::2]  # every other segment of the fine sampling: a dashed line
    for i, edge in enumerate((left - normal * track_margin, right + normal * track_margin)):
        segments = np.stack((edge[dashes], edge[dashes + 1]), axis=1).reshape(-1, 2)
        markers.append(line("mpc_limit", segments, rgba(REF_COLOR, 0.7), 0.006, i, Marker.LINE_LIST))
    return markers


class Replay(Node):
    def __init__(self, runs, reference, length, width, lf, lr, track_margin, speed, readout_xy):
        super().__init__("replay_compare")
        self.runs, self.reference, self.speed = runs, reference, speed
        self.length, self.width, self.lf, self.lr = length, width, lf, lr
        self.readout_xy = readout_xy
        self.static = track_markers(reference, track_margin)
        self.pub = self.create_publisher(MarkerArray, "/replay/markers", 10)
        self.dt = runs[0]["dt"]
        self.duration = max(len(r["x"]) for r in runs) * self.dt
        self.play_time = 0.0
        self.tick = 0
        self.create_timer(1.0 / RATE, self.step)

    def car(self, index, run, k, a):
        """Markers of one car at step k + a (0 <= a < 1): body, wheels, trail, largest |n| so far."""
        name = run["name"]
        color = run["color"]
        x, y, yaw, steer = (run[c][k] + a * (run[c][k + 1] - run[c][k]) for c in ("x", "y", "yaw", "steer"))
        c, s = math.cos(yaw), math.sin(yaw)
        wheel = rgba(INK, 0.9)
        half = 0.5 * self.width - 0.5 * WHEEL_WIDTH
        markers = [box(name, 0, x, y, yaw, self.length, self.width, rgba(color, 0.45))]
        for i, (along, side, wheel_yaw) in enumerate(((self.lf, half, yaw + steer), (self.lf, -half, yaw + steer),
                                                      (-self.lr, half, yaw), (-self.lr, -half, yaw))):
            markers.append(box(name, 1 + i, x + along * c - side * s, y + along * s + side * c, wheel_yaw,
                               WHEEL_LENGTH, WHEEL_WIDTH, wheel, Z_CAR + 0.005))
        trail = np.vstack((np.c_[run["x"][:k + 1], run["y"][:k + 1]], [[x, y]]))
        markers.append(line(name + "_trail", trail, rgba(color), TRAIL_WIDTH, z=Z_TRAIL))
        worst = int(np.abs(run["n"][:k + 1]).argmax())
        wx, wy = run["x"][worst], run["y"][worst]
        dot = box(name + "_worst", 0, wx, wy, 0.0, 0.05, 0.05, rgba(color), Z_TEXT)
        dot.type = Marker.SPHERE
        dot.scale.z = 0.05
        markers.append(dot)
        label_x = wx + (0.62 if index == 0 else -0.62)  # beside the track line, one car to each side
        markers.append(text(name + "_worst", 1, label_x, wy, f"{name} max |n| {100 * abs(run['n'][worst]):.1f} cm", 0.09,
                            rgba(INK)))
        return markers

    def readout(self, step):
        """The current values, three lines beside the S-bend (updated every frame, one colour per car)."""
        x, y = self.readout_xy
        markers = [text("readout", 0, x, y, f"t = {self.play_time:.2f} s    playback {self.speed:g}×", 0.11, rgba(INK))]
        for i, run in enumerate(self.runs):
            k = min(step, len(run["x"]) - 1)
            markers.append(text("readout", 1 + i, x, y - 0.17 * (i + 1),
                                f"{run['name']}:  n {100 * run['n'][k]:+.1f} cm   dv {run['dv'][k]:+.2f} m/s   margin "
                                f"{100 * run['margin'][k]:+.1f} cm   steer {abs(run['steer_rate'][k]):.2f} rad/s",
                                0.11, rgba(run["color"])))
        return markers

    def step(self):
        step_f = self.play_time / self.dt
        step = int(step_f)
        markers = []
        for index, run in enumerate(self.runs):
            last = len(run["x"]) - 1
            k = min(step, last - 1)
            a = min(step_f - k, 1.0)
            markers += self.car(index, run, k, a)
        markers += self.readout(step)
        if self.tick % int(RATE) == 0:  # the track does not change: once per second, so a restarted RViz gets it
            markers += self.static
        stamp = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = stamp
        self.pub.publish(MarkerArray(markers=markers))
        self.tick += 1
        self.play_time += self.speed / RATE
        if self.play_time > self.duration + END_HOLD * self.speed:  # restart: the trails start empty again
            self.play_time = 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--s0", type=float, default=1.711, help="start arc length [m] (default: settles before the S-bend)")
    ap.add_argument("--speed", type=float, default=0.25, help="playback speed (1 = real time)")
    ap.add_argument("--config", default="nominal car", choices=[c[0] for c in CONFIGS])
    ap.add_argument("--static-run", default="train_static/theta0_sp1e-1_lat1_v1_400")
    ap.add_argument("--static-update", type=int, default=310)
    ap.add_argument("--policy-file", type=Path,
                    default=DATA / "train_policy" / "wmpc_hidden_none" / "policy_update140.pt")
    ap.add_argument("--readout-xy", type=float, nargs=2, default=None, help="map position of the text readout [m]")
    ap.add_argument("--scenario", default="ground_vehicle_racing_rl")
    ap.add_argument("--agent", default="agent_1")
    ap.add_argument("--seed", type=int, default=0)
    args, ros_args = ap.parse_known_args()

    params = SRC / "scenarios" / args.scenario / "params"
    agent = yaml.safe_load((params / "agents.yaml").read_text())["agents"][args.agent]
    geometry, dynamics = agent["geometry_params"], agent["dynamics_params"]
    reference = ReferenceTrack(compute_raceline(params / "agents.yaml", args.agent, params / "min_curv_planner.yaml"))
    _, perturbation, oracle = next(c for c in CONFIGS if c[0] == args.config)
    plant = BatchPlant.from_scenario(params, n_cars=1, agent_name=args.agent, reference=reference,
                                     perturbations=[perturbation])
    cfg = OcpConfig()
    mpc = MpcController.create_controller(plant, cfg=cfg)
    vehicle = [plant.plants[0].true_params if oracle else plant.plants[0].nominal_params]
    policy = WeightPolicy.load(args.policy_file)
    policy.eval()
    runs = [
        {"name": "Static", "color": SERIES[0],
         **record(plant, mpc, vehicle, args.s0, args.seed, theta=static_theta(args.static_run, args.static_update))},
        {"name": "Policy", "color": SERIES[1],
         **record(plant, mpc, vehicle, args.s0, args.seed, theta=None, theta_fn=policy)},
    ]
    for run in runs:
        print(f"{run['name']}: {len(run['x'])} steps, RMS |n| {100 * np.sqrt((run['n'] ** 2).mean()):.2f} cm, max |n| "
              f"{100 * np.abs(run['n']).max():.1f} cm, smallest margin {100 * run['margin'].min():+.2f} cm, "
              f"end {run['end_reason']}", flush=True)

    readout_xy = args.readout_xy
    if readout_xy is None:
        bend = reference.sample(np.linspace(4.0, 9.5, 20))
        readout_xy = (bend["x"].min() - 3.0, bend["y"].max() - 0.3)  # off the track, left of the S-bend
    rclpy.init(args=ros_args)
    node = Replay(runs, reference, geometry["length"], geometry["width"], dynamics["lf"], dynamics["lr"],
                  cfg.track_margin, args.speed, readout_xy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
