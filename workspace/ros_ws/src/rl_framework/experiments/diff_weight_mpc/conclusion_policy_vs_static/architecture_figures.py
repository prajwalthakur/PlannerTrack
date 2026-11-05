#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Draws the three figures of architecture.md into this folder.

Run from the workspace root after `source install/setup.bash`:

    python3 src/rl_framework/data/conclusion_policy_vs_static/architecture_figures.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  
import numpy as np  
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch  

from plannertrack_py import ReferenceTrack, compute_raceline  

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
INK, MUTED, RULE = "#1f1f1d", "#52514e", "#c9c8c1"
BLUE, TEAL, ORANGE, PURPLE = "#2a78d6", "#1baf7a", "#eb6834", "#6250d6"
TINT_POLICY, TINT_MPC, TINT_PLANT, TINT_LOSS = "#fdeee7", "#e8f1fb", "#f1f0ea", "#efedfb"
plt.rcParams.update({"font.size": 11, "mathtext.fontset": "dejavusans", "text.color": INK})


def canvas(width, height, xlim, ylim):
    fig, ax = plt.subplots(figsize=(width, height))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.axis("off")
    fig.subplots_adjust(0, 0, 1, 1)
    return fig, ax


def box(ax, x0, y0, x1, y1, title, body=None, fill="white", edge=INK, title_size=12, body_size=10):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=0.8", fc=fill, ec=edge,
                                lw=1.4, zorder=3))
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    if body is None:
        ax.text(cx, cy, title, ha="center", va="center", fontsize=title_size, fontweight="bold", zorder=4)
        return
    lines = body.count("\n") + 1
    ax.text(cx, cy + 0.72 * lines, title, ha="center", va="center", fontsize=title_size, fontweight="bold", zorder=4)
    ax.text(cx, cy - 0.85, body, ha="center", va="center", fontsize=body_size, color=MUTED, zorder=4, linespacing=1.35)


def arrow(ax, points, color=INK, dashed=False, lw=1.6):
    """A polyline with an arrow head at its last point."""
    style = (0, (5, 3)) if dashed else "solid"
    for a, b in zip(points[:-2], points[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, ls=style, zorder=2, solid_capstyle="round")
    ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>", mutation_scale=15, color=color, lw=lw,
                                 ls=style, zorder=2, shrinkA=0, shrinkB=0))


def label(ax, x, y, value, color=MUTED, size=10, ha="center", va="center"):
    ax.text(x, y, value, ha=ha, va=va, fontsize=size, color=color, zorder=5,
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none"))


def training_loop():
    fig, ax = canvas(14, 7.0, (0, 112), (0, 57))
    top, bottom = 43, 33
    box(ax, 2, bottom, 20, top, "Reference raceline", "speed, curvature,\ntrack bounds along s")
    box(ax, 26, bottom, 44, top, r"Observation $o_k$", "5 speeds + 5 curvatures\nover the next 2.55 s")
    box(ax, 50, bottom, 66, top, r"Policy $\pi_\phi$", "10 → 128 → 128 → 6\noutput > 0 (softplus)", TINT_POLICY, ORANGE)
    box(ax, 74, bottom, 92, top, "Diff-MPC", "acados SQP-RTI + leap-c\n34 nodes × 0.075 s", TINT_MPC, BLUE)
    box(ax, 98, bottom, 110, top, "Plant", "single-track\nsim, 50 Hz", TINT_PLANT, MUTED)
    mid = 0.5 * (top + bottom)

    # forward pass, every control step
    arrow(ax, [(20, mid), (26, mid)])
    arrow(ax, [(44, mid), (50, mid)])
    arrow(ax, [(66, mid + 1.6), (74, mid + 1.6)])
    label(ax, 70, mid + 3.5, r"$\theta_k$", INK, 12)
    arrow(ax, [(92, mid), (98, mid)])
    label(ax, 95, mid + 2, r"$u^\star_0$", INK, 12)
    arrow(ax, [(11, top), (11, 49), (78, 49), (78, top)], MUTED)
    label(ax, 44, 49, "stage references", MUTED)
    arrow(ax, [(104, top), (104, 54), (6, 54), (6, top)])
    arrow(ax, [(84, 54), (84, top)])
    label(ax, 44, 54, r"next state $x_{k+1}$ and track position $s_{k+1}$", INK)

    # plan -> loss
    arrow(ax, [(80, bottom), (80, 26)])
    label(ax, 80, 29.5, r"plan $z^\star$", INK, 11)
    box(ax, 70, 16, 110, 26, "Task loss at node 1",
        r"$L_k = \alpha\, n_1^2 + \beta\,(v_1 - v_{\mathrm{ref},1})^2 + \gamma\, P_j(j_1) + \delta\, P_\omega(\omega_1)$",
        TINT_LOSS, PURPLE, body_size=11.5)

    # backward pass
    arrow(ax, [(87, 26), (87, bottom)], PURPLE, dashed=True)
    label(ax, 89, 29.5, r"$\partial L_k / \partial z^\star$", PURPLE, 11.5, ha="left")
    arrow(ax, [(74, mid - 1.6), (66, mid - 1.6)], PURPLE, dashed=True)
    label(ax, 70, 34.6, r"$\partial z^\star / \partial\theta$", PURPLE, 11)
    arrow(ax, [(60, bottom), (60, 26)], PURPLE, dashed=True)
    label(ax, 60, 29.5, r"$\nabla_\phi L_k$", PURPLE, 11.5)
    box(ax, 26, 16, 64, 26, "Accumulate gradients", "summed over the lap (one episode)", TINT_LOSS, PURPLE, body_size=10)

    box(ax, 2, 16, 20, 26, "Adam update", "at lap end: mean,\nclip ±0.1, step", TINT_LOSS, PURPLE, body_size=10)
    arrow(ax, [(26, 21), (20, 21)], PURPLE, dashed=True)
    arrow(ax, [(11, 26), (11, 30), (54, 30), (54, bottom)], PURPLE, dashed=True)
    label(ax, 30, 30, r"new policy parameters $\phi$", PURPLE)

    # no gradient through the plant
    ax.text(104, 28.5, "no gradient\nthrough the plant", ha="center", va="center", fontsize=10, color=MUTED,
            linespacing=1.3, zorder=5)

    # legend
    y = 9.5
    ax.plot([3, 11], [y, y], color=INK, lw=1.6)
    ax.text(12.5, y, "Forward pass: runs at every control step (50 Hz)", va="center", fontsize=11)
    ax.plot([3, 11], [y - 4, y - 4], color=PURPLE, lw=1.6, ls=(0, (5, 3)))
    ax.text(12.5, y - 4, "Backward pass: a gradient at every control step, one parameter update per lap", va="center",
            fontsize=11)
    fig.savefig(HERE / "architecture_training_loop.png", dpi=150, facecolor="white")
    plt.close(fig)


def policy_network():
    params = SRC / "scenarios" / "ground_vehicle_racing_rl" / "params"
    ref = ReferenceTrack(compute_raceline(params / "agents.yaml", "agent_1", params / "min_curv_planner.yaml"))
    fig = plt.figure(figsize=(14, 7.2))

    # the track ahead
    ax = fig.add_axes((0.005, 0.03, 0.235, 0.86))
    s0 = 3.42
    span = np.linspace(s0 - 1.0, s0 + 13.5, 300)
    r = ref.sample(span)
    normal = np.c_[-np.sin(r["psi"]), np.cos(r["psi"])]
    xy = np.c_[r["x"], r["y"]]
    for edge in (xy + normal * r["w_left"][:, None], xy - normal * r["w_right"][:, None]):
        ax.plot(*edge.T, color=INK, lw=1.4)
    ax.plot(*xy.T, color=MUTED, lw=1, ls=(0, (4, 3)))
    speeds, curvatures = ref.policy_lookahead(s0)
    times = (np.arange(5) + 1) * 2.55 / 5
    stages = ref.stage_references(s0, 255, 0.01)
    at = np.interp(times, stages["t"], stages["s"])
    p = ref.sample(at)
    ax.plot(p["x"], p["y"], "o", ms=9, color=BLUE, mec="white", mew=1.5, zorder=4)
    for i, (x, y) in enumerate(zip(p["x"], p["y"])):
        ax.annotate(f"{times[i]:.2f} s", (x, y), xytext=(10, 0), textcoords="offset points", fontsize=9.5, va="center",
                    bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none"))
    start = ref.sample(s0)
    ax.plot(start["x"], start["y"], "s", ms=11, color=ORANGE, mec="white", mew=1.5, zorder=4)
    ax.annotate("car", (start["x"], start["y"]), xytext=(11, 0), textcoords="offset points", fontsize=10, va="center",
                bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none"))
    ax.set_aspect("equal")
    ax.axis("off")

    fig.text(0.012, 0.955, "Track ahead", fontsize=13, fontweight="bold")
    fig.text(0.012, 0.915, "Real track section; dots show where\nthe 5 samples fall in time", fontsize=10, color=MUTED,
             va="center", linespacing=1.3)

    # the network
    ax = fig.add_axes((0.245, 0.0, 0.755, 1.0))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 72)
    ax.axis("off")
    r_node = 1.05
    center = 37
    inputs_v = [center + 3.0 * (9 / 2 - i) for i in range(5)]
    inputs_k = [center + 3.0 * (9 / 2 - i) for i in range(5, 10)]
    hidden = [center + 3.6 * (3 - i) for i in range(7)]
    outputs = [center + 5.0 * (5 / 2 - i) for i in range(6)]
    x_in, x_h1, x_h2, x_z, x_theta = 18, 36, 54, 72, 80

    def connect(xa, ya, xb, yb):
        for a in ya:
            for b in yb:
                ax.plot([xa, xb], [a, b], color=RULE, lw=0.45, zorder=1)

    def nodes(x, ys, color, fill="white"):
        for y in ys:
            ax.add_patch(Circle((x, y), r_node, fc=fill, ec=color, lw=1.5, zorder=3))

    all_in = inputs_v + inputs_k
    visible_hidden = [y for i, y in enumerate(hidden) if i != 3]
    connect(x_in, all_in, x_h1, visible_hidden)
    connect(x_h1, visible_hidden, x_h2, visible_hidden)
    connect(x_h2, visible_hidden, x_z, outputs)
    nodes(x_in, inputs_v, BLUE, "#e8f1fb")
    nodes(x_in, inputs_k, TEAL, "#e6f6f0")
    nodes(x_h1, visible_hidden, MUTED)
    nodes(x_h2, visible_hidden, MUTED)
    nodes(x_z, outputs, ORANGE, TINT_POLICY)
    for x in (x_h1, x_h2):
        ax.text(x, hidden[3], "⋮", ha="center", va="center", fontsize=17, color=MUTED, zorder=4,
                bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none"))

    # input labels
    ax.text(x_in - 2.2, np.mean(inputs_v), "5 reference speeds\n÷ 8.07 m/s (lap max)", ha="right", va="center",
            fontsize=10.5, linespacing=1.35)
    ax.text(x_in - 2.2, np.mean(inputs_k), "5 reference curvatures\n÷ 0.967 1/m (lap max)", ha="right", va="center",
            fontsize=10.5, linespacing=1.35)

    # output labels
    names = [(r"$q_n$", "lateral error"), (r"$q_\psi$", "heading error"), (r"$q_v$", "speed error"),
             (r"$q_{a_y}$", "lateral acceleration"), (r"$r_j$", "jerk input"), (r"$r_\omega$", "steering rate input")]
    for y, (symbol, meaning) in zip(outputs, names):
        arrow(ax, [(x_z + r_node, y), (x_theta - 0.3, y)], ORANGE, lw=1.3)
        ax.text(x_theta + 0.2, y, symbol, ha="left", va="center", fontsize=13)
        ax.text(x_theta + 4.2, y, meaning, ha="left", va="center", fontsize=10.5, color=MUTED)

    # headers
    head = 57.5
    for x, title, sub in ((x_in, "Input", "10 values"), (x_h1, "Hidden 1", "128 units"), (x_h2, "Hidden 2", "128 units"),
                          (x_z, "Output z", "6 values")):
        ax.text(x, head + 1.8, title, ha="center", fontsize=12, fontweight="bold")
        ax.text(x, head - 0.8, sub, ha="center", fontsize=10.5, color=MUTED)
    ax.text(x_theta + 0.2, head + 1.8, r"MPC cost weights $\theta_k$", ha="left", fontsize=12, fontweight="bold")
    ax.text(x_theta + 0.2, head - 0.8, r"$\theta = \mathrm{softplus}(z) > 0$", ha="left", fontsize=10.5, color=MUTED)

    # notes
    ax.text(33, 16.5, "Hidden activation: softplus in the paper,\nnone (linear layers) in the best run here",
            ha="center", va="center", fontsize=10.5, linespacing=1.4)
    ax.text(75, 16.5, "Start: last-layer weights 0, bias = softplus$^{-1}(\\theta_0)$,\n"
            "so every input gives exactly $\\theta_0$ at first", ha="center", va="center", fontsize=10.5, linespacing=1.4)
    fig.savefig(HERE / "architecture_policy.png", dpi=150, facecolor="white")
    plt.close(fig)
    return speeds, curvatures


def loss_node():
    fig, ax = canvas(13, 5.0, (0, 100), (0, 39))
    x = np.linspace(4, 96, 300)
    reference = 15.5 + 4.5 * np.sin((x - 4) / 92 * np.pi)
    plan = reference + 9.0 * np.exp(-(x - 7) / 20)
    ax.plot(x, reference, color=MUTED, lw=1.8, ls=(0, (6, 4)))
    ax.text(62, np.interp(62, x, reference) - 3.0, "reference raceline", ha="center", fontsize=10.5, color=MUTED)
    node_x = [7, 19, 31, 43, 55]
    node_y = np.interp(node_x, x, plan)
    ax.plot(x[x <= 62], plan[x <= 62], color=BLUE, lw=2)
    ax.plot(x[x > 62], plan[x > 62], color=BLUE, lw=2, ls=(0, (1.5, 3)))
    for nx, ny in zip(node_x[2:], node_y[2:]):
        ax.plot(nx, ny, "o", ms=8, color=BLUE, mec="white", mew=1.5, zorder=4)
    ax.plot(node_x[0], node_y[0], "s", ms=13, color=ORANGE, mec="white", mew=1.5, zorder=5)
    ax.plot(node_x[1], node_y[1], "o", ms=15, color=PURPLE, mec="white", mew=2, zorder=5)
    ax.text(node_x[0], node_y[0] + 3, "node 0\ncurrent state", ha="center", va="bottom", fontsize=10.5, linespacing=1.3)
    ax.text(node_x[1] + 1, node_y[1] + 3, "node 1, 0.075 s ahead:\nthe only node in the loss", ha="center", va="bottom",
            fontsize=11, fontweight="bold", color=INK, linespacing=1.3)
    ax.text(43, node_y[3] + 2.6, "nodes 2 … 34 are solved\nbut not scored", ha="center", va="bottom", fontsize=10.5,
            color=MUTED, linespacing=1.3)
    ax.text(80, np.interp(80, x, plan) + 2.2, "plan, 2.55 s", ha="center", fontsize=10.5, color=BLUE)

    # the lateral error at node 1
    ref_y = np.interp(node_x[1], x, reference)
    ax.add_patch(FancyArrowPatch((node_x[1], ref_y), (node_x[1], node_y[1] - 0.9), arrowstyle="<|-|>", mutation_scale=13,
                                 color=PURPLE, lw=1.6, zorder=4))
    ax.text(node_x[1] + 1.2, 0.5 * (ref_y + node_y[1]) - 0.4, r"$n_1$", fontsize=14, color=PURPLE, va="center")

    # the terms
    y = 6.2
    terms = [(r"$\alpha\, n_1^2$", "lateral error"), (r"$\beta\,(v_1 - v_{\mathrm{ref},1})^2$", "speed error"),
             (r"$\gamma\, P_j(j_1)$", "jerk penalty"), (r"$\delta\, P_\omega(\omega_1)$", "steering-rate penalty")]
    for i, (term, meaning) in enumerate(terms):
        cx = 14 + 24 * i
        ax.text(cx, y, term, ha="center", va="center", fontsize=14)
        ax.text(cx, y - 3.4, meaning, ha="center", va="center", fontsize=10.5, color=MUTED)
    ax.text(50, y + 3.9, "Terms read at node 1 of each plan (schematic, node spacing not to scale)", ha="center",
            fontsize=10.5, color=MUTED)
    fig.savefig(HERE / "architecture_loss_node.png", dpi=150, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    training_loop()
    speeds, curvatures = policy_network()
    print("look-ahead speeds", np.round(speeds, 2), "curvatures", np.round(curvatures, 2))
    loss_node()
