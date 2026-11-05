# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Stage parameters for the Diff-WMPC OCP 

Every MPC call needs, per car: x0 (its state) and the 35-row table of reference values the OCP's
stage parameters expect (STAGE_REF_NAMES), shape (B, N+1, 1) each. This module builds both from the
car's arc length s0 (the plant's projection, info["s"]) and the reference.

Stage k sits at s_k, with s_0 = s0 and s_{k+1} = s_k + v_ref(s_k)·dt (dt = the MPC's interval), i.e.
the reference indexed by arc length; s_k is not wrapped (it continues past the start line). Each row
of psi_ref is put on x0's yaw branch along the whole horizon (align_yaw); x0 itself is unchanged (the
controller keeps it continuous in time, see MpcController.step).
"""

import numpy as np

from diff_weight_mpc.mpc.model import parameter_values
from diff_weight_mpc.mpc.ocp import STAGE_REF_NAMES
from plannertrack_py.utils import angle_diff
from plannertrack_py.plant import PSI


def stage_arc_lengths(reference, s0, n_horizon_steps, dt):
    """(B, N+1) arc lengths s_k of the stages for B cars at s0 (B,): s_{k+1} = s_k + v_ref(s_k)·dt.
    One vectorized step per stage for all cars (ReferenceTrack.stage_references does one car)."""
    s0 = np.atleast_1d(np.asarray(s0, dtype=float))
    S = np.empty((len(s0), n_horizon_steps + 1))
    S[:, 0] = s0
    for k in range(n_horizon_steps):
        # Assuming the car wont drift a lot from reference line.
        S[:, k + 1] = S[:, k] + reference.sample(S[:, k])["vx"] * dt 
    return S


# def align_yaw(states, psi_ref0):
#     """Copy of ``states`` (B, NX) with each yaw shifted by a multiple of 2π to the branch closest to
#     psi_ref0 (B,): psi0 <- psi_ref0 + wrap(psi0 - psi_ref0). Same direction, consistent numbers."""
#     x0 = np.array(states, dtype=float, copy=True)
#     x0[:, PSI] = psi_ref0 + angle_diff(x0[:, PSI], psi_ref0)
#     return x0

def align_yaw(x0_yaw, psi_ref):
    """psi_ref (B, N+1) on x0_yaw's branch along the WHOLE horizon: stage 0 within π of x0_yaw (B,), every
    next stage within π of the previous one (consecutive changes taken as the shortest angle). Robust to
    2π jumps anywhere in psi_ref. x0 unchanged. Assumes the reference turns < π between two stages
    (here ≤ 0.21 rad per 0.075 s)."""
    start = x0_yaw + angle_diff(psi_ref[:, 0], x0_yaw)
    steps = angle_diff(psi_ref[:, 1:], psi_ref[:, :-1])
    return np.concatenate([start[:, None], start[:, None] + np.cumsum(steps, axis=1)], axis=1)


def stage_parameters(reference, states, s0, n_horizon_steps, dt, vehicle_width, track_margin=0.0):
    """x0 and the OCP's stage parameters for B cars.

    Args:
        reference: ReferenceTrack (shared by all cars).
        states: (B, NX) or (NX,) plant states.
        s0: (B,) or scalar arc lengths of the cars (Plant: info["s"], or info["s0"] after reset).
        n_horizon_steps: N (OcpConfig.N); dt: the MPC interval (OcpConfig.dt).
        vehicle_width: track limits are the bounds minus half of it (= Plant.track_limits).
        track_margin: the MPC's limits are tightened by this much on each side [m] (OcpConfig.track_margin),
            but never past the reference line itself (n = 0 always stays inside them).

    Returns:
        x0 (B, NX) (a copy of ``states``), and {name: (B, N+1, 1)} for STAGE_REF_NAMES, with psi_ref
        on x0's yaw branch along each row, ready for
        ``diff_mpc(torch.tensor(x0), params={**params, ...})``.
    """
    states = np.atleast_2d(states)
    S = stage_arc_lengths(reference, s0, n_horizon_steps, dt)
    if S.shape[0] != states.shape[0]:
        raise ValueError(f"{states.shape[0]} states for {S.shape[0]} arc lengths")
    r = reference.sample(S)  # every field (B, N+1); psi continuous, laps handled
    r["psi"] = align_yaw(states[:, PSI], r["psi"])
    half = 0.5 * vehicle_width
    values = {
        "x_ref": r["x"], "y_ref": r["y"], "psi_ref": r["psi"], "v_ref": r["vx"], "alat_ref": r["a_lat"],
        "n_left": np.maximum(r["w_left"] - half - track_margin, 0.0),
        "n_right": np.minimum(-(r["w_right"] - half - track_margin), 0.0),
    }
    params = {name: values[name][..., None] for name in STAGE_REF_NAMES}
    return np.array(states, dtype=float, copy=True), params


def vehicle_parameters(vehicle_params, n_horizon_steps):
    """{"vehicle": (B, N+1, 5)} from a list of B VehicleParams (e.g. each car's true parameters for
    the oracle); omit it to use the nominal default."""
    p = np.stack([parameter_values(vp) for vp in vehicle_params])  # (B, 5)
    return {"vehicle": np.repeat(p[:, None, :], n_horizon_steps + 1, axis=1)}


