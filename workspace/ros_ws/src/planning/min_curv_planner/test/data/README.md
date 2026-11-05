# tph reference fixtures

Reference outputs of TUM `trajectory_planning_helpers` 0.76 ("tph", the Python implementation of
Heilmeier et al. that `min_curv_planner` ports), for the racing scenario centerline
(`scenarios/ground_vehicle_racing/map/global_waypoints.json`). Regenerate with
`generate_tph_fixture.py` (needs tph 0.76, quadprog, scipy).

| File | Content |
|---|---|
| `tph_reftrack.csv` | tph `spline_approximation` of the centerline (s_reg 10, 0.1 m / 0.3 m): the reference line fed to the optimizer |
| `tph_qp_alpha.csv` | α of **one** `opt_min_curv` solve on that reference, original tph objective (parity for `objective: tph_compat`) |
| `tph_raceline.csv` | full IQP + final raceline + speed profile. The IQP uses tph's functions with the **corrected** objective (H doubled): with the original objective the IQP does not converge on this track (linearization error oscillates 0.02–0.07 rad/m) and tph has no iteration cap. Its speed profile keeps tph's start/finish bug (no braking for the first corner at the end of the lap); `min_curv_planner` brakes there, so speeds differ in that zone only |

Parameters (must match the C++ tests): μ = 1.0489, friction margin 0.85, a_x tires 8 m/s²,
a_x machines 5 m/s², v_max 10 m/s, κ_bound = tan(0.4189)/0.34, w_veh = 0.51 m, steps
0.1 / 0.3 / 0.2 m, iters_min 3, curvature error 0.01 rad/m, no drag.

Note: tph 0.76 needs a one-line patch for scipy ≥ 1.9 in `spline_approximation.dist_to_p`
(applied inside the script).
