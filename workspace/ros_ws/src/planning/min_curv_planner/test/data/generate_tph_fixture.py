# Generates the tph_*.csv fixtures in this folder (see README.md). Run:
#   python3 -I generate_tph_fixture.py <dir containing trajectory_planning_helpers 0.76> \
#       ../../../../scenarios/ground_vehicle_racing/map/global_waypoints.json .
# Reference outputs of TUM trajectory_planning_helpers 0.76 for the racing
# scenario centerline: preprocessing, one QP, IQP, final raceline, speed profile.
import sys, json, io, contextlib, math
import numpy as np
sys.path.insert(0, sys.argv[1]); out = sys.argv[3]
import trajectory_planning_helpers as tph
from scipy import interpolate, spatial
# tph 0.76 vs scipy >= 1.9: splev returns a (2, 1) array, euclidean() now needs 1-D.
# Same math, flattened.
def _dist_to_p(t_glob, path, p):
    return spatial.distance.euclidean(p, np.ravel(interpolate.splev(t_glob, path)))
tph.spline_approximation.dist_to_p = _dist_to_p

# --- parameters (must match the C++ test) ---
mu, g, margin = 1.0489, 9.81, 0.85
ay_max = margin * mu * g
ax_tires, ax_machines, v_max = 8.0, 5.0, 10.0
kappa_bound = math.tan(0.4189) / 0.34
w_veh = 0.31 + 2 * 0.10
stepsize_prep, stepsize_reg, stepsize_interp = 0.1, 0.3, 0.2
iters_min, curv_err = 3, 0.01

d = json.load(open(sys.argv[2]))['centerline_waypoints']['wpnts']
center = np.array([[w['x_m'], w['y_m'], w['d_right'], w['d_left']] for w in d])
if np.allclose(center[0, :2], center[-1, :2]):
    center = center[:-1]

reftrack = tph.spline_approximation.spline_approximation(track=center, k_reg=3, s_reg=10,
    stepsize_prep=stepsize_prep, stepsize_reg=stepsize_reg)
_, _, A, normvec = tph.calc_splines.calc_splines(path=np.vstack((reftrack[:, :2], reftrack[0, :2])))
np.savetxt(out + '/tph_reftrack.csv', reftrack, delimiter=',', fmt='%.17g',
           header='x_m,y_m,w_tr_right_m,w_tr_left_m (tph spline_approximation of the racing centerline)')

# one QP solve (tph objective), first IQP iteration
alpha1, err1 = tph.opt_min_curv.opt_min_curv(reftrack=reftrack.copy(), normvectors=normvec, A=A,
                                            kappa_bound=kappa_bound, w_veh=w_veh)
np.savetxt(out + '/tph_qp_alpha.csv', alpha1, fmt='%.17g',
           header='alpha of one opt_min_curv solve on tph_reftrack.csv; curv_error_max = %.17g' % err1)

# IQP: tph 0.76 iqp_handler, verbatim except (1) the corrected objective (H doubled,
# tph's own QP builder otherwise) and (2) an iteration cap. The original objective does
# not converge on this track (error oscillates 0.02-0.07 rad/m, tph has no cap).
import quadprog
_orig_solve = quadprog.solve_qp
quadprog.solve_qp = lambda G, a, C, b, meq: _orig_solve(2 * G, a, C, b, meq)
ref_iqp = reftrack.copy(); nv_iqp = normvec; A_iqp = A
for iters in range(1, 21):
    alpha, err = tph.opt_min_curv.opt_min_curv(ref_iqp, nv_iqp, A_iqp, kappa_bound, w_veh)
    if iters < iters_min:
        alpha *= iters / iters_min
    if iters >= iters_min and err <= curv_err:
        break
    rl_, _, _, _, inds_, tv_ = tph.create_raceline.create_raceline(ref_iqp[:, :2], nv_iqp, alpha, stepsize_reg)[:6]
    ref_iqp[:, 2] -= alpha; ref_iqp[:, 3] += alpha
    ws_ = tph.interp_track_widths.interp_track_widths(ref_iqp[:, 2:], inds_, tv_, incl_last_point=False)
    ref_iqp = np.column_stack((rl_, ws_))
    _, _, A_iqp, nv_iqp = tph.calc_splines.calc_splines(np.vstack((ref_iqp[:, :2], ref_iqp[0, :2])), use_dist_scaling=False)
quadprog.solve_qp = _orig_solve
assert err <= curv_err, "IQP did not converge"
widths_rl = np.column_stack((ref_iqp[:, 2] - alpha, ref_iqp[:, 3] + alpha))

rl, A_rl, cx, cy, inds, tvals, s_pts, spl_len, el_cl = tph.create_raceline.create_raceline(
    refline=ref_iqp[:, :2], normvectors=nv_iqp, alpha=alpha, stepsize_interp=stepsize_interp)
psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(cx, cy, inds, tvals)
w_int = tph.interp_track_widths.interp_track_widths(widths_rl, inds, tvals, incl_last_point=False)
ggv = np.array([[0.0, ax_tires, ay_max], [20.0, ax_tires, ay_max]])
axm = np.array([[0.0, ax_machines], [20.0, ax_machines]])
vx = tph.calc_vel_profile.calc_vel_profile(ggv=ggv, ax_max_machines=axm, v_max=v_max, kappa=kappa,
    el_lengths=el_cl, closed=True, drag_coeff=0.0, m_veh=3.47, dyn_model_exp=1.0)
ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=np.append(vx, vx[0]), el_lengths=el_cl)
t = tph.calc_t_profile.calc_t_profile(vx_profile=vx, ax_profile=ax, el_lengths=el_cl)
psi_ours = np.arctan2(np.sin(psi + np.pi / 2), np.cos(psi + np.pi / 2))  # tph north=0 -> ours east=0
np.savetxt(out + '/tph_raceline.csv', np.column_stack((s_pts, rl, psi_ours, kappa, vx, ax, w_int)), delimiter=',',
           fmt='%.17g', header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_right_m,w_left_m (psi: 0 along +x; corrected objective); '
           'iqp_iterations = %d; lap_time_s = %.17g; length_m = %.17g' % (iters, t[-1], float(np.sum(spl_len))))
print('reftrack', reftrack.shape, 'qp err', err1, 'iqp iters', iters, 'raceline', rl.shape, 'lap', t[-1])
