# SSC Planner

## Pass case

![Safe corridor](../../images/ssc_pass.gif)


Four Agents, other three agents are in rest condition. Ego agent is able to create a safe cooridor and trajectory through qp optimization. 

## Failure case

![Safe corridor](../../images/ssc_failure.png)

### Why it fails

Corridor construction seeds cube growth from a **single, pre-committed reference
trajectory** — a purely geometric route (lattice/graph planner) combined with a
constant-velocity timing profile, computed independently of any other agent's predicted
trajectory. When another agent's predicted occupancy intersects this seed's own
`(s, d, t)` coordinate, the very first cube grown at that point is infeasible by
construction. Because cube-chain growth is sequential and all-or-nothing — no
back-tracking once a link fails — the entire corridor, and consequently the downstream
Bezier QP, is abandoned. The result is honest and safety-correct (no unsafe trajectory
is ever published), but uninformative: the planner has no way to ask *"would a different
timing, or a different lane, avoid this conflict?"*

This is a direct consequence of reproducing only a single behavioral hypothesis. In the
original formulation (Ding et al., *"Safe Trajectory Generation for Complex Urban
Environments Using Spatio-Temporal Semantic Corridor,"* RA-L 2019), this planner sits
**downstream** of a multi-policy decision layer (MPDM-style): several candidate seed
trajectories — e.g. proceed, yield, overtake — are proposed upstream, each is
corridor-checked, and the best *feasible* one is selected. Corridor construction and QP
smoothing were never meant to resolve behavioral ambiguity on their own; they only
certify and refine a decision already made elsewhere. That decision layer is not
implemented in this reproduction.

### Future work

Reintroduce the missing decision layer: generate a small set of candidate seeds per
detected conflict (e.g. a decelerate-and-yield profile re-timed against the conflicting
agent's predicted trajectory, and/or a laterally-offset seed into an adjacent lane), run
the existing corridor + QP pipeline against each candidate (cheap to prune early via the
same `checkIfCubeIsFree` check already used today), and select the first/best feasible
result. No change to the corridor or QP formulation itself is required — only to what
feeds it.
