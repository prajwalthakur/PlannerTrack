/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

# Jerk-cost `duration` scaling — Step 1: differentiate via the chain rule

Context: EPSILON's Bezier-QP convention defines the *physical* curve as
`p(t) = duration · B(τ)`, where `τ = (t - t_lb)/duration ∈ [0,1]` is the
normalized segment parameter and `B(τ) = Σⱼ Bⱼ(τ)·xⱼ` is the plain
(unscaled) Bernstein-Bezier sum over the raw decision variables `xⱼ`. This
is the convention behind `spline_generator.cc`'s
`val = hessian(j,k) / pow(duration, 2*derivative_degree - 3)`.

## Setup

Since `τ` is a rescaling of `t` with `duration` held constant for the
segment:
```
τ = (t - t_lb) / duration   =>   dτ/dt = 1/duration   (a constant)
```

## Step 1 — differentiate `p(t)` three times w.r.t. real time `t`

Every time we differentiate `p(t) = duration · B(τ(t))` w.r.t. `t`, the
outer `duration` factor is a constant (unaffected), and the chain rule
pulls out one more factor of `dτ/dt = 1/duration` from the inner `B(τ(t))`:

```
p(t)     = duration · B(τ)

dp/dt    = duration · (dB/dτ) · (dτ/dt)
         = duration · (dB/dτ) · (1/duration)
         = dB/dτ                                    <- the durations cancel

d²p/dt²  = d/dt [ dB/dτ ]
         = (d²B/dτ²) · (dτ/dt)
         = (1/duration) · d²B/dτ²

d³p/dt³  = d/dt [ (1/duration) · d²B/dτ² ]
         = (1/duration) · (d³B/dτ³) · (dτ/dt)
         = (1/duration) · (d³B/dτ³) · (1/duration)
         = (1/duration²) · d³B/dτ³
```

So the result of Step 1:
```
d³p/dt³ = duration⁻² · d³B/dτ³
```

Note the pattern: each derivative order costs one more power of
`1/duration` — but the very first derivative "gives one back" because the
outer `duration` factor in `p(t) = duration·B(τ)` cancels against the first
`dτ/dt = 1/duration` picked up. That's why the running exponent goes
`0 -> 0 -> -1 -> -2` (position, velocity, acceleration, jerk) instead of
`0 -> -1 -> -2 -> -3`. This is the source of EPSILON's (initially
surprising) scaling convention: position scales as `duration⁺¹` overall
once you also account for the leading `duration` multiplying `B(τ)` itself,
velocity as `duration⁰`, acceleration as `duration⁻¹`, and (as derived
here) the pre-integration jerk expression as `duration⁻²`.

This `d³p/dt³ = duration⁻² · d³B/dτ³` result is the input to Step 2
(squaring) and Step 3 (integrating over `t`), which is where the final
`duration⁻³` factor in the code comes from.

## Step 2 — square it

The jerk cost is the integral of jerk *squared*, so square Step 1's result:
```
(d³p/dt³)² = (duration⁻² · d³B/dτ³)²
           = duration⁻⁴ · (d³B/dτ³)²
```

## Step 3 — integrate over real time `t`, converting to `τ`

We want `∫_{t_lb}^{t_ub} (d³p/dt³)² dt`. Since `t = t_lb + duration·τ`, the
differential converts as `dt = duration · dτ`, and the limits `t∈[t_lb,t_ub]`
become `τ∈[0,1]`. Substitute Step 2's result:
```
∫_{t_lb}^{t_ub} (d³p/dt³)² dt = ∫₀¹ duration⁻⁴ · (d³B/dτ³)² · duration dτ
                               = duration⁻⁴⁺¹ · ∫₀¹ (d³B/dτ³)² dτ
                               = duration⁻³ · ∫₀¹ (d³B/dτ³)² dτ
```
That `duration⁻³` is exactly the `pow(duration, 2*derivative_degree-3)`
denominator in the code (`derivative_degree=3` → `2·3-3=3`). Everything
else, `∫₀¹(d³B/dτ³)² dτ`, is a pure-`τ` quantity — no `duration` left in it
at all, which is what makes it possible to precompute it *once* as a fixed
constant matrix and reuse it for every segment (just rescaling by that
segment's own `duration⁻³` afterward).

## Step 4 — the remaining integral is exactly `xᵀHx`

`B(τ) = Σⱼ Bⱼ(τ)·xⱼ` is a linear combination of the control points, so its
third derivative is too:
```
d³B/dτ³ = Σⱼ Bⱼ'''(τ) · xⱼ
```
Squaring a linear combination gives a quadratic form — multiply the sum by
itself and expand:
```
(d³B/dτ³)² = ( Σⱼ Bⱼ'''(τ)xⱼ ) · ( Σₖ Bₖ'''(τ)xₖ )
           = Σⱼ Σₖ [ Bⱼ'''(τ) · Bₖ'''(τ) ] · xⱼxₖ
```
Now integrate over `τ∈[0,1]`. The `xⱼxₖ` factors are constants (don't
depend on `τ`), so they pull outside the integral, leaving only the basis
functions to actually integrate:
```
∫₀¹ (d³B/dτ³)² dτ = Σⱼ Σₖ [ ∫₀¹ Bⱼ'''(τ)·Bₖ'''(τ) dτ ] · xⱼxₖ
                   = Σⱼ Σₖ H(j,k) · xⱼxₖ
                   = xᵀ H x
```
where `H(j,k) = ∫₀¹ Bⱼ'''(τ)·Bₖ'''(τ) dτ` — exactly `hessian(j,k)` in the
code: a fixed 6×6 matrix (for degree 5), computed once, with no `duration`
dependence anywhere in it.

## Putting Steps 1-4 together

```
Jerk cost for one segment = duration⁻³ · xᵀHx
```
which is precisely what the code computes: `hessian(j,k) / pow(duration, 3)`
inserted into `Q` at the block belonging to that segment's control points
`x`, for every `(j,k)` pair.

# Reference-tracking (proximity) cost — derivation

This is the second cost term, built from `ref_stamps`/`ref_points` (the seed/
reference trajectory samples) into `P`/`c`, sitting alongside the jerk cost
`Q` derived above. It's what makes the fitted curve hug the reference path
instead of just being the smoothest curve that happens to fit the corridor.

## Setup: what the local variables mean

```cpp
decimal_t s = cubes[n].t_ub - cubes[n].t_lb;   // THIS segment's duration
decimal_t t = ref_stamps[i] - cubes[n].t_lb;   // time elapsed since this segment started
```
This local `s` is **segment duration** -- nothing to do with Frenet
longitudinal position `s` used elsewhere in our own project, pure naming
coincidence from EPSILON's generic, dimension-agnostic code. `τ = t/s`
(normalized `[0,1]` parameter) is never stored in a variable -- it's
recomputed inline as `t/s` everywhere below.

`n = std::min(num_segments - 1, n);` is a boundary clamp: the lookup loop
above (`for(n...) if(cubes[n].t_ub > ref_stamps[i]) break;`) can run off the
end without ever breaking if `ref_stamps[i]` lands exactly on the very last
cube's `t_ub` (the strict `>` never trips), leaving `n == num_segments`.
This clamps it back to the last valid segment index.

## Step 1 -- what does the curve actually predict at this sample's time?

Under EPSILON's convention (`p(t) = duration · B(τ)`), the predicted
position in dimension `d`, for segment `n`'s control points `x_{d,n,j}`, is:
```
p_d(t) = s · Σⱼ Bⱼ(τ) · x_{d,n,j},     Bⱼ(τ) = C(5,j)·τʲ·(1-τ)^(5-j)
```

## Step 2 -- the squared-error cost for this one sample

```
cost_i = (p_d(t_i) - ref_i)²  = p_d(t_i)² − 2·ref_i·p_d(t_i) + ref_i²
```
Drop `ref_i²` (constant, doesn't affect the optimizer) -- same
"expand-the-square, keep quadratic + linear" move as the jerk cost above.

## Step 3 -- substitute `p_d(t_i)` and expand

**Linear term:**
```
−2·ref_i·p_d(t_i) = −2·ref_i·s·Σⱼ Bⱼ(τᵢ)·xⱼ = Σⱼ [ −2·ref_i·s·Bⱼ(τᵢ) ]·xⱼ
```
So control point `xⱼ` gets coefficient `−2·ref_i·s·Bⱼ(τᵢ)`. Matching the
code:
```cpp
c[idx] += -2 * ref_points[i][d] * s * nchoosek(N_DEG, j) * pow(t/s, j) * pow(1-t/s, N_DEG-j);
```
`nchoosek(N_DEG,j)*pow(t/s,j)*pow(1-t/s,N_DEG-j)` is exactly
`C(5,j)·τʲ·(1-τ)^(5-j) = Bⱼ(τᵢ)` written out longhand. Matches the
derivation term-for-term.

**Quadratic term:**
```
p_d(t_i)² = [s·Σⱼ Bⱼ(τᵢ)xⱼ]² = s²·(Σⱼ Bⱼ(τᵢ)xⱼ)·(Σₖ Bₖ(τᵢ)xₖ) = s²·Σⱼ Σₖ [Bⱼ(τᵢ)·Bₖ(τᵢ)] xⱼxₖ
```
Coefficient of `xⱼxₖ` is `s²·Bⱼ(τᵢ)·Bₖ(τᵢ)`. Now the code:
```cpp
P.coeffRef(idx, idy) += s*s * nchoosek(N_DEG,j) * nchoosek(N_DEG,k) * pow(t/s, j+k) * pow(1-t/s, 2*N_DEG-j-k);
```
This is `Bⱼ(τ)·Bₖ(τ)` written a slightly different (but algebraically
identical) way -- instead of computing `Bⱼ(τ)` and `Bₖ(τ)` separately and
multiplying, it combines the exponents of the shared bases first:
```
Bⱼ(τ)·Bₖ(τ) = [C(5,j)τʲ(1-τ)^(5-j)] · [C(5,k)τᵏ(1-τ)^(5-k)]
            = C(5,j)C(5,k) · τ^(j+k) · (1-τ)^(5-j+5-k)
            = C(5,j)C(5,k) · τ^(j+k) · (1-τ)^(2·5-j-k)
```
-- exactly `pow(t/s, j+k) * pow(1-t/s, 2*N_DEG-j-k)`. Same value, just two
`pow()` calls instead of four -- a minor efficiency trick, not a different
formula. So the code is exactly `s²·Bⱼ(τᵢ)·Bₖ(τᵢ)`, matching the derivation.

## Step 4 -- indices

`idx`/`idy` use the same `d,n` (this loop iteration's fixed dimension/
segment) with only `j`/`k` varying -- same flattening scheme, same "one
block per (segment,dimension)" structure as `Q`'s jerk-cost fill loop
above.

## Net result

Every reference sample contributes one `−2·ref_i·s·Bⱼ(τᵢ)` term to `c` and
one `s²·Bⱼ(τᵢ)Bₖ(τᵢ)` term to `P`, for every `j,k` pair in its owning
segment -- summed (via `+=`) across however many samples land in that
segment. That accumulated `(P, c)` pair is exactly the quadratic-form
expansion of `Σᵢ (p_d(t_i) − ref_i)²` restricted to that one segment's
control points.

## Final assembly (combining with the jerk cost)

```cpp
P = P * weight_proximity;
c = c * weight_proximity;
Q = 2 * (Q + P);  // 0.5 * x' * Q * x
```
`P` and `c` are scaled together by `weight_proximity` (they came from
expanding the same squared-error term, so they must be scaled together --
scaling only one would mean solving a different, inconsistent cost).

The `× 2` on the last line matters and is easy to miss: the QP solver's
convention is `min 0.5·x'Qx + c'x` -- a built-in `0.5` in front of the
quadratic term. But neither derived cost (jerk: `duration⁻³·xᵀHx`, no
leading 0.5; tracking: `(B(τ)x-ref)²`, no leading 0.5) has that factor
baked in. Handing the solver `Q+P` directly would make it enforce
`0.5·(Q+P)` -- half the intended cost. Pre-multiplying by 2 cancels this:
`0.5·[2·(Q+P)] = (Q+P)`, the actual derived cost, unchanged. `c` needs no
such correction since the solver's linear term is just `c'x`, no `0.5` in
front of it.

**Relevant for our own port**: `mpl_qp_interface::OSQPInterface` uses the
same standard `0.5x'Px+q'x` OSQP convention, so this same `×2` step must be
replicated when assembling our own combined cost matrix.

# Stage II -- Equality constraints (continuity + start/end boundary conditions)

Two kinds of equality rows go into `A x = b`: **continuity** rows (link
adjacent segments to each other, right-hand side always 0) and
**start/end** rows (pin the very first/last segment to real physical target
values, right-hand side is an actual number).

## Setup: how many rows, and matrix sizing

```cpp
int num_continuity = 3;                 // continuity up to acceleration (see caveat below)
int num_connections = num_segments - 1; // one connection per adjacent pair of segments
int num_continuity_constraints = N_DIM * num_connections * num_continuity;
int num_start_eq_constraints = start_constraints.size() * N_DIM;  // 3 orders (pos,vel,acc) * 2 dims = 6
int num_end_eq_constraints   = end_constraints.size()   * N_DIM;  // 2 orders (pos,vel)     * 2 dims = 4
int total_num_eq_constraints = num_continuity_constraints + num_start_eq_constraints + num_end_eq_constraints;
```
`A` is `(total_num_eq_constraints x total_num_vals)`, `b` is zero-initialized
(`b.setZero()`) and only the start/end rows ever write a nonzero value into
it -- continuity rows leave their `b` entry at 0, since they encode
"segment n's value = segment n+1's value" (difference = 0), not a target.

## Comment vs. code mismatch #1: "continuity up to jerk"

`for (int c = 0; c < num_continuity; c++)` with `num_continuity = 3` only
ever produces `c = 0, 1, 2` (position, velocity, acceleration) -- the loop
condition is false before `c` can reach `3`, so the `else if (c == 3)`
jerk-continuity branch later in the function is real, correctly-written
code that is simply **unreachable**. Position is the 0th derivative,
velocity the 1st, acceleration the 2nd, jerk the 3rd -- `num_continuity=3`
gives exactly 3 derivative orders, `0..2`, i.e. continuity up to
**acceleration**, not jerk. The `// continuity up to jerk` comment appears
to be a miscount (reading "3" as "jerk is the 3rd derivative" rather than
"the loop only produces 3 values, 0 through 2").

This is also the mathematically correct choice, not just what the code
happens to do: a degree-5 Bezier segment has exactly 6 control points = 6
degrees of freedom. C2 continuity (position+velocity+acceleration) at
*both* ends of an interior segment is `3 conditions x 2 ends = 6`
constraints -- already exactly saturating all 6 DOF. Adding jerk continuity
would need `4 x 2 = 8` constraints on 6 unknowns -- generally infeasible,
and would leave zero freedom for the cost function or corridor bounds to
do anything for any interior segment.

## Comment vs. code mismatch #2: "exclude s position constraint"

```cpp
int num_end_eq_constraints = end_constraints.size() * N_DIM;  // exclude s position constraint
```
This computes `2 * 2 = 4` end rows (2 derivative orders x 2 dims), no
exclusion actually applied. The comment refers to a design idea visible a
bit further down in the same function, also inactive:
```cpp
for (int d = 0; d < N_DIM; d++) {
  // if (j == 0 && d == 0) continue;   // <- would have skipped the final s-position row
```
`j==0` = position order, `d==0` = the `s` (longitudinal) dimension -- the
idea being that the corridor's final `s` cutoff is somewhat
arbitrary/grid-quantized, so pinning the trajectory to hit it exactly may
be an unnecessarily rigid constraint, unlike final lateral position or
either velocity. As shipped, this `continue` is commented out, so both
dimensions' position rows are included -- a second case (after the jerk
one above) of a comment describing an intent the code doesn't actually
implement.

## `A.reserve(2 * num_order)` -- a conservative, not exact, bound

Unlike `Q`'s and `P`'s `.reserve()` calls (which matched their fill loops'
*exact* nonzero-per-row counts), this one over-allocates. Counting nonzeros
per row directly from the fill loops:
- continuity row, order `c`: `2*(c+1)` (c+1 points from each side) ->
  `c=0`: 2, `c=1`: 4, `c=2`: 6
- start row, order `j`: `j+1` -> 1, 2, 3
- end row, order `j`: `j+1` -> 1, 2

True max across every row this function builds: **6** (the
acceleration-continuity row), which equals `num_order` (6, for degree 5).
`A.reserve(2*num_order=12)` reserves double that -- reads like a quick,
safe mental shortcut ("touches at most 2 segments, each up to `num_order`
values") rather than the tighter per-row-type count. Harmless (just a
little unused memory) since under-reserving, not over-reserving, is the
thing `.reserve()` exists to avoid.

## Continuity row construction -- the general pattern

Each row enforces "segment `n`'s order-`c` derivative at its own end =
segment `n+1`'s order-`c` derivative at its own start", built as `[+left
terms] + [-right terms] = 0` (right side negated to turn `left=right` into
`left-right=0`). The formulas reuse the standard Bezier finite-difference
property (k-th derivative of a degree-N curve = degree-(N-k) curve whose
control points are k-th differences of the original ones, scaled by
`N!/(N-k)!`):
- `c=0` (position): `P_N` (left) vs `P'_0` (right) -- a curve passes
  exactly through its first/last control point at `τ=0`/`τ=1`.
- `c=1` (velocity): `N·(P_N-P_{N-1})` (left) vs `N·(P'_1-P'_0)` (right).
- `c=2` (acceleration): `N(N-1)·(P_{N-2}-2P_{N-1}+P_N)` (left) vs the
  mirrored forward-difference (right) -- derived by applying the
  first-difference step twice (a "difference of differences"), landing on
  the standard `[1,-2,1]` second-difference stencil. **The code's `val`s
  are the bare `1,-2,1` coefficients, missing the `N(N-1)` factor** -- and
  that's fine specifically because this is a homogeneous (`=0`) row: since
  `N(N-1)` multiplies *both* sides identically, dividing the whole equation
  by it changes nothing. (Contrast: the start/end constraint rows below
  *do* keep the full `N(N-1)`, because their right-hand side is a real
  physical target value, not zero, so the scale can't be dropped there.)
  `scale_l/scale_r = pow(duration, 1-c)` is EPSILON's own duration
  convention (acceleration -> `duration⁻¹`), applied per-side using that
  side's own segment duration.

## Bug found while porting: `c <= num_continuity` in our own `bezier_curve.cpp`

Our port (`planning/ssc_planner/src/bezier_traj_gen/bezier_curve.cpp`) has:
```cpp
int num_continuity = 3;                    // still 3
for (int c = 0; c <= num_continuity; c++)  // <= instead of EPSILON's <
```
This produces `c = 0,1,2,3` -- reaching the jerk branch, which (per the
degrees-of-freedom argument above) shouldn't be enabled for a degree-5
segment in the first place. Worse, nothing else was updated to account for
a 4th row per connection, so it actively **corrupts indices**, not just
"adds an extra unwanted row":
```
idx(c=3) = d*num_connections*3 + n*3 + 3
         = d*num_connections*3 + (n+1)*3 + 0
         = idx(c=0) for connection (n+1), same dimension d
```
Connection `n`'s jerk-continuity row silently lands on the exact same row
index as connection `(n+1)`'s position-continuity row. Since `n` is the
outer loop, connection `n`'s `c=3` write happens first; connection `n+1`'s
`c=0` write (different columns, same row) follows -- no crash (Eigen's
`.insert()` only errors on a literal duplicate `(row,col)` pair, and these
have different columns), just a row that silently encodes a meaningless mix
of two unrelated constraints. This misalignment cascades through every
subsequent row, and the very last connection's `c=3` row lands exactly on
the index where the start-constraint block begins, corrupting that
boundary too. `A`'s allocated size (`num_continuity_constraints`, still
computed from `num_continuity=3`) was also never sized for a real 4th row.

Likely fix: match EPSILON's original `c < num_continuity` (not `<=`),
consistent with the DOF argument for why C2-only continuity is the correct
choice here -- flagged for confirmation, not applied.

## Start constraints -- segment 0 pinned to ego's actual current state

```cpp
decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;  // segment 0's own duration
int n = 0;
for (int j = 0; j < num_order_constraint_start; j++) {  // j=0,1,2: pos,vel,acc
  scale = pow(duration, 1 - j);
  for (int d = 0; d < N_DIM; d++) {
    idx = num_continuity_constraints + d * num_order_constraint_start + j;
```
Rows sit right after all continuity rows. Note the `idx` formula groups by
`d` first even though the loop nests `j` outer/`d` inner -- every `(j,d)`
still gets a unique row (no collision, since both the loop bound and the
formula share the same `num_order_constraint_start`), just not in strict
loop-execution order -- a minor style quirk, not a bug (unlike the
continuity-block issue above).

- `j=0` (position): `idy` = segment 0's first control point (`P_0`).
  `val=1.0*scale`, `scale=duration¹`. Row: `duration·P_0 = target_position`.
- `j=1` (velocity): `-N·P_0 + N·P_1`, `scale=duration⁰=1`. Row:
  `N·(P_1-P_0) = target_velocity`.
- `j=2` (acceleration): `N(N-1)·P_0 - 2N(N-1)·P_1 + N(N-1)·P_2`,
  `scale=duration⁻¹`. Row: `N(N-1)·(P_0-2P_1+P_2) = target_acceleration`.
  Full `N(N-1)` factor kept this time -- `b[idx]` is a real value, so it
  can't be dropped (see the continuity-row note above).

## End constraints -- last segment pinned to the seed trajectory's final state

Mirrors the start block, but only `j=0,1` (position, velocity --
`end_constraints.size()==2`, **no acceleration constraint at the end,
deliberately left free** -- this is the consuming side of the same fact we
found earlier at `ssc_planner.cc`'s call site, where only 2 entries ever
get pushed into `end_constraints`):
```cpp
decimal_t duration = cubes[num_segments-1].t_ub - cubes[num_segments-1].t_lb;
int n = num_segments - 1;
int accu_eq_cons_idx = num_continuity_constraints + num_start_eq_constraints;
for (int j = 0; j < num_order_constraint_end; j++) {
  ...
  idx = accu_eq_cons_idx++;   // simple running counter, not a formula this time
```
- `j=0`: `idy` = last segment's *last* control point (`P_N`). Row:
  `duration·P_N = target_position`.
- `j=1`: `-N·P_{N-1} + N·P_N`. Row: `N·(P_N-P_{N-1}) = target_velocity`.

## Big picture

Continuity rows stitch every adjacent pair of segments together smoothly
(no gap in position/velocity/acceleration across cube boundaries).
Start/end rows anchor the *whole chain* to real values at both ends --
ego's actual current state at the start, the seed trajectory's final state
at the end. Together they fully pin the curve down except for whatever
freedom is left for the cost function (jerk-min + reference-tracking, Stage
I) and the corridor's inequality bounds (Stage III) to actually act on.

# Stage III -- Inequality constraints (corridor bounds)

This is where the corridor cube bounds actually get applied -- turning
"the curve must stay inside this box" into linear constraints via the
**convex-hull property**: a Bezier curve always lies entirely within the
convex hull of its own control points, so bounding *every individual
control point* inside the box is a sufficient (if slightly conservative --
the curve needn't touch every corner) way to guarantee the whole curve
stays inside it.

## Sizing

```cpp
for (int i = 0; i < num_segments; i++) {
  total_num_ineq += cubes[i].p_ub.size() * num_order;       // 2 * 6 = 12
  total_num_ineq += cubes[i].v_ub.size() * (num_order - 1); // 2 * 5 = 10
  total_num_ineq += cubes[i].a_ub.size() * (num_order - 2); // 2 * 4 = 8
}
```
`p_ub`/`v_ub`/`a_ub` are each `N_DIM`-length arrays (one bound per
dimension), so `.size()` here is just `N_DIM=2` -- that's why there's no
separate `x N_DIM` written explicitly, it's baked into iterating those
arrays' own size. Per segment: `12+10+8=30` rows.

`C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3))` -- unlike Stage
II's `A.reserve(2*num_order)` (a loose, generous bound), this one is
**exactly tight**: position rows need 1 nonzero, velocity rows need 2,
acceleration rows need 3 -- max is 3, and that's precisely what's reserved
for every row (slightly over-reserving the position/velocity rows, but no
more than that).

## Position bounds -- the direct case

```cpp
scale = pow(duration, 1 - 0);              // duration^1
for (int j = 0; j < num_order; j++) {
  idy = ...segment n's control point j...
  C.insert(idx, idy) = scale;
  lbd[idx] = cubes[n].p_lb[d];  ubd[idx] = cubes[n].p_ub[d];
}
```
One row **per control point** (6 of them): `duration*P_j` in
`[p_lb, p_ub]`. No differencing needed -- position is the 0th derivative,
so it's just the raw control point value, scaled into physical units
(EPSILON's `duration^1` convention). This is the convex-hull property
applied literally: bound every one of the segment's 6 control points
inside the cube's `(s,d)` box.

## Velocity bounds -- reusing the same first-difference formula from continuity/boundary rows

```cpp
scale = pow(duration, 1 - 1);              // duration^0 = 1
for (int j = 0; j < num_order - 1; j++) {  // 5 rows
  C.insert(idx, idy_j)   = -N_DEG * scale;
  C.insert(idx, idy_j+1) =  N_DEG * scale;
  lbd[idx]=v_lb; ubd[idx]=v_ub;
}
```
This is exactly the `N*(P_{j+1}-P_j)` formula already derived for velocity
continuity/boundary rows -- except now applied to **every consecutive
pair** `j,j+1` (5 pairs for 6 control points), not just the two endpoints.
Each row is one control point of the **hodograph** (the degree-4
first-derivative curve) -- and since that derivative curve is itself a
Bezier curve, it has its *own* convex-hull property: bounding each of
*its* control points keeps the velocity profile inside `[v_lb,v_ub]`.

## Acceleration bounds -- same idea, one derivative further

```cpp
scale = pow(duration, 1 - 2);              // duration^-1
for (int j = 0; j < num_order - 2; j++) {  // 4 rows
  N_DEG*(N_DEG-1)*P_j - 2*N_DEG*(N_DEG-1)*P_{j+1} + N_DEG*(N_DEG-1)*P_{j+2}  in [a_lb, a_ub]
}
```
Same `N(N-1)*(P_j-2P_{j+1}+P_{j+2})` second-difference formula from the
acceleration-continuity derivation, now applied to each of the 4 triples of
consecutive control points -- bounding every control point of the
second-derivative curve.

## The connecting thread

This is the *exact same three formulas* (raw value / first difference /
second difference) already derived for the continuity and start/end
boundary rows in Stage II -- just reused for a different purpose: there
they enforced an **equality** at one specific `τ` (a segment's very start
or end), here they enforce an **inequality** at *every* control point
along the whole segment. And unlike the continuity rows (where the
`N`/`N(N-1)` factor could be dropped because the row was homogeneous), here
it's kept in full -- `lbd`/`ubd` are real physical bounds, not zero, so the
scale matters, same reasoning as the start/end constraint rows.
