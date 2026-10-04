/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

# MPPI Interview Prep — Q&A Scripts

Working notes from a 2026-10-02 session. Captures three interview questions, each answered from a
specific primary source so the citation is always available if pushed. Two different derivation
routes are used on purpose for Q2/Q3 — **info-theoretic** (papers 2/3, the free-energy/Girsanov
route already taught in [lessons 0001–0002](lessons/0001-free-energy-and-the-optimal-distribution.html))
vs. **probabilistic-inference** (`papers/mppi_overview.pdf`, Honda's control-as-inference route,
§2.3.1–2.5) — because the two routes give structurally different *reasons* for the same conclusions,
and conflating them on a whiteboard is a tell that the material is memorized rather than understood.

---

## Q0: Why is MPPI a sampling-based method at all?

Source: `papers/mppi_overview.pdf`, §1 (Fig. 1 + footnote 1), §2.1 (end), §2.2, §2.5.

Two distinct reasons, not one:

1. **The OCP itself defeats classical solvers.** §2.1: when $J$ and/or $f$ are "highly nonlinear,
   nonconvex, or nondifferentiable, or when stochastic dynamics are considered," classical
   gradient/Hessian-based MPC solvers suffer sensitivity to initialization, poor conditioning,
   convergence to local minima — and are inapplicable at all if nondifferentiable. Stochastic
   dynamics compound this by making trajectory evaluations noisy. Sampling needs no gradient, so it's
   indifferent to all of this. Footnote 1 states the design philosophy directly: "This tutorial
   focuses on sampling-based methods because they impose the fewest assumptions on cost functions
   and dynamics."
2. **Even the resulting optimal control distribution $\pi^*$ can't be sampled directly.** §2.5:
   "Despite having a mathematical expression for the optimal control distribution in Eq. (2),
   directly sampling from it computationally is challenging." This forces a second use of sampling —
   Monte Carlo approximation of $\pi^*$ itself (§2.4.4: "we approximate the optimal control
   distribution in Eq. (2) using Monte Carlo sampling").

Sample efficiency is the reason it's *this* sampling method and not naive random shooting: §2.2 +
Fig. 2 show random shooting (pick the single best of $K$ samples) is exponentially sample-inefficient
in input dimension; estimating a whole *distribution* over good controls instead is what fixes it.

---

## Q1: "What is MPPI?"

Source: Williams et al., *Information Theoretic MPC for Model-Based RL*, ICRA 2017, §III-B–C,
Algorithm 2 (lessons [0001](lessons/0001-free-energy-and-the-optimal-distribution.html)/[0002](lessons/0002-importance-sampling-and-the-mppi-update-law.html)).

**15s opener:**
> MPPI — Model Predictive Path Integral control — is a sampling-based, gradient-free optimal control
> algorithm. Each cycle it draws $K$ random perturbations of the current control sequence, rolls each
> out through the dynamics, scores the trajectory cost, and combines them into a cost-weighted
> average — a softmax over trajectory cost, not a gradient step. It's receding-horizon: shift and
> warm-start after each cycle.

**Mechanism, mapped to Algorithm 2:**
- Sample: $v_t^{(k)} = u_t + \varepsilon_t^{(k)},\ \varepsilon_t^{(k)}\sim\mathcal N(0,\Sigma)$
- Rollout + cost: accumulate $S(E^{(k)})$
- Weight: $w^{(k)}\propto\exp(-\tfrac1\lambda S(E^{(k)}))$ ($\beta$-shifted for numerical stability)
- Update: $u_t \mathrel{+}= \sum_k w^{(k)}\varepsilon_t^{(k)}$
- Send $u_0$, shift, reinitialize tail, repeat

**Why, in one line:** never differentiates through dynamics or cost, so it's agnostic to
nonlinearity, nonconvexity, and nondifferentiable terms (crash indicators, hard thresholds, mode
switches) where QP/iLQR would need smoothing or wouldn't apply.

**If pushed on pedigree:** it's the closed-form solution to relaxing $\min_U E[\text{cost}]$ into
$\min_Q E_Q[S(V)] + \lambda D_{KL}(Q\|P)$ over *all* trajectory distributions, not just achievable
Gaussians — convex in $Q$, exact Gibbs/Boltzmann solution $Q^*$ — then projected back onto an
achievable Gaussian via importance sampling. This is also where $\lambda$ and the hard-constraints
answer come from (see Q3).

---

## Q2: "Is MPPI a method to solve a stochastic optimal control problem?"

### Version A — information-theoretic (papers 2/3)

Source: Williams et al., *Aggressive Driving with MPPI Control*, ICRA 2016, §II-A Eq. 1; lesson
[0004](lessons/0004-tube-mppi-nominal-real-tracking.html).

> Yes — paper 3 states it outright: "In the classical stochastic optimal control setting we seek a
> control sequence $u(\cdot)$ such that $u^*(\cdot)=\operatorname{argmin}_{u(\cdot)}E_Q[\phi(x_T,T)+\int L\,dt]$,"
> with dynamics "disturbed by Brownian motion $dw$." That's a controlled SDE with an expectation
> objective — textbook stochastic OC — and MPPI derives the optimal law for exactly that object.
>
> **Precision:** the noise that makes it stochastic is $\varepsilon_t\sim\mathcal N(0,\Sigma)$ — the
> *same* object MPPI samples $K$ times per cycle, because the derivation restricts the Brownian
> disturbance to enter through the same control-affine channel $G(x,t)$ as $u$. So MPPI solves the
> SOC problem for the noise it designs and injects, not for whatever the real plant experiences.
>
> **Proof it's not semantics:** this is exactly why Tube-MPPI exists. Plain MPPI's $K$ rollouts are
> scored under planning-time dynamics with only $\varepsilon_t$; an unmodeled execution-time
> disturbance $w_k$ (wind, slip, model error) never enters a rollout, so the softmax-weighted cost
> estimate goes stale (lesson 0004). If plain MPPI already solved the general stochastic problem,
> Tube-MPPI's nominal/real tracking split would have nothing to fix.

### Version B — probabilistic inference (`papers/mppi_overview.pdf`, §2.3.1 Steps 1–3)

> Yes, but via a different mechanism — this route needs **no stochastic dynamics at all**. The
> graphical-model transition $p(x_{t+1}|x_t,u_t)$ (Fig. 3) can be fully deterministic. What makes it
> "stochastic" is that the whole OCP is recast as Bayesian inference: introduce a virtual binary
> optimality variable $O_t$ and infer the posterior $p(\tau|O_{0:T}=1)$ via Bayes' rule (Eq. 3).
>
> **Precision, near-verbatim from the paper's own Step 2:** "since the prior distribution
> $p(u_{0:T-1})$ included in $p(\tau)$ may contain system-side input noise, it is **not necessarily
> directly controllable by us**." The randomness lives in the control prior you choose
> ($p(u)=\mathcal N(\mu^{\text{prev}}_{0:T-1},\Sigma)$, Eq. 12) and the artificial Boltzmann
> optimality likelihood (Eq. 6) — not a literal noise term in $f$.
>
> **Same gap, reached differently:** whatever noise sits in $p(u)$ is still a *designed* object, not
> a model of a real disturbance $w_k$ — same Tube-MPPI caveat as Version A, derived from a completely
> different starting point.

---

## Q3: "Why does λ trade exploration vs. exploitation, and why no hard-constraint guarantee?"

### λ — probabilistic-inference framing (`papers/mppi_overview.pdf` Eq. 5, 6, §2.4.3–2.4.4)

> λ is the inverse temperature in the VI objective, Eq. 5:
> $\pi^*=\min_\pi\{\lambda^{-1}E_\pi[J_\tau]+D_{KL}(\pi\|p)\}$ — the weight on expected cost relative
> to staying near the prior. Equivalently, Eq. 6 makes it the **precision of the Boltzmann
> "optimality likelihood"** $p(O{=}1|\tau)=\eta^{-1}\exp(-\lambda^{-1}J_\tau)$: small λ → an
> extremely peaked, overconfident likelihood → posterior collapses onto the cost minimum. The paper
> states the limits directly (§2.4.3): "$\lambda\to0$ yields deterministic optimal actions that
> minimize the cost function, while $\lambda\to\infty$ yields stochastic optimal actions following
> only the prior distribution."
>
> **Sourced bonus beyond the info-theoretic lessons:** §2.4.4 ties λ directly to *sample complexity* —
> smaller λ sharpens the target distribution being Monte-Carlo-approximated, directly increasing the
> samples $K$ needed for a good estimate (Kong 1992 effective-sample-size result; Yoon et al. 2022;
> Tao et al. 2022a).

(Info-theoretic version of this same question — softmax-temperature framing via the free-energy
exponential tilt — is already derived in lesson 0001; not duplicated here.)

### No hard-constraint guarantee — probabilistic-inference framing (Eq. 8, 9–13; lesson 0009's forward-KL vocabulary)

> **Baseline argument:** the only Lagrange multiplier in the entire §2.3 derivation is Eq. 8, and it
> enforces exactly one constraint — $\int\pi\,du_{0:T-1}=1$, probabilities summing to one. No
> multiplier anywhere is tied to a feasibility inequality $g(x)\le0$; no KKT system was ever set up.
> Constraints can only enter as penalties inside $J_\tau$, reshaping the Boltzmann likelihood — not
> the same machinery as enforcing a constraint.
>
> **Sharper, probabilistic-inference-specific layer:** even granting an idealized infinite-cost
> penalty that drives $\pi^*$'s density to exactly zero outside the feasible set, the executed control
> isn't a sample of $\pi^*$ — it's $\mu^*$, from a *second* projection, Eq. 9:
> $\min_\theta D_{KL}(\pi^*\|\pi_\theta)$, onto a Gaussian family. That's **forward KL** — the
> direction Honda's §3.2.2 calls "mode-covering" (already used in lesson
> [0009](lessons/0009-multimodal-distributions-cover-or-commit.html)): it heavily penalizes $\pi_\theta$
> for missing mass where $\pi^*$ has it, but never penalizes $\pi_\theta$ for placing mass where
> $\pi^*$ has none. A Gaussian has unbounded support by construction, so the forward-KL-optimal fit
> structurally cannot inherit $\pi^*$'s zero density outside the feasible region. $\mu^*$ (Eq. 12–13)
> is a softmax-weighted average under a distribution that was never forced to respect the boundary.

This last argument is new synthesis — not in lessons 0001–0009 as stated, though it leans on
vocabulary lesson 0009 already locked in (forward-KL = mode-covering). It's sharper than the
info-theoretic version (which argues from "average over a nonconvex set") because it names the exact
mechanism: forward KL's one-sided mass-covering behavior + a Gaussian's unbounded support.

---

## Status / open items

- This file is a plain-markdown cram sheet, not a formal `reference/*.html` cheat sheet or a
  `lessons/*.html` lesson — not wired into `index.html`. Say the word if it should become either.
- The probabilistic-inference route (`papers/mppi_overview.pdf` §2.3.1–2.5) is `NOTES.md`'s Part 4
  "candidate thread 1" (VI/control-as-inference derivation) — this session is the first time it's
  actually been exercised, not just logged as a candidate. `MISSION.md`/`NOTES.md` haven't been
  updated to reflect that yet; also worth a word if this should become a committed Part 4 thread
  rather than a one-off Q&A pass.