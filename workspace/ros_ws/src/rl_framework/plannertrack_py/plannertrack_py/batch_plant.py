# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""B cars on the same reference, stepped together (batched MPC solves, steps.md §6).

:class:`BatchPlant` holds B independent :class:`~plannertrack_py.plant.Plant` instances (one C++
simulator each) that share one :class:`~plannertrack_py.reference.ReferenceTrack`. States and inputs
are stacked as (B, nx) / (B, nu), the layout of leap-c's batched ``AcadosDiffMpcTorch`` calls, and
the per-car ``info`` dicts are stacked into arrays of shape (B,). Each car has its own mass/mu, RNG
stream, progress and episode flags, and can be reset on its own (the others keep driving).

Seeding: one master seed is split with ``numpy.random.SeedSequence(seed).spawn(B)``, so car i
behaves exactly like a single ``Plant`` reset with the i-th child seed, and a batch run is
reproducible from one number.

Example::

    batch = BatchPlant.from_scenario(PARAMS_DIR, n_cars=8, reference=ref, dt=0.02,
                                     perturbation_range=PerturbationRange(mu_scale=(0.9, 1.0)))
    x, info = batch.reset(seed=0)                 # x: (8, 8); info["s0"]: (8,)
    x, info = batch.step(u)                       # u: (8, 2); info["n"], info["terminated"]: (8,)
    done = np.flatnonzero(info["terminated"] | info["lap_done"])
    x[done], _ = batch.reset_cars(done)           # new episodes for those cars only
"""

from pathlib import Path

import numpy as np

from plannertrack_py.plant import NU, NX, Perturbation, Plant
from plannertrack_py.raceline import compute_raceline
from plannertrack_py.reference import ReferenceTrack


class BatchPlant:
    """B :class:`Plant` s on one reference, with stacked states, inputs and info.

    Args:
        sim_yaml_path, agents_yaml_path, planner_yaml_path, agent_name, reference: as for
            :class:`Plant`; every car uses the same scenario and the same reference object.
        n_cars: batch size B.
        perturbations: optional list of B :class:`Perturbation` (one per car); default nominal.
        **plant_kwargs: passed to every :class:`Plant` (dt, perturbation_range, start_noise).
    """

    def __init__(self, sim_yaml_path, agents_yaml_path, planner_yaml_path, agent_name, reference,
                 n_cars, perturbations=None, **plant_kwargs):
        if n_cars < 1:
            raise ValueError(f"n_cars must be >= 1, got {n_cars}")
        if perturbations is not None and len(perturbations) != n_cars:
            raise ValueError(f"{len(perturbations)} perturbations for {n_cars} cars")
        if "perturbation" in plant_kwargs:
            raise ValueError("pass per-car values as perturbations=[...]")
        perturbations = perturbations or [Perturbation()] * n_cars
        self.reference = reference
        self.plants = [Plant(sim_yaml_path, agents_yaml_path, planner_yaml_path, agent_name, reference,
                             perturbation=p, **plant_kwargs) for p in perturbations]
        self._seeds = None  # per-car child SeedSequences of the last reset(seed=...)

    @classmethod
    def from_scenario(cls, params_dir, n_cars, agent_name="agent_1", reference=None, **kwargs):
        """Batch from a scenario's ``params/`` folder; the raceline is computed once if not given."""
        p = Path(params_dir)
        sim, agents, planner = p / "sim.yaml", p / "agents.yaml", p / "min_curv_planner.yaml"
        if reference is None:
            reference = ReferenceTrack(compute_raceline(agents, agent_name, planner))
        return cls(sim, agents, planner, agent_name, reference, n_cars, **kwargs)

    # ------------------------------------------------------------ properties

    def __len__(self):
        return len(self.plants)

    @property
    def n_cars(self):
        return len(self.plants)

    @property
    def dt(self):
        return self.plants[0].dt

    @property
    def limits(self):
        return self.plants[0].limits

    @property
    def nominal_params(self):
        """Nominal vehicle parameters (the same for every car; what the MPC gets)."""
        return self.plants[0].nominal_params

    @property
    def perturbations(self):
        return [p.perturbation for p in self.plants]

    def states(self):
        """Current states, (B, nx)."""
        return np.stack([p.sim.get_state() for p in self.plants])

    def set_perturbations(self, perturbations):
        """Change every car's true mass/mu (list of B :class:`Perturbation`); states are kept."""
        if len(perturbations) != self.n_cars:
            raise ValueError(f"{len(perturbations)} perturbations for {self.n_cars} cars")
        for plant, pert in zip(self.plants, perturbations):
            plant.set_perturbation(pert)

    # ------------------------------------------------------------- episodes

    def reset(self, seed=None, s0=None, perturbations=None):
        """Reset every car. ``seed`` is split into one child seed per car (None: continue each
        car's stream); ``s0`` is None (random per car), a scalar or B values; ``perturbations``
        is None or B values (None entries keep/draw as :meth:`Plant.reset`).

        Returns ``(states (B, nx), info)`` with the reset infos stacked into (B,) arrays.
        """
        if seed is not None:
            self._seeds = np.random.SeedSequence(seed).spawn(self.n_cars)
        return self.reset_cars(range(self.n_cars), s0=s0, perturbations=perturbations,
                               seeds=self._seeds if seed is not None else None)

    def reset_cars(self, cars, s0=None, perturbations=None, seeds=None):
        """Reset only the cars with indices ``cars`` (the others keep their state and episode).

        ``s0``, ``perturbations`` and ``seeds`` are None, or one value per entry of ``cars``
        (``s0`` may also be a scalar). Returns ``(states (len(cars), nx), info)`` for those cars.
        """
        cars = np.atleast_1d(np.asarray(cars, dtype=int))
        k = len(cars)
        s0s = _per_car(s0, k, "s0", allow_scalar=True)
        perts = _per_car(perturbations, k, "perturbations")
        seeds_k = _per_car(seeds, k, "seeds")
        states, infos = [], []
        for j, i in enumerate(cars):
            x, info = self.plants[i].reset(seed=seeds_k[j], s0=s0s[j], perturbation=perts[j])
            states.append(x)
            infos.append(info)
        return np.reshape(states, (k, NX)), stack_infos(infos)

    def step(self, u):
        """Apply ``u`` (B, nu), one row per car, for one interval. Returns ``(states (B, nx), info)``
        with every :meth:`Plant.step` info field stacked into a (B,) array. Cars whose episode has
        ended keep being simulated; reset them with :meth:`reset_cars`."""
        u = np.asarray(u, dtype=float)
        if u.shape != (self.n_cars, NU):
            raise ValueError(f"u must have shape ({self.n_cars}, {NU}), got {u.shape}")
        results = [plant.step(ui) for plant, ui in zip(self.plants, u)]
        return np.stack([x for x, _ in results]), stack_infos([info for _, info in results])


def stack_infos(infos):
    """List of per-car info dicts -> dict of arrays (B,). Nested dicts (the perturbation) stay a
    list."""
    out = {}
    for key in infos[0]:
        values = [info[key] for info in infos]
        out[key] = values if isinstance(values[0], dict) else np.asarray(values)
    return out


def _per_car(values, k, name, allow_scalar=False):
    if values is None:
        return [None] * k
    if allow_scalar and np.isscalar(values):
        return [float(values)] * k
    values = list(values)
    if len(values) != k:
        raise ValueError(f"{name}: {len(values)} values for {k} cars")
    return values
