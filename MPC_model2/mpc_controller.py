"""Model-2-only constrained MPC.

The physical prediction used at every step is

    x[k+1] = A_d x[k] + B_d tau[k].

The previous force is used only by the force-rate constraint and the
delta-force cost. There is no model-1 candidate or fusion weight in this
public controller interface.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from MPC_dual_model_FineSUB_20260803.mpc_controller import (
    MPCConfig,
    RelativeMPCController as _SharedQPController,
)


@dataclass
class MPCResult:
    force: np.ndarray
    force_sequence: np.ndarray
    predicted_states: np.ndarray
    slacks: np.ndarray
    status: str
    iterations: int
    objective: float
    used_fallback: bool


class RelativeMPCController(_SharedQPController):
    """The shared QP implementation with its prediction fixed to model 2."""

    def _build_prediction_matrices(self, _unused_weight=None) -> None:
        # Fix a1=[0,0,0]. This reduces the shared physical dynamics to
        # x+ = A_d*x + B_d*tau. The augmented previous-force state remains
        # only to share the QP machinery; it has zero influence on x.
        super()._build_prediction_matrices(np.zeros(3))

    def solve(
        self,
        state,
        tau_previous,
        reference_position=None,
    ) -> MPCResult:
        shared = super().solve(
            state=state,
            tau_previous=tau_previous,
            reference_position=reference_position,
            model1_weight=np.zeros(3),
        )
        return MPCResult(
            force=shared.force,
            force_sequence=shared.force_sequence,
            predicted_states=shared.predicted_states,
            slacks=shared.slacks,
            status=shared.status,
            iterations=shared.iterations,
            objective=shared.objective,
            used_fallback=shared.used_fallback,
        )
