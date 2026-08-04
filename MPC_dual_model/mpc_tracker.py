"""High-level bridge: position measurement -> state estimate -> MPC -> command."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

try:
    from .device_adapter import (
        DeviceCommand,
        FineSUBThrusterAllocator,
        ForceCommandAdapter,
        ThrusterAllocation,
    )
    from .fossen_fixed_dl_model import FixedLinearDampingRelativeModel, vector3
    from .model_fusion import FusionConfig, OnlineModelFusion
    from .mpc_controller import MPCResult, RelativeMPCController
    from .relative_kalman import RelativePositionKalmanFilter
except ImportError:
    from device_adapter import (
        DeviceCommand,
        FineSUBThrusterAllocator,
        ForceCommandAdapter,
        ThrusterAllocation,
    )
    from fossen_fixed_dl_model import FixedLinearDampingRelativeModel, vector3
    from model_fusion import FusionConfig, OnlineModelFusion
    from mpc_controller import MPCResult, RelativeMPCController
    from relative_kalman import RelativePositionKalmanFilter


DEFAULT_STAIRCASE_HORIZON_CAPS = (3, 3, 2, 2, 1, 1)
DEFAULT_PREDICTION_HORIZON_WEIGHTS = (0.5, 0.3, 0.2)


def build_default_staircase_fusion() -> OnlineModelFusion:
    """Single source of truth for the maintained translation history shape."""
    return OnlineModelFusion(
        FusionConfig(
            window=len(DEFAULT_STAIRCASE_HORIZON_CAPS),
            prediction_horizon=len(DEFAULT_PREDICTION_HORIZON_WEIGHTS),
            forgetting_factor=0.8,
            prediction_horizon_weights=DEFAULT_PREDICTION_HORIZON_WEIGHTS,
            staircase_horizon_caps=DEFAULT_STAIRCASE_HORIZON_CAPS,
            initial_model1_weight=(0.80, 0.80, 0.80),
            minimum_weight=0.05,
        )
    )


@dataclass
class TrackerOutput:
    estimated_state: np.ndarray
    mpc: MPCResult
    device_command: DeviceCommand
    thruster_allocation: ThrusterAllocation | None


@dataclass
class SafeControlOutput:
    force: np.ndarray
    device_command: DeviceCommand
    status: str


@dataclass
class BaselineAdaptationConfig:
    """Optional slow baseline learning, active only near the matched state."""

    enabled: bool = False
    adaptation_rate: float = 0.02
    position_error_tolerance: float = 0.06
    velocity_tolerance: float = 0.04


@dataclass
class _PendingPositionPrediction:
    """A historical model comparison waiting for future position measurements."""

    origin_index: int
    state1: np.ndarray
    state2: np.ndarray
    tau_previous: np.ndarray
    steps: int = 0


class MPCTracker:
    """Combine estimator, controller, and lower-controller command scaling."""

    def __init__(
        self,
        model: FixedLinearDampingRelativeModel,
        estimator: RelativePositionKalmanFilter,
        controller: RelativeMPCController,
        adapter: ForceCommandAdapter,
        fusion: OnlineModelFusion | None = None,
        thruster_allocator: FineSUBThrusterAllocator | None = None,
        baseline_adaptation: BaselineAdaptationConfig | None = None,
    ) -> None:
        if estimator.model is not model or controller.model is not model:
            raise ValueError("model, estimator.model, and controller.model must be identical")
        self.model = model
        self.estimator = estimator
        self.controller = controller
        self.adapter = adapter
        self.fusion = fusion or build_default_staircase_fusion()
        self.thruster_allocator = thruster_allocator
        self.baseline_adaptation = baseline_adaptation or BaselineAdaptationConfig()
        self._tau_before_last = np.zeros(3)
        self._pending_position_predictions: deque[_PendingPositionPrediction] = deque()
        self._frame_index = -1

    def latch_baseline(self, tau_achieved) -> None:
        """Initialize rolling force history when automatic tracking is enabled."""
        achieved = vector3(tau_achieved, "tau_achieved")
        self.model.set_tau_base(achieved)  # retained only for target-lost fallback
        self._tau_before_last = achieved.copy()
        self.fusion.reset()
        self.controller.reset()
        self._pending_position_predictions.clear()
        self._frame_index = -1

    def target_lost(self, tau_achieved_previous) -> SafeControlOutput:
        """Stop target maneuvers and rate-limit force back to the latched baseline."""
        self.controller.reset()
        self._pending_position_predictions.clear()
        force = self.controller.safe_force(tau_achieved_previous)
        return SafeControlOutput(
            force=force,
            device_command=self.adapter.convert(force),
            status="target_lost:return_to_baseline",
        )

    def _adapt_baseline_if_matched(
        self,
        state: np.ndarray,
        tau_achieved: np.ndarray,
        reference_position,
    ) -> None:
        cfg = self.baseline_adaptation
        if not cfg.enabled:
            return
        reference = (
            self.controller.config.reference_position
            if reference_position is None
            else vector3(reference_position, "reference_position")
        )
        if (
            np.linalg.norm(state[:3] - reference) <= cfg.position_error_tolerance
            and np.linalg.norm(state[3:]) <= cfg.velocity_tolerance
        ):
            alpha = float(cfg.adaptation_rate)
            if not 0.0 < alpha <= 1.0:
                raise ValueError("baseline adaptation rate must be in (0, 1]")
            self.model.tau_base = (
                (1.0 - alpha) * self.model.tau_base + alpha * tau_achieved
            )

    def _score_completed_position_predictions(
        self,
        filtered_position: np.ndarray,
        tau_achieved: np.ndarray,
    ) -> None:
        """Advance all historical candidates with actual force, then score p.

        A record started at frame i is advanced by every actually applied force
        until frame i+h arrives.  At that point its model-1 and model-2
        predicted positions are compared against the filtered camera position.
        """
        retained: deque[_PendingPositionPrediction] = deque()
        for record in self._pending_position_predictions:
            record.state1 = (
                self.model.A_d @ record.state1
                + self.model.B_d @ (tau_achieved - record.tau_previous)
            )
            record.state2 = (
                self.model.A_d @ record.state2
                + self.model.B_d @ tau_achieved
            )
            record.tau_previous = tau_achieved.copy()
            record.steps += 1
            self.fusion.observe_position(
                actual_position=filtered_position,
                prediction1=record.state1[:3],
                prediction2=record.state2[:3],
                horizon_step=record.steps,
                origin_index=record.origin_index,
                target_index=self._frame_index,
            )
            if record.steps < self.fusion.config.prediction_horizon:
                retained.append(record)
        self._pending_position_predictions = retained

    def _start_position_prediction(self, state: np.ndarray, tau_previous: np.ndarray) -> None:
        """Save the current posterior as a new multi-step comparison origin."""
        self._pending_position_predictions.append(
            _PendingPositionPrediction(
                origin_index=self._frame_index,
                state1=state.copy(),
                state2=state.copy(),
                tau_previous=tau_previous.copy(),
            )
        )

    def update(
        self,
        position_body,
        tau_achieved_previous,
        reference_position=None,
    ) -> TrackerOutput:
        """Process one new 3-D position measurement.

        tau_achieved_previous is the force that acted during the interval since
        the preceding image. When no force observer is available, pass the
        prior saturated command as an approximation.
        """
        tau_achieved = vector3(tau_achieved_previous, "tau_achieved_previous")
        self._frame_index += 1
        if not self.estimator.initialized:
            state = self.estimator.initialize(position_body)
            self.fusion.advance_time(self._frame_index)
        else:
            old_state = self.estimator.x.copy()
            prediction1 = (
                self.model.A_d @ old_state
                + self.model.B_d @ (tau_achieved - self._tau_before_last)
            )
            prediction2 = (
                self.model.A_d @ old_state + self.model.B_d @ tau_achieved
            )
            weight6 = np.diag(
                np.concatenate(
                    (self.fusion.model1_weight, self.fusion.model1_weight)
                )
            )
            fused_prediction = (
                weight6 @ prediction1 + (np.eye(6) - weight6) @ prediction2
            )
            self.estimator.predict_mean(fused_prediction)
            state = self.estimator.update(position_body)
            self._score_completed_position_predictions(state[:3], tau_achieved)
        self._tau_before_last = tau_achieved.copy()
        self._adapt_baseline_if_matched(state, tau_achieved, reference_position)
        self._start_position_prediction(state, tau_achieved)
        result = self.controller.solve(
            state=state,
            tau_previous=tau_achieved,
            reference_position=reference_position,
            model1_weight=self.fusion.model1_weight,
        )
        allocation = (
            None
            if self.thruster_allocator is None
            else self.thruster_allocator.allocate(result.force)
        )
        return TrackerOutput(
            estimated_state=state,
            mpc=result,
            device_command=self.adapter.convert(result.force),
            thruster_allocation=allocation,
        )
