"""Online per-axis fusion using completed multi-step position predictions."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

try:
    from .fossen_fixed_dl_model import vector3
except ImportError:
    from fossen_fixed_dl_model import vector3


@dataclass
class FusionConfig:
    """Settings for correlated-error least-squares model weights.

    model-1: x+ = A x + B (tau - tau_previous)
    model-2: x+ = A x + B tau

    Each axis has an independent model-1 weight a1; model-2 uses 1-a1.

    ``window`` is the number of recent prediction start times to retain.
    Each start time generates one position residual for every completed step up
    to ``prediction_horizon``.  This realizes the triangular, multi-step
    history described in the design PDF.
    """

    window: int = 20
    prediction_horizon: int = 8
    forgetting_factor: float = 0.92
    horizon_weight_decay: float = 0.95
    epsilon: float = 1.0e-5
    minimum_weight: float = 0.05
    initial_model1_weight: object = (0.80, 0.80, 0.80)
    position_error_clip: object = (0.50, 0.50, 0.50)
    # Optional staircase mask. Entry r-1 is the largest retained prediction
    # horizon for an origin that is r control periods old. None preserves the
    # original rectangular/triangular completed-prediction history.
    staircase_horizon_caps: tuple[int, ...] | None = None
    # Optional explicit omega_h. When omitted, horizon_weight_decay**(h-1)
    # is used exactly as in the original implementation.
    prediction_horizon_weights: tuple[float, ...] | None = None
    # Backward-compatible alias.  Values supplied here are interpreted in m.
    velocity_error_clip: object | None = None

    def normalized(self) -> "FusionConfig":
        if self.window < 2:
            raise ValueError("fusion window must be at least 2")
        if self.prediction_horizon < 1:
            raise ValueError("prediction_horizon must be at least 1")
        if not 0.0 < self.forgetting_factor <= 1.0:
            raise ValueError("forgetting_factor must be in (0, 1]")
        if not 0.0 < self.horizon_weight_decay <= 1.0:
            raise ValueError("horizon_weight_decay must be in (0, 1]")
        if self.epsilon <= 0.0:
            raise ValueError("epsilon must be positive")
        if not 0.0 <= self.minimum_weight < 0.5:
            raise ValueError("minimum_weight must be in [0, 0.5)")
        self.initial_model1_weight = vector3(
            self.initial_model1_weight, "initial_model1_weight"
        )
        if self.velocity_error_clip is not None:
            self.position_error_clip = self.velocity_error_clip
        self.position_error_clip = vector3(
            self.position_error_clip, "position_error_clip"
        )
        if np.any(self.position_error_clip <= 0.0):
            raise ValueError("position_error_clip must be positive")
        if self.staircase_horizon_caps is not None:
            caps = tuple(int(value) for value in self.staircase_horizon_caps)
            if len(caps) != self.window:
                raise ValueError(
                    "staircase_horizon_caps must have exactly window entries"
                )
            if any(value < 1 or value > self.prediction_horizon for value in caps):
                raise ValueError(
                    "each staircase cap must be in [1,prediction_horizon]"
                )
            if any(caps[index] < caps[index + 1] for index in range(len(caps) - 1)):
                raise ValueError(
                    "staircase_horizon_caps must be non-increasing with origin age"
                )
            self.staircase_horizon_caps = caps
        if self.prediction_horizon_weights is not None:
            weights = np.asarray(
                self.prediction_horizon_weights, dtype=float
            ).reshape(-1)
            if (
                weights.size < self.prediction_horizon
                or np.any(~np.isfinite(weights))
                or np.any(weights < 0.0)
                or np.sum(weights[: self.prediction_horizon]) <= 0.0
            ):
                raise ValueError(
                    "prediction_horizon_weights must cover the horizon and be nonnegative"
                )
            self.prediction_horizon_weights = tuple(weights.tolist())
        lower = self.minimum_weight
        upper = 1.0 - lower
        self.initial_model1_weight = np.clip(
            self.initial_model1_weight, lower, upper
        )
        return self


@dataclass(frozen=True)
class _TimedPositionError:
    """One completed prediction cell in the origin/target-time plane."""

    origin_index: int
    target_index: int
    horizon_step: int
    error1: np.ndarray
    error2: np.ndarray


class OnlineModelFusion:
    """Estimate fusion weights from completed multi-step position residuals."""

    def __init__(self, config: FusionConfig | None = None) -> None:
        self.config = (config or FusionConfig()).normalized()
        maximum_samples = self.config.window * self.config.prediction_horizon
        self._errors1: deque[np.ndarray] = deque(maxlen=maximum_samples)
        self._errors2: deque[np.ndarray] = deque(maxlen=maximum_samples)
        self._horizon_weights: deque[float] = deque(maxlen=maximum_samples)
        self._timed_errors: deque[_TimedPositionError] = deque(
            maxlen=maximum_samples
        )
        self._active_pairs: tuple[tuple[int, int], ...] = ()
        self.current_index = 0
        self.model1_weight = self.config.initial_model1_weight.copy()
        self.M1 = np.zeros(3)
        self.M2 = np.zeros(3)
        self.C12 = np.zeros(3)

    @property
    def model2_weight(self) -> np.ndarray:
        return 1.0 - self.model1_weight

    @property
    def sample_count(self) -> int:
        if self.config.staircase_horizon_caps is not None:
            return len(self._active_pairs)
        return len(self._errors1)

    @property
    def active_pairs(self) -> tuple[tuple[int, int], ...]:
        """Currently scored (origin_index,target_index) cells."""
        return self._active_pairs

    def reset(self) -> None:
        self._errors1.clear()
        self._errors2.clear()
        self._horizon_weights.clear()
        self._timed_errors.clear()
        self._active_pairs = ()
        self.current_index = 0
        self.model1_weight = self.config.initial_model1_weight.copy()
        self.M1.fill(0.0)
        self.M2.fill(0.0)
        self.C12.fill(0.0)

    def _set_scores(
        self,
        errors1: np.ndarray,
        errors2: np.ndarray,
        weights: np.ndarray,
    ) -> np.ndarray:
        weights = np.asarray(weights, dtype=float).reshape(-1)
        if weights.size == 0 or np.sum(weights) <= 0.0:
            return self.model1_weight.copy()
        weights = weights / np.sum(weights)
        self.M1 = np.sum(weights[:, None] * errors1**2, axis=0)
        self.M2 = np.sum(weights[:, None] * errors2**2, axis=0)
        self.C12 = np.sum(weights[:, None] * errors1 * errors2, axis=0)

        denominator = self.M1 + self.M2 - 2.0 * self.C12
        raw = (self.M2 - self.C12 + self.config.epsilon) / (
            denominator + 2.0 * self.config.epsilon
        )
        lower = self.config.minimum_weight
        self.model1_weight = np.clip(raw, lower, 1.0 - lower)
        return self.model1_weight.copy()

    def advance_time(self, current_index: int) -> np.ndarray:
        """Recompute a configured staircase mask at one control time.

        The mask uses origin age r=current-origin and prediction horizon
        h=target-origin.  Target-time recency and horizon weights are then
        normalized only across cells that remain inside the staircase.
        """
        if self.config.staircase_horizon_caps is None:
            return self.model1_weight.copy()
        current = int(current_index)
        if current < self.current_index:
            raise ValueError("current_index cannot move backward")
        self.current_index = current
        caps = self.config.staircase_horizon_caps
        retained = deque(maxlen=self._timed_errors.maxlen)
        active: list[_TimedPositionError] = []
        for sample in self._timed_errors:
            origin_age = current - sample.origin_index
            if 1 <= origin_age <= len(caps):
                retained.append(sample)
                if sample.horizon_step <= caps[origin_age - 1]:
                    active.append(sample)
        self._timed_errors = retained
        self._active_pairs = tuple(
            (sample.origin_index, sample.target_index) for sample in active
        )
        if not active:
            return self.model1_weight.copy()

        weights = []
        for sample in active:
            target_age = current - sample.target_index
            recency_weight = self.config.forgetting_factor**target_age
            if self.config.prediction_horizon_weights is None:
                horizon_weight = self.config.horizon_weight_decay ** (
                    sample.horizon_step - 1
                )
            else:
                horizon_weight = self.config.prediction_horizon_weights[
                    sample.horizon_step - 1
                ]
            weights.append(recency_weight * horizon_weight)
        errors1 = np.asarray([sample.error1 for sample in active])
        errors2 = np.asarray([sample.error2 for sample in active])
        return self._set_scores(errors1, errors2, np.asarray(weights))

    def observe_position(
        self,
        actual_position,
        prediction1,
        prediction2,
        horizon_step: int | None = None,
        origin_index: int | None = None,
        target_index: int | None = None,
    ) -> np.ndarray:
        """Score one completed position prediction from a historical start.

        ``horizon_step`` is the number of elapsed control periods between the
        saved start state and this position measurement.  Longer predictions
        can be gently discounted while still contributing to model selection.
        """
        if horizon_step is None:
            if origin_index is None or target_index is None:
                raise ValueError("horizon_step or both time indices are required")
            horizon_step = int(target_index) - int(origin_index)
        horizon_step = int(horizon_step)
        if not 1 <= horizon_step <= self.config.prediction_horizon:
            raise ValueError("horizon_step is outside the configured prediction horizon")
        actual = vector3(actual_position, "actual_position")
        predicted1 = vector3(prediction1, "prediction1")
        predicted2 = vector3(prediction2, "prediction2")
        limit = self.config.position_error_clip
        error1 = np.clip(actual - predicted1, -limit, limit)
        error2 = np.clip(actual - predicted2, -limit, limit)

        if self.config.staircase_horizon_caps is not None:
            if origin_index is None or target_index is None:
                raise ValueError(
                    "staircase scoring requires origin_index and target_index"
                )
            origin = int(origin_index)
            target = int(target_index)
            if target - origin != horizon_step:
                raise ValueError("time indices do not match horizon_step")
            self._timed_errors.append(
                _TimedPositionError(
                    origin_index=origin,
                    target_index=target,
                    horizon_step=horizon_step,
                    error1=error1,
                    error2=error2,
                )
            )
            return self.advance_time(max(self.current_index, target))

        self._errors1.append(error1)
        self._errors2.append(error2)
        self._horizon_weights.append(
            self.config.horizon_weight_decay ** (horizon_step - 1)
        )

        E1 = np.asarray(self._errors1)
        E2 = np.asarray(self._errors2)
        age = np.arange(len(E1) - 1, -1, -1, dtype=float)
        weights = np.asarray(self._horizon_weights) * (
            self.config.forgetting_factor**age
        )
        return self._set_scores(E1, E2, weights)

    def observe(self, actual_velocity, prediction1, prediction2) -> np.ndarray:
        """Compatibility wrapper for legacy callers.

        New tracker code must use :meth:`observe_position` with completed
        multi-step position predictions.  This method treats its vectors as a
        one-step position score to avoid silently preserving velocity scoring.
        """
        return self.observe_position(
            actual_velocity,
            prediction1,
            prediction2,
            horizon_step=1,
        )
