"""Experimental yaw-aware extension of the maintained dual-model MPC."""

from .yaw_kalman import RotationAwareKalmanFilter
from .yaw_mpc_controller import (
    RotationAwareMPCController,
    YawMPCConfig,
    YawMPCResult,
)
from .yaw_relative_model import (
    LinearYawDynamics,
    RotationAwareRelativeModel,
    line_of_sight_angle,
    rotation_body_from_previous,
    rotation_compensated_velocity,
    wrap_angle,
)
from .yaw_tracker import (
    DEFAULT_PREDICTION_HORIZON_WEIGHTS,
    DEFAULT_STAIRCASE_HORIZON_CAPS,
    RotationAwareMPCTracker,
    YawMomentChannelAdapter,
    YawSafeControlOutput,
    YawTrackerOutput,
    build_default_staircase_fusion,
)

__all__ = [
    "DEFAULT_PREDICTION_HORIZON_WEIGHTS",
    "DEFAULT_STAIRCASE_HORIZON_CAPS",
    "LinearYawDynamics",
    "RotationAwareKalmanFilter",
    "RotationAwareMPCController",
    "RotationAwareMPCTracker",
    "RotationAwareRelativeModel",
    "YawMPCConfig",
    "YawMPCResult",
    "YawMomentChannelAdapter",
    "YawSafeControlOutput",
    "YawTrackerOutput",
    "build_default_staircase_fusion",
    "line_of_sight_angle",
    "rotation_body_from_previous",
    "rotation_compensated_velocity",
    "wrap_angle",
]
