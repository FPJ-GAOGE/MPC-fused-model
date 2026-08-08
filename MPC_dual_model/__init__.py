"""Fixed-D_L relative MPC package."""

from .camera_transform import ALIGNED_OPENCV_TO_BODY, camera_to_body_position
from .device_adapter import (
    DeviceCommand,
    FINESUB_CANONICAL_THRUSTERS,
    FINESUB_V4_PRO1_FORCE_NEGATIVE_N,
    FINESUB_V4_PRO1_FORCE_POSITIVE_N,
    FineSUBThrusterAllocator,
    ForceCommandAdapter,
    ThrusterAllocation,
    finesub_translation_command_matrix,
    finesub_translation_force_bounds,
    finesub_translation_thruster_force_matrix,
)
from .fossen_fixed_dl_model import FixedLinearDampingRelativeModel
from .mpc_controller import MPCConfig, MPCResult, RelativeMPCController
from .model_fusion import FusionConfig, OnlineModelFusion
from .mpc_tracker import (
    BaselineAdaptationConfig,
    DEFAULT_PREDICTION_HORIZON_WEIGHTS,
    DEFAULT_STAIRCASE_HORIZON_CAPS,
    MPCTracker,
    SafeControlOutput,
    TrackerOutput,
    build_default_staircase_fusion,
)
from .relative_kalman import KalmanConfig, RelativePositionKalmanFilter

__all__ = [
    "DeviceCommand",
    "FINESUB_CANONICAL_THRUSTERS",
    "FINESUB_V4_PRO1_FORCE_NEGATIVE_N",
    "FINESUB_V4_PRO1_FORCE_POSITIVE_N",
    "BaselineAdaptationConfig",
    "DEFAULT_PREDICTION_HORIZON_WEIGHTS",
    "DEFAULT_STAIRCASE_HORIZON_CAPS",
    "ALIGNED_OPENCV_TO_BODY",
    "FixedLinearDampingRelativeModel",
    "ForceCommandAdapter",
    "FineSUBThrusterAllocator",
    "ThrusterAllocation",
    "finesub_translation_command_matrix",
    "finesub_translation_force_bounds",
    "finesub_translation_thruster_force_matrix",
    "FusionConfig",
    "OnlineModelFusion",
    "KalmanConfig",
    "MPCConfig",
    "MPCResult",
    "MPCTracker",
    "RelativeMPCController",
    "RelativePositionKalmanFilter",
    "SafeControlOutput",
    "TrackerOutput",
    "build_default_staircase_fusion",
    "camera_to_body_position",
]
