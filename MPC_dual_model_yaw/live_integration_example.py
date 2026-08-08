"""Safe construction and one-update interface for the yaw MPC experiment.

This module does not open a camera, socket, serial port, or propeller output.
All numerical values marked TODO are placeholders and must be identified.
Run it from the MPC workspace root with ``python -m ...`` as shown in README.
"""

from __future__ import annotations

import numpy as np

from MPC_dual_model.camera_transform import (
    ALIGNED_OPENCV_TO_BODY,
    camera_to_body_position,
)
from MPC_dual_model.dense_qp import QPSolverSettings
from MPC_dual_model.device_adapter import (
    FINESUB_V4_PRO1_FORCE_NEGATIVE_N,
    FINESUB_V4_PRO1_FORCE_POSITIVE_N,
    FineSUBThrusterAllocator,
    ForceCommandAdapter,
    finesub_six_dof_wrench_matrix_frd,
    finesub_translation_force_outer_bounds,
)
from MPC_dual_model.fossen_fixed_dl_model import (
    FixedLinearDampingRelativeModel,
)
from MPC_dual_model.finesub_protocol import (
    FineSUBHardwareAdapter,
    build_default_hardware_adapter,
)
from MPC_dual_model.relative_kalman import KalmanConfig
from MPC_dual_model.mpc_tracker import BaselineAdaptationConfig

from .yaw_kalman import RotationAwareKalmanFilter
from .yaw_controller import YawControlConfig, YawStateController
from .yaw_mpc_controller import RotationAwareMPCController, YawMPCConfig
from .yaw_relative_model import LinearYawDynamics, RotationAwareRelativeModel
from .yaw_tracker import (
    RotationAwareMPCTracker,
    YawMomentChannelAdapter,
    build_default_staircase_fusion,
)


def build_hardware_adapter() -> FineSUBHardwareAdapter:
    return build_default_hardware_adapter()


def to_finesub_command(output, *, armed: bool, adapter=None):
    """Convert translation force and yaw moment for full upper-level control."""

    hardware = adapter or build_hardware_adapter()
    if hasattr(output, "mpc"):
        force = output.mpc.force
        moment = output.yaw_control.yaw_moment
    else:
        force = output.force
        moment = output.yaw_moment
    return hardware.convert(force, moment, armed=armed, yaw_direct=True)


def build_tracker() -> RotationAwareMPCTracker:
    force_min, force_max = finesub_translation_force_outer_bounds()
    wrench_matrix = finesub_six_dof_wrench_matrix_frd()
    translation = FixedLinearDampingRelativeModel(
        # Identified in UnderwaterVision with direct +/-6 N and +/-12 N steps.
        # Axis order is body forward/right/down = Unity X/Z/-Y.
        M_t=np.diag([26.07276, 26.79684, 26.07276]),
        D_L=np.diag([93.88006, 143.69195, 280.86849]),
        dt=0.05,
        tau_base=np.zeros(3),
    )
    yaw = LinearYawDynamics(
        # Closed-loop UnderwaterVision estimate from direct-world yaw trials.
        effective_inertia=0.8,
        linear_damping=0.8,
        quadratic_damping=0.0,
        dt=translation.dt,
    )
    model = RotationAwareRelativeModel(translation, yaw)
    yaw_controller = YawStateController(
        yaw,
        YawControlConfig(
            alpha_on=np.deg2rad(1.8),
            alpha_off=np.deg2rad(0.7),
            alpha_emergency=np.deg2rad(4.5),
            trigger_frames=3,
            settle_frames=5,
            outer_kp=4.0,
            outer_kd=0.8,
            inner_kp=2.0,
            inner_ki=0.1,
            omega_command_max=np.deg2rad(45.0),
            omega_command_acceleration_max=np.deg2rad(90.0),
            yaw_moment_min=-1.5,
            yaw_moment_max=1.5,
            delta_yaw_moment_min=-0.25,
            delta_yaw_moment_max=0.25,
        ),
    )
    controller = RotationAwareMPCController(
        model,
        YawMPCConfig(
            horizon=10,
            # Translation parameters mirror the tuned fusion-model simulation.
            reference_position=(0.80, 0.0, 0.0),
            position_weights=(10000.0, 14000.0, 25000.0),
            velocity_weights=(2.0, 20.0, 12.0),
            force_weights=(0.003, 0.002, 0.04),
            delta_force_weights=(0.01, 0.01, 0.5),
            force_min=force_min,
            force_max=force_max,
            delta_force_min=(-4.0, -4.0, -5.6),
            delta_force_max=(4.0, 4.0, 5.6),
            thruster_wrench_matrix=wrench_matrix,
            thruster_force_min=-np.asarray(FINESUB_V4_PRO1_FORCE_NEGATIVE_N),
            thruster_force_max=np.asarray(FINESUB_V4_PRO1_FORCE_POSITIVE_N),
            horizontal_half_fov_deg=42.0,
            vertical_half_fov_deg=30.0,
            fov_margin_deg=5.0,
            slack_quadratic_weight=5.0e4,
            slack_linear_weight=100.0,
            solver_settings=QPSolverSettings(
                rho=10.0,
                sigma=1e-8,
                max_iterations=1500,
                absolute_tolerance=2e-5,
                relative_tolerance=3e-4,
                time_limit_seconds=0.035,
            ),
        ),
    )
    estimator = RotationAwareKalmanFilter(
        model,
        KalmanConfig(position_std=(0.015, 0.015, 0.025)),
    )
    fusion = build_default_staircase_fusion()
    force_adapter = ForceCommandAdapter(
        positive_force_at_limit=force_max,
        signs=(1.0, 1.0, 1.0),
        command_limits=(99.0, 99.0, 45.0),
    )
    yaw_adapter = YawMomentChannelAdapter(
        # Unity tuning limit; replace with measured real-vehicle calibration.
        positive_yaw_moment_at_limit=2.0,
        channel_limit=0.20,
        sign=1.0,
    )
    allocator = FineSUBThrusterAllocator(
        positive_force_at_limit=force_max,
        enable_depth=True,
    )
    return RotationAwareMPCTracker(
        model=model,
        estimator=estimator,
        controller=controller,
        yaw_controller=yaw_controller,
        force_adapter=force_adapter,
        yaw_adapter=yaw_adapter,
        fusion=fusion,
        thruster_allocator=allocator,
        baseline_adaptation=BaselineAdaptationConfig(
            enabled=True,
            update_mode="gated_ema",
            adaptation_rate=0.01,
            transient_adaptation_rate=0.03,
            steady_position_error_tolerance=0.03,
            steady_velocity_tolerance=0.02,
            position_error_tolerance=0.20,
            velocity_tolerance=0.08,
        ),
    )


def one_control_update(
    tracker: RotationAwareMPCTracker,
    position_camera_xyz,
    imu_yaw_rad: float,
    imu_yaw_rate_rad_s: float,
    last_achieved_force_body,
    last_achieved_yaw_moment: float,
    roll_pitch_control=(0.0, 0.0),
    rotation_body_from_camera=ALIGNED_OPENCV_TO_BODY,
    camera_origin_in_body=(0.0, 0.0, 0.0),
):
    """Call once for each timestamp-aligned 3-D camera measurement."""
    position_body = camera_to_body_position(
        position_camera_xyz,
        rotation_body_from_camera=rotation_body_from_camera,
        camera_origin_in_body=camera_origin_in_body,
    )
    rotation_body_from_camera = np.asarray(
        rotation_body_from_camera, dtype=float
    )
    rotation_visibility_from_body = (
        ALIGNED_OPENCV_TO_BODY @ rotation_body_from_camera.T
    )
    return tracker.update(
        position_body=position_body,
        yaw_rad=imu_yaw_rad,
        yaw_rate_rad_s=imu_yaw_rate_rad_s,
        force_achieved_previous=last_achieved_force_body,
        yaw_moment_achieved_previous=last_achieved_yaw_moment,
        roll_pitch_control=roll_pitch_control,
        rotation_visibility_from_body=rotation_visibility_from_body,
        camera_origin_in_body=camera_origin_in_body,
    )
