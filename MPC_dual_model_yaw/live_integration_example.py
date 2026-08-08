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
from MPC_dual_model.device_adapter import (
    FineSUBThrusterAllocator,
    ForceCommandAdapter,
)
from MPC_dual_model.fossen_fixed_dl_model import (
    FixedLinearDampingRelativeModel,
)
from MPC_dual_model.finesub_protocol import (
    FineSUBHardwareAdapter,
    build_default_hardware_adapter,
)
from MPC_dual_model.relative_kalman import KalmanConfig

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
    translation = FixedLinearDampingRelativeModel(
        # TODO: pool-identification results, including translational added mass.
        M_t=np.diag([20.0, 25.0, 30.0]),
        D_L=np.diag([8.0, 10.0, 12.0]),
        dt=0.05,
        tau_base=np.zeros(3),
    )
    yaw = LinearYawDynamics(
        # TODO: m_omega=I_z-N_rdot and identified yaw damping.
        effective_inertia=2.5,
        linear_damping=1.2,
        quadratic_damping=0.0,
        dt=translation.dt,
    )
    model = RotationAwareRelativeModel(translation, yaw)
    yaw_controller = YawStateController(
        yaw,
        YawControlConfig(
            # TODO: determine FOV trigger hysteresis and all PID gains in pool tests.
            alpha_on=np.deg2rad(25.0),
            alpha_off=np.deg2rad(10.0),
            alpha_emergency=np.deg2rad(35.0),
            trigger_frames=3,
            settle_frames=5,
            omega_command_max=np.deg2rad(35.0),
            omega_command_acceleration_max=np.deg2rad(60.0),
            yaw_moment_min=-4.0,
            yaw_moment_max=4.0,
            delta_yaw_moment_min=-0.8,
            delta_yaw_moment_max=0.8,
        ),
    )
    controller = RotationAwareMPCController(
        model,
        YawMPCConfig(
            horizon=10,
            reference_position=(0.60, 0.0, 0.0),
            force_min=(-20.0, -15.0, -15.0),
            force_max=(20.0, 15.0, 15.0),
            delta_force_min=(-3.0, -2.0, -2.0),
            delta_force_max=(3.0, 2.0, 2.0),
        ),
    )
    estimator = RotationAwareKalmanFilter(
        model,
        KalmanConfig(position_std=(0.015, 0.015, 0.025)),
    )
    fusion = build_default_staircase_fusion()
    force_adapter = ForceCommandAdapter(
        positive_force_at_limit=(20.0, 15.0, 15.0),
        signs=(1.0, 1.0, 1.0),
        command_limits=(99.0, 99.0, 45.0),
    )
    yaw_adapter = YawMomentChannelAdapter(
        # TODO: measured yaw moment at FineSUB normalized yaw=+0.20.
        positive_yaw_moment_at_limit=4.0,
        channel_limit=0.20,
        sign=1.0,
    )
    allocator = FineSUBThrusterAllocator(
        positive_force_at_limit=(20.0, 15.0, 15.0),
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
