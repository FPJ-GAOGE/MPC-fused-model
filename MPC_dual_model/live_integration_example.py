"""Minimal integration pattern for rov_track_control3.py.

This file does not open sockets or start propellers. Copy the marked calls into
the existing video/control loop after calibration.
"""

from __future__ import annotations

import numpy as np

from camera_transform import camera_to_body_position
from device_adapter import FineSUBThrusterAllocator, ForceCommandAdapter
from fossen_fixed_dl_model import FixedLinearDampingRelativeModel
from mpc_controller import MPCConfig, RelativeMPCController
from mpc_tracker import MPCTracker, build_default_staircase_fusion
from relative_kalman import KalmanConfig, RelativePositionKalmanFilter


def build_tracker() -> MPCTracker:
    model = FixedLinearDampingRelativeModel(
        # TODO: replace every placeholder with pool-identification results.
        M_t=np.diag([20.0, 25.0, 30.0]),
        D_L=np.diag([8.0, 10.0, 12.0]),
        dt=0.05,
        # Capture the real achieved hold force when AUTO is enabled.
        tau_base=np.zeros(3),
    )
    config = MPCConfig(
        horizon=10,
        reference_position=(0.60, 0.0, 0.0),
        force_min=(-20.0, -15.0, -15.0),
        force_max=(20.0, 15.0, 15.0),
        delta_force_min=(-3.0, -2.0, -2.0),
        delta_force_max=(3.0, 2.0, 2.0),
        horizontal_half_fov_deg=42.0,
        vertical_half_fov_deg=30.0,
        fov_margin_deg=5.0,
    )
    estimator = RelativePositionKalmanFilter(
        model,
        KalmanConfig(position_std=(0.015, 0.015, 0.025)),
    )
    controller = RelativeMPCController(model, config)
    adapter = ForceCommandAdapter(
        # TODO: body force that produces the positive device limit.
        positive_force_at_limit=(20.0, 15.0, 15.0),
        # TODO: flip individual signs after a dry test if required.
        signs=(1.0, 1.0, 1.0),
        command_limits=(99.0, 99.0, 45.0),
    )
    fusion = build_default_staircase_fusion()
    allocator = FineSUBThrusterAllocator(
        positive_force_at_limit=(20.0, 15.0, 15.0),
        # True enables the intended depth channel. Set False only to reproduce
        # the checked source line that passed 0.0 into the depth mixer.
        enable_depth=True,
    )
    return MPCTracker(
        model,
        estimator,
        controller,
        adapter,
        fusion=fusion,
        thruster_allocator=allocator,
    )


def one_control_update(
    tracker: MPCTracker,
    position_camera_xyz,
    last_achieved_force_body,
):
    """Call this when a new stereo 3-D position is available."""
    position_body = camera_to_body_position(position_camera_xyz)
    return tracker.update(
        position_body=position_body,
        tau_achieved_previous=last_achieved_force_body,
    )


# Integration outline inside rov_track_control3.py:
#
# tracker = build_tracker()
# last_force_body = np.zeros(3)
#
# When AUTO changes from OFF to ON:
# tracker.latch_baseline(last_force_body)
#
# When stereo returns position_camera = [x_right, y_down, z_forward] in metres:
# output = one_control_update(tracker, position_camera, last_force_body)
# cmd = output.device_command
# motor_throttles = output.thruster_allocation.throttles
# send_xy_2(cmd.planar_forward, cmd.planar_right)
# send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_DEPTH_FORCE, i16le(cmd.depth_force))
# last_force_body = output.mpc.force.copy()  # approximation without force feedback
#
# IMPORTANT: use either the existing high-level cmd path above (the MCU mixes),
# or motor_throttles for a direct eight-ESC path. Do not apply both mixers.
#
# When the target is lost:
# safe = tracker.target_lost(last_force_body)
# send_xy_2(safe.device_command.planar_forward, safe.device_command.planar_right)
# send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_DEPTH_FORCE,
#          i16le(safe.device_command.depth_force))
# last_force_body = safe.force.copy()
