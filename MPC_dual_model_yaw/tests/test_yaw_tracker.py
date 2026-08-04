import unittest

import numpy as np

from MPC_dual_model.dense_qp import QPSolverSettings
from MPC_dual_model.device_adapter import (
    FineSUBThrusterAllocator,
    ForceCommandAdapter,
)
from MPC_dual_model.fossen_fixed_dl_model import (
    FixedLinearDampingRelativeModel,
)
from MPC_dual_model.model_fusion import (
    FusionConfig,
    OnlineModelFusion,
)
from MPC_dual_model_yaw.yaw_kalman import (
    RotationAwareKalmanFilter,
)
from MPC_dual_model_yaw.yaw_mpc_controller import (
    RotationAwareMPCController,
    YawMPCConfig,
)
from MPC_dual_model_yaw.yaw_relative_model import (
    LinearYawDynamics,
    RotationAwareRelativeModel,
    rotation_body_from_previous,
)
from MPC_dual_model_yaw.yaw_tracker import (
    DEFAULT_PREDICTION_HORIZON_WEIGHTS,
    DEFAULT_STAIRCASE_HORIZON_CAPS,
    RotationAwareMPCTracker,
    YawMomentChannelAdapter,
    build_default_staircase_fusion,
)


class YawTrackerTest(unittest.TestCase):
    def build(self):
        translation = FixedLinearDampingRelativeModel(
            np.diag([20.0, 25.0, 30.0]),
            np.diag([8.0, 10.0, 12.0]),
            0.05,
        )
        model = RotationAwareRelativeModel(
            translation,
            LinearYawDynamics(2.5, 1.2, translation.dt),
        )
        controller = RotationAwareMPCController(
            model,
            YawMPCConfig(
                horizon=4,
                solver_settings=QPSolverSettings(
                    backend="numpy_admm",
                    rho=10.0,
                    max_iterations=3000,
                    time_limit_seconds=1.0,
                ),
            ),
        )
        fusion = OnlineModelFusion(
            FusionConfig(window=3, prediction_horizon=2)
        )
        tracker = RotationAwareMPCTracker(
            model=model,
            estimator=RotationAwareKalmanFilter(model),
            controller=controller,
            force_adapter=ForceCommandAdapter(),
            yaw_adapter=YawMomentChannelAdapter(
                positive_yaw_moment_at_limit=4.0,
                channel_limit=0.2,
            ),
            fusion=fusion,
            thruster_allocator=FineSUBThrusterAllocator(deadband=0.0),
        )
        return tracker, fusion

    def test_default_fusion_matches_translation_staircase_definition(self) -> None:
        fusion = build_default_staircase_fusion()
        self.assertEqual(fusion.config.window, 6)
        self.assertEqual(fusion.config.prediction_horizon, 3)
        self.assertEqual(
            fusion.config.staircase_horizon_caps,
            DEFAULT_STAIRCASE_HORIZON_CAPS,
        )
        self.assertEqual(
            fusion.config.prediction_horizon_weights,
            DEFAULT_PREDICTION_HORIZON_WEIGHTS,
        )

    def test_rotating_tracker_keeps_and_masks_exact_staircase_cells(self) -> None:
        tracker, _ = self.build()
        fusion = build_default_staircase_fusion()
        tracker.fusion = fusion
        for _ in range(4):
            tracker.update(
                position_body=(1.0, 0.0, 0.0),
                yaw_rad=0.0,
                yaw_rate_rad_s=0.0,
                force_achieved_previous=(0.0, 0.0, 0.0),
                yaw_moment_achieved_previous=0.0,
            )

        # At k=3, origin t0=0 has r=3 and H_cap(3)=2.
        self.assertIn((0, 1), fusion.active_pairs)   # k-2 | k-3
        self.assertIn((0, 2), fusion.active_pairs)   # k-1 | k-3
        self.assertNotIn((0, 3), fusion.active_pairs)  # k | k-3
        self.assertTrue(
            all(origin != target for origin, target in fusion.active_pairs)
        )  # h=0 is never scored.

    def test_active_staircase_cell_weights_are_renormalized(self) -> None:
        fusion = build_default_staircase_fusion()
        errors = (0.1, 0.2, 0.4)
        for target, error in enumerate(errors, start=1):
            fusion.observe_position(
                actual_position=(0.0, 0.0, 0.0),
                prediction1=(-error, 0.0, 0.0),
                prediction2=(0.0, 0.0, 0.0),
                horizon_step=target,
                origin_index=0,
                target_index=target,
            )

        # At current k=3 the h=3 cell is masked. Remaining raw weights are
        # 0.8^2*0.5=0.32 and 0.8*0.3=0.24, normalized by their sum 0.56.
        expected_m1 = (0.32 * 0.1**2 + 0.24 * 0.2**2) / 0.56
        self.assertEqual(fusion.active_pairs, ((0, 1), (0, 2)))
        self.assertAlmostEqual(fusion.M1[0], expected_m1, places=12)

    def test_actual_yaw_is_used_in_filter_and_fusion_history(self) -> None:
        tracker, fusion = self.build()
        position0 = np.array([1.0, 0.20, 0.0])
        tracker.update(position0, 0.0, 0.0, np.zeros(3), 0.0)

        delta_yaw = 0.04
        position1 = rotation_body_from_previous(delta_yaw) @ position0
        output = tracker.update(
            position1,
            delta_yaw,
            delta_yaw / tracker.model.dt,
            np.zeros(3),
            0.0,
        )
        self.assertLess(np.linalg.norm(output.estimated_state[3:]), 1.0e-10)
        self.assertEqual(fusion.sample_count, 1)

    def test_yaw_moment_reaches_finesub_lower_thrusters(self) -> None:
        tracker, _ = self.build()
        output = tracker.update(
            position_body=(1.0, 0.35, 0.0),
            yaw_rad=0.0,
            yaw_rate_rad_s=0.0,
            force_achieved_previous=(0.0, 0.0, 0.0),
            yaw_moment_achieved_previous=0.0,
        )
        self.assertGreater(output.mpc.yaw_moment, 0.0)
        self.assertGreater(output.yaw_channel, 0.0)
        self.assertIsNotNone(output.thruster_allocation)
        self.assertAlmostEqual(
            output.thruster_allocation.attitude_channels[2],
            output.yaw_channel,
        )
        self.assertTrue(
            np.any(
                np.abs(output.thruster_allocation.throttles[[0, 3, 4, 7]])
                > 0.0
            )
        )


if __name__ == "__main__":
    unittest.main()
