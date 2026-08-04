import unittest

import numpy as np

from device_adapter import FineSUBThrusterAllocator
from fossen_fixed_dl_model import FixedLinearDampingRelativeModel
from model_fusion import FusionConfig, OnlineModelFusion
from mpc_controller import MPCConfig, RelativeMPCController


class FusionAndAllocationTest(unittest.TestCase):
    def test_fusion_prefers_better_model_and_stays_bounded(self) -> None:
        fusion = OnlineModelFusion(
            FusionConfig(minimum_weight=0.05, initial_model1_weight=(0.8, 0.8, 0.8))
        )
        weight = fusion.observe_position(
            actual_position=(0.0, 0.0, 0.0),
            prediction1=(0.0, 0.0, 0.0),
            prediction2=(0.5, -0.4, 0.3),
            horizon_step=1,
        )
        np.testing.assert_allclose(weight, np.full(3, 0.95))
        self.assertTrue(np.allclose(fusion.model1_weight + fusion.model2_weight, 1.0))

    def test_position_history_keeps_completed_multi_step_scores(self) -> None:
        fusion = OnlineModelFusion(
            FusionConfig(
                window=2,
                prediction_horizon=3,
                horizon_weight_decay=0.8,
            )
        )
        for step in range(8):
            fusion.observe_position(
                actual_position=(0.0, 0.0, 0.0),
                prediction1=(0.0, 0.0, 0.0),
                prediction2=(0.2, 0.0, 0.0),
                horizon_step=step % 3 + 1,
            )
        # Two historical start times, each retaining three horizons.
        self.assertEqual(fusion.sample_count, 6)
        # Only x differs between the two candidate predictions.  Identical
        # y/z candidates are intentionally neutral rather than favouring one.
        self.assertGreater(fusion.model1_weight[0], fusion.model2_weight[0])
        np.testing.assert_allclose(fusion.model1_weight[1:], (0.5, 0.5))

    def test_staircase_history_masks_old_origin_long_prediction(self) -> None:
        fusion = OnlineModelFusion(
            FusionConfig(
                window=6,
                prediction_horizon=3,
                forgetting_factor=0.8,
                prediction_horizon_weights=(0.5, 0.3, 0.2),
                staircase_horizon_caps=(3, 3, 2, 2, 1, 1),
            )
        )
        for target in (1, 2, 3):
            fusion.observe_position(
                actual_position=(0.0, 0.0, 0.0),
                prediction1=(0.0, 0.0, 0.0),
                prediction2=(0.2, 0.0, 0.0),
                horizon_step=target,
                origin_index=0,
                target_index=target,
            )
        fusion.advance_time(3)
        self.assertIn((0, 1), fusion.active_pairs)
        self.assertIn((0, 2), fusion.active_pairs)
        self.assertNotIn((0, 3), fusion.active_pairs)  # k|k-3 is masked.
        self.assertEqual(fusion.sample_count, 2)
        self.assertGreater(fusion.model1_weight[0], fusion.model2_weight[0])

    def test_augmented_prediction_has_rolling_previous_force(self) -> None:
        model = FixedLinearDampingRelativeModel(
            np.diag([20.0, 25.0, 30.0]),
            np.diag([8.0, 10.0, 12.0]),
            0.1,
        )
        controller = RelativeMPCController(model, MPCConfig(horizon=2))
        z0 = np.concatenate((np.zeros(6), np.array([2.0, 0.0, 0.0])))
        controller._build_prediction_matrices(np.ones(3))
        model1_free = (controller.Sx @ z0).reshape(2, 6)[0]
        controller._build_prediction_matrices(np.zeros(3))
        model2_free = (controller.Sx @ z0).reshape(2, 6)[0]
        np.testing.assert_allclose(model1_free, -model.B_d @ [2.0, 0.0, 0.0])
        np.testing.assert_allclose(model2_free, np.zeros(6))

    def test_effective_force_cost_matches_each_candidate_model(self) -> None:
        model = FixedLinearDampingRelativeModel(
            np.diag([20.0, 25.0, 30.0]),
            np.diag([8.0, 10.0, 12.0]),
            0.1,
        )
        controller = RelativeMPCController(model, MPCConfig(horizon=3))

        controller._build_prediction_matrices(np.ones(3))
        np.testing.assert_allclose(
            controller.effective_force_matrix,
            controller.rate_matrix,
        )

        controller._build_prediction_matrices(np.zeros(3))
        np.testing.assert_allclose(
            controller.effective_force_matrix,
            np.eye(9),
        )

    def test_finesub_forward_mixer_and_motor_order(self) -> None:
        allocator = FineSUBThrusterAllocator(deadband=0.0)
        allocation = allocator.allocate((20.0, 0.0, 0.0))
        expected = np.array([-0.35, 0.0, 0.0, -0.35, 0.35, 0.0, 0.0, 0.35])
        np.testing.assert_allclose(allocation.throttles, expected)

    def test_finesub_attitude_and_depth_use_upper_group(self) -> None:
        allocator = FineSUBThrusterAllocator(deadband=0.0)
        allocation = allocator.allocate(
            (0.0, 0.0, 15.0), attitude_control=(0.1, -0.1, 0.0)
        )
        self.assertTrue(np.allclose(allocation.throttles[[0, 3, 4, 7]], 0.0))
        self.assertTrue(np.any(np.abs(allocation.throttles[[1, 2, 5, 6]]) > 0.0))


if __name__ == "__main__":
    unittest.main()
