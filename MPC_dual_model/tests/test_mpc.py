import unittest

import numpy as np

from device_adapter import FineSUBThrusterAllocator, ForceCommandAdapter
from dense_qp import QPSolution
from fossen_fixed_dl_model import FixedLinearDampingRelativeModel
from model_fusion import FusionConfig, OnlineModelFusion
from live_integration_example import build_tracker as build_live_tracker
from mpc_controller import MPCConfig, RelativeMPCController
from mpc_tracker import (
    DEFAULT_PREDICTION_HORIZON_WEIGHTS,
    DEFAULT_STAIRCASE_HORIZON_CAPS,
    MPCTracker,
)
from relative_kalman import RelativePositionKalmanFilter


class MPCTest(unittest.TestCase):
    def build(self):
        model = FixedLinearDampingRelativeModel(
            M_t=np.diag([20.0, 25.0, 30.0]),
            D_L=np.diag([8.0, 10.0, 12.0]),
            dt=0.1,
        )
        config = MPCConfig(
            horizon=5,
            reference_position=(0.6, 0.0, 0.0),
            force_min=(-12.0, -10.0, -10.0),
            force_max=(12.0, 10.0, 10.0),
            delta_force_min=(-3.0, -2.0, -2.0),
            delta_force_max=(3.0, 2.0, 2.0),
        )
        return model, RelativeMPCController(model, config)

    def test_force_and_rate_limits(self) -> None:
        model, controller = self.build()
        result = controller.solve(
            state=np.array([1.2, 0.15, -0.10, 0.0, 0.0, 0.0]),
            tau_previous=np.zeros(3),
            tau_base=np.zeros(3),
        )
        self.assertFalse(result.used_fallback, result.status)
        self.assertTrue(np.all(result.force <= [3.0, 2.0, 2.0] + np.ones(3) * 1e-9))
        self.assertTrue(np.all(result.force >= [-3.0, -2.0, -2.0] - np.ones(3) * 1e-9))
        self.assertEqual(result.predicted_states.shape, (5, 6))
        self.assertEqual(result.slacks.shape, (5, 6))
        self.assertGreaterEqual(float(np.min(result.slacks)), -2e-5)

    def test_high_level_tracker(self) -> None:
        model, controller = self.build()
        tracker = MPCTracker(
            model=model,
            estimator=RelativePositionKalmanFilter(model),
            controller=controller,
            adapter=ForceCommandAdapter(
                positive_force_at_limit=(12.0, 10.0, 10.0),
                command_limits=(99.0, 99.0, 45.0),
            ),
            thruster_allocator=FineSUBThrusterAllocator(
                positive_force_at_limit=(12.0, 10.0, 10.0)
            ),
        )
        output = tracker.update(
            position_body=(1.0, 0.1, -0.05),
            tau_achieved_previous=(0.0, 0.0, 0.0),
        )
        self.assertEqual(output.estimated_state.shape, (6,))
        self.assertLessEqual(abs(output.device_command.planar_forward), 99)
        self.assertLessEqual(abs(output.device_command.planar_right), 99)
        self.assertLessEqual(abs(output.device_command.depth_force), 45)
        self.assertIsNotNone(output.thruster_allocation)
        self.assertEqual(output.thruster_allocation.throttles.shape, (8,))
        self.assertEqual(output.estimated_state.shape, (6,))
        self.assertFalse(hasattr(output.mpc, "yaw_moment"))

    def test_default_tracker_uses_staircase_translation_history(self) -> None:
        model, controller = self.build()
        tracker = MPCTracker(
            model=model,
            estimator=RelativePositionKalmanFilter(model),
            controller=controller,
            adapter=ForceCommandAdapter(
                positive_force_at_limit=(12.0, 10.0, 10.0),
                command_limits=(99.0, 99.0, 45.0),
            ),
        )
        for _ in range(4):
            tracker.update((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        # At k=3, the three-step cell 3|0 is outside cap(age=3)=2.
        self.assertNotIn((0, 3), tracker.fusion.active_pairs)
        self.assertIn((0, 2), tracker.fusion.active_pairs)
        self.assertEqual(
            tracker.fusion.config.staircase_horizon_caps,
            DEFAULT_STAIRCASE_HORIZON_CAPS,
        )
        self.assertEqual(tracker.estimator.x.shape, (6,))

    def test_live_builder_uses_same_six_level_staircase(self) -> None:
        tracker = build_live_tracker()
        self.assertEqual(
            tracker.fusion.config.staircase_horizon_caps,
            DEFAULT_STAIRCASE_HORIZON_CAPS,
        )
        self.assertEqual(
            tracker.fusion.config.prediction_horizon_weights,
            DEFAULT_PREDICTION_HORIZON_WEIGHTS,
        )
        self.assertEqual(tracker.fusion.config.window, 6)
        self.assertEqual(tracker.fusion.config.prediction_horizon, 3)

    def test_solver_failure_returns_to_latched_baseline(self) -> None:
        model, controller = self.build()
        model.set_tau_base((5.0, 1.0, -1.0))

        class AlwaysFailSolver:
            backend_name = "test_failure"

            def solve(self, P, q, A, lower, upper, warm_start=None):
                return QPSolution(
                    x=np.zeros_like(q),
                    status="time_limit",
                    iterations=0,
                    objective=np.inf,
                    primal_residual=np.inf,
                    dual_residual=np.inf,
                )

        controller.solver = AlwaysFailSolver()
        result = controller.solve(
            state=np.array([1.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
            tau_previous=np.zeros(3),
        )
        self.assertTrue(result.used_fallback)
        self.assertEqual(result.status, "fallback:baseline:time_limit")
        # Forward baseline is rate-limited from 0 to +3 N on this first step.
        np.testing.assert_allclose(result.force, (3.0, 1.0, -1.0))

    def test_solver_failure_holds_last_feasible_force(self) -> None:
        _, controller = self.build()
        solved = controller.solve(
            state=np.array([1.0, 0.05, 0.0, 0.0, 0.0, 0.0]),
            tau_previous=np.zeros(3),
        )
        last_feasible = solved.force.copy()

        class AlwaysFailSolver:
            backend_name = "test_failure"

            def solve(self, P, q, A, lower, upper, warm_start=None):
                return QPSolution(
                    x=np.zeros_like(q),
                    status="maximum_iterations",
                    iterations=0,
                    objective=np.inf,
                    primal_residual=np.inf,
                    dual_residual=np.inf,
                )

        controller.solver = AlwaysFailSolver()
        failed = controller.solve(
            state=np.array([1.0, 0.05, 0.0, 0.0, 0.0, 0.0]),
            tau_previous=last_feasible,
        )
        self.assertEqual(
            failed.status,
            "fallback:last_feasible:maximum_iterations",
        )
        np.testing.assert_allclose(failed.force, last_feasible)

    def test_tracker_scores_triangular_multi_step_position_history(self) -> None:
        model, controller = self.build()
        fusion = OnlineModelFusion(
            FusionConfig(window=3, prediction_horizon=2)
        )
        tracker = MPCTracker(
            model=model,
            estimator=RelativePositionKalmanFilter(model),
            controller=controller,
            adapter=ForceCommandAdapter(
                positive_force_at_limit=(12.0, 10.0, 10.0),
                command_limits=(99.0, 99.0, 45.0),
            ),
            fusion=fusion,
        )

        tracker.update((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(fusion.sample_count, 0)
        tracker.update((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(fusion.sample_count, 1)  # start i=0, h=1
        tracker.update((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        # start i=0, h=2 and start i=1, h=1 both complete now.
        self.assertEqual(fusion.sample_count, 3)


if __name__ == "__main__":
    unittest.main()
