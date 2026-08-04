import unittest

import numpy as np

from MPC_dual_model.dense_qp import QPSolverSettings
from MPC_dual_model.fossen_fixed_dl_model import (
    FixedLinearDampingRelativeModel,
)
from MPC_dual_model_yaw.yaw_mpc_controller import (
    RotationAwareMPCController,
    YawMPCConfig,
)
from MPC_dual_model_yaw.yaw_relative_model import (
    LinearYawDynamics,
    RotationAwareRelativeModel,
)


class YawMPCTest(unittest.TestCase):
    def build(self):
        translation = FixedLinearDampingRelativeModel(
            np.diag([20.0, 25.0, 30.0]),
            np.diag([8.0, 10.0, 12.0]),
            0.1,
        )
        model = RotationAwareRelativeModel(
            translation,
            LinearYawDynamics(2.0, 0.8, translation.dt),
        )
        config = YawMPCConfig(
            horizon=5,
            reference_position=(1.0, 0.0, 0.0),
            position_weights=(5.0, 5.0, 5.0),
            line_of_sight_angle_weight=300.0,
            force_min=(-0.05, -0.05, -0.05),
            force_max=(0.05, 0.05, 0.05),
            delta_force_min=(-0.05, -0.05, -0.05),
            delta_force_max=(0.05, 0.05, 0.05),
            yaw_moment_min=-2.0,
            yaw_moment_max=2.0,
            delta_yaw_moment_min=-0.5,
            delta_yaw_moment_max=0.5,
            solver_settings=QPSolverSettings(
                backend="numpy_admm",
                rho=10.0,
                max_iterations=3000,
                time_limit_seconds=1.0,
            ),
        )
        return model, RotationAwareMPCController(model, config)

    def test_target_to_right_commands_positive_yaw_moment(self) -> None:
        _, controller = self.build()
        result = controller.solve(
            state=np.array([1.0, 0.35, 0.0, 0.0, 0.0, 0.0]),
            yaw_rate=0.0,
            force_previous=np.zeros(3),
            yaw_moment_previous=0.0,
            model1_weight=(0.8, 0.8, 0.8),
        )
        self.assertFalse(result.used_fallback, result.status)
        self.assertGreater(result.yaw_moment, 0.0)
        self.assertLessEqual(result.yaw_moment, 0.5 + 1.0e-8)
        self.assertEqual(result.input_sequence.shape, (5, 4))
        self.assertEqual(result.predicted_states.shape, (5, 8))
        self.assertEqual(result.slacks.shape, (5, 6))

    def test_centred_target_does_not_create_yaw_bias(self) -> None:
        _, controller = self.build()
        result = controller.solve(
            state=np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            yaw_rate=0.0,
            force_previous=np.zeros(3),
            yaw_moment_previous=0.0,
        )
        self.assertFalse(result.used_fallback, result.status)
        self.assertAlmostEqual(result.yaw_moment, 0.0, places=7)

    def test_yaw_rate_and_moment_constraints_are_applied(self) -> None:
        _, controller = self.build()
        result = controller.solve(
            state=np.array([1.0, -0.4, 0.0, 0.0, 0.0, 0.0]),
            yaw_rate=0.0,
            force_previous=np.zeros(3),
            yaw_moment_previous=0.0,
        )
        self.assertGreaterEqual(result.yaw_moment, -0.5 - 1.0e-8)
        self.assertLessEqual(result.yaw_moment, 0.5 + 1.0e-8)
        self.assertTrue(
            np.all(result.predicted_states[:, 7] <= controller.config.yaw_rate_max + 1.0e-4)
        )
        self.assertTrue(
            np.all(result.predicted_states[:, 7] >= controller.config.yaw_rate_min - 1.0e-4)
        )


if __name__ == "__main__":
    unittest.main()
