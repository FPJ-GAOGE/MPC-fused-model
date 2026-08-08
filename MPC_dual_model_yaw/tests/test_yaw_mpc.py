import unittest

import numpy as np

from MPC_dual_model.dense_qp import QPSolverSettings
from MPC_dual_model.fossen_fixed_dl_model import FixedLinearDampingRelativeModel
from MPC_dual_model_yaw.yaw_controller import (
    YawControlConfig,
    YawStateController,
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
        yaw = LinearYawDynamics(2.0, 0.8, translation.dt)
        model = RotationAwareRelativeModel(translation, yaw)
        controller = RotationAwareMPCController(
            model,
            YawMPCConfig(
                horizon=5,
                reference_position=(1.0, 0.0, 0.0),
                position_weights=(5.0, 5.0, 5.0),
                force_min=(-30.0, -30.0, -30.0),
                force_max=(30.0, 30.0, 30.0),
                delta_force_min=(-10.0, -10.0, -10.0),
                delta_force_max=(10.0, 10.0, 10.0),
                solver_settings=QPSolverSettings(
                    backend="numpy_admm",
                    rho=10.0,
                    max_iterations=5000,
                    time_limit_seconds=2.0,
                ),
            ),
        )
        yaw_controller = YawStateController(
            yaw,
            YawControlConfig(
                alpha_on=0.10,
                alpha_off=0.05,
                alpha_emergency=0.60,
                trigger_frames=1,
                settle_frames=2,
                omega_command_acceleration_max=10.0,
                yaw_moment_min=-2.0,
                yaw_moment_max=2.0,
                delta_yaw_moment_min=-0.5,
                delta_yaw_moment_max=0.5,
            ),
        )
        return model, controller, yaw_controller

    def solve(self, position):
        _, controller, yaw_controller = self.build()
        alpha = float(np.arctan2(position[1], position[0]))
        yaw_control = yaw_controller.update(0.0, 0.0, alpha, 0.0, 5)
        result = controller.solve(
            state=np.concatenate((position, np.zeros(3))),
            force_previous=np.zeros(3),
            yaw_prediction=yaw_control.prediction,
            model1_weight=(0.8, 0.8, 0.8),
        )
        return controller, yaw_control, result

    def test_yaw_is_generated_outside_translation_qp(self) -> None:
        _, yaw_control, result = self.solve(np.array([1.0, 0.35, 0.0]))
        self.assertGreater(yaw_control.yaw_moment, 0.0)
        self.assertAlmostEqual(result.yaw_moment, yaw_control.yaw_moment)
        self.assertEqual(result.force_sequence.shape, (5, 3))
        self.assertEqual(result.delta_force_sequence.shape, (5, 3))
        self.assertEqual(result.predicted_states.shape, (5, 6))
        self.assertEqual(result.frozen_yaw_rates.shape, (6,))
        self.assertEqual(result.frozen_delta_yaw.shape, (5,))

    def test_centred_target_has_no_yaw_bias(self) -> None:
        _, yaw_control, result = self.solve(np.array([1.0, 0.0, 0.0]))
        self.assertAlmostEqual(yaw_control.yaw_moment, 0.0, places=12)
        self.assertAlmostEqual(result.yaw_moment, 0.0, places=12)

    def test_frozen_yaw_uses_trapezoidal_angle_increment(self) -> None:
        _, yaw_control, result = self.solve(np.array([1.0, 0.35, 0.0]))
        expected = 0.5 * 0.1 * (
            result.frozen_yaw_rates[:-1] + result.frozen_yaw_rates[1:]
        )
        np.testing.assert_allclose(result.frozen_delta_yaw, expected, atol=1.0e-12)
        np.testing.assert_allclose(
            result.frozen_yaw_moments,
            yaw_control.prediction.moments,
            atol=0.0,
        )

if __name__ == "__main__":
    unittest.main()
