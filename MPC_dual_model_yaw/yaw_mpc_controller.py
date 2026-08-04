"""Rotation-aware dual-model QP MPC with translational force and yaw moment."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from MPC_dual_model.dense_qp import (
    QPSolverSettings,
    create_qp_solver,
)
from MPC_dual_model.fossen_fixed_dl_model import vector3
from MPC_dual_model.mpc_controller import _block_diagonal

from .yaw_relative_model import (
    RotationAwareRelativeModel,
    finite_scalar,
    line_of_sight_angle,
)


Array = np.ndarray


def _positive_vector3(value, name: str) -> Array:
    result = vector3(value, name)
    if np.any(result < 0.0):
        raise ValueError(f"{name} must be nonnegative")
    return result


def _line_of_sight_rate_and_jacobian(state: Array, epsilon: float) -> tuple[float, Array]:
    """Linearize (px*vy-py*vx)/(px^2+py^2+epsilon)."""
    px, py, _, vx, vy, _ = state
    numerator = px * vy - py * vx
    denominator = px * px + py * py + epsilon
    value = numerator / denominator
    jacobian = np.zeros(6)
    jacobian[0] = (vy * denominator - 2.0 * px * numerator) / denominator**2
    jacobian[1] = (-vx * denominator - 2.0 * py * numerator) / denominator**2
    jacobian[3] = -py / denominator
    jacobian[4] = px / denominator
    return float(value), jacobian


@dataclass
class YawMPCConfig:
    """MPC weights and constraints; force is N, yaw moment is N*m."""

    horizon: int = 10
    reference_position: object = (0.60, 0.0, 0.0)

    position_weights: object = (50.0, 80.0, 100.0)
    velocity_weights: object = (8.0, 10.0, 12.0)
    line_of_sight_angle_weight: float = 80.0
    yaw_rate_weight: float = 8.0
    terminal_weight_scale: float = 4.0

    force_weights: object = (0.04, 0.06, 0.06)
    yaw_moment_weight: float = 0.20
    delta_force_weights: object = (0.8, 1.0, 1.0)
    delta_yaw_moment_weight: float = 2.0

    force_min: object = (-20.0, -15.0, -15.0)
    force_max: object = (20.0, 15.0, 15.0)
    delta_force_min: object = (-4.0, -3.0, -3.0)
    delta_force_max: object = (4.0, 3.0, 3.0)
    yaw_moment_min: float = -4.0
    yaw_moment_max: float = 4.0
    delta_yaw_moment_min: float = -0.8
    delta_yaw_moment_max: float = 0.8
    yaw_rate_min: float = -np.deg2rad(45.0)
    yaw_rate_max: float = np.deg2rad(45.0)

    forward_distance_min: float = 0.25
    forward_distance_max: float = 1.50
    horizontal_half_fov_deg: float = 42.0
    vertical_half_fov_deg: float = 30.0
    fov_margin_deg: float = 5.0
    line_of_sight_epsilon: float = 1.0e-5

    slack_quadratic_weight: float = 2.0e4
    slack_linear_weight: float = 50.0
    slack_max: float = 5.0

    solver_settings: QPSolverSettings = field(
        default_factory=lambda: QPSolverSettings(
            # The 4-input yaw problem is too large for the portable NumPy
            # fallback at 20 Hz.  Fail during construction if OSQP is absent
            # instead of silently timing out in the control loop.
            backend="osqp",
            rho=10.0,
            sigma=1.0e-8,
            max_iterations=1800,
            absolute_tolerance=2.0e-5,
            relative_tolerance=3.0e-4,
        )
    )

    def normalized(self) -> "YawMPCConfig":
        if self.horizon < 1:
            raise ValueError("horizon must be at least 1")
        self.reference_position = vector3(self.reference_position, "reference_position")
        self.position_weights = _positive_vector3(
            self.position_weights, "position_weights"
        )
        self.velocity_weights = _positive_vector3(
            self.velocity_weights, "velocity_weights"
        )
        self.force_weights = _positive_vector3(self.force_weights, "force_weights")
        self.delta_force_weights = _positive_vector3(
            self.delta_force_weights, "delta_force_weights"
        )
        self.force_min = vector3(self.force_min, "force_min")
        self.force_max = vector3(self.force_max, "force_max")
        self.delta_force_min = vector3(self.delta_force_min, "delta_force_min")
        self.delta_force_max = vector3(self.delta_force_max, "delta_force_max")
        scalar_nonnegative = (
            "line_of_sight_angle_weight",
            "yaw_rate_weight",
            "yaw_moment_weight",
            "delta_yaw_moment_weight",
        )
        for name in scalar_nonnegative:
            value = finite_scalar(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} must be nonnegative")
            setattr(self, name, value)
        if np.any(self.force_min >= self.force_max):
            raise ValueError("force_min must be smaller than force_max")
        if np.any(self.delta_force_min >= self.delta_force_max):
            raise ValueError("delta_force_min must be smaller than delta_force_max")
        if self.yaw_moment_min >= self.yaw_moment_max:
            raise ValueError("invalid yaw moment limits")
        if self.delta_yaw_moment_min >= self.delta_yaw_moment_max:
            raise ValueError("invalid yaw moment rate limits")
        if self.yaw_rate_min >= self.yaw_rate_max:
            raise ValueError("invalid yaw rate limits")
        if not 0.0 < self.forward_distance_min < self.forward_distance_max:
            raise ValueError("invalid forward distance limits")
        if not 0.0 < self.fov_margin_deg < min(
            self.horizontal_half_fov_deg, self.vertical_half_fov_deg
        ):
            raise ValueError("invalid FOV margin")
        if self.line_of_sight_epsilon <= 0.0:
            raise ValueError("line_of_sight_epsilon must be positive")
        if self.terminal_weight_scale <= 0.0:
            raise ValueError("terminal_weight_scale must be positive")
        if self.slack_quadratic_weight <= 0.0 or self.slack_linear_weight < 0.0:
            raise ValueError("invalid slack weights")
        if self.slack_max <= 0.0:
            raise ValueError("slack_max must be positive")
        return self


@dataclass
class YawMPCResult:
    force: Array
    yaw_moment: float
    input_sequence: Array
    predicted_states: Array
    frozen_yaw_rates: Array
    slacks: Array
    status: str
    iterations: int
    objective: float
    used_fallback: bool
    model1_weight: Array
    model2_weight: Array


class RotationAwareMPCController:
    """Frozen-rotation, linear-time-varying MPC.

    State reported to the user is y=[p(3), v(3), alpha, r].  The optimizer
    augments it with previous translation force and previous yaw moment.  The
    rotation matrices use the prior MPC trajectory, while alpha dynamics are
    linearized around that same trajectory.  This keeps each solve a convex QP.
    """

    OUTPUT_DIM = 8
    AUGMENTED_DIM = 12
    INPUT_DIM = 4
    SLACKS_PER_STEP = 6

    def __init__(
        self,
        model: RotationAwareRelativeModel,
        config: YawMPCConfig | None = None,
    ) -> None:
        self.model = model
        self.config = (config or YawMPCConfig()).normalized()
        self.solver = create_qp_solver(self.config.solver_settings)
        self._warm_start = None
        self._last_feasible_input = None
        self._nominal_prediction = None
        self._prediction_weight = np.ones(3)

    def reset(self) -> None:
        self._warm_start = None
        self._last_feasible_input = None
        self._nominal_prediction = None

    def safe_input(
        self,
        force_previous,
        yaw_moment_previous: float,
        force_target=None,
        yaw_moment_target: float | None = None,
    ) -> tuple[Array, float]:
        previous = np.concatenate(
            (
                vector3(force_previous, "force_previous"),
                [finite_scalar(yaw_moment_previous, "yaw_moment_previous")],
            )
        )
        target = np.concatenate(
            (
                self.model.translation.tau_base
                if force_target is None
                else vector3(force_target, "force_target"),
                [
                    self.model.yaw.yaw_moment_base
                    if yaw_moment_target is None
                    else finite_scalar(yaw_moment_target, "yaw_moment_target")
                ],
            )
        )
        safe = self._safe_fallback(previous, target)
        return safe[:3], float(safe[3])

    def _nominal_state(self, step: int, current_state: Array, current_rate: float) -> Array:
        if step == 0 or self._nominal_prediction is None:
            return np.concatenate(
                (current_state, [line_of_sight_angle(current_state[:3]), current_rate])
            )
        index = min(step, self._nominal_prediction.shape[0] - 1)
        return self._nominal_prediction[index].copy()

    def _build_prediction_matrices(
        self,
        current_state: Array,
        current_rate: float,
        model1_weight: Array,
    ) -> None:
        cfg = self.config
        N = cfg.horizon
        nz = self.AUGMENTED_DIM
        nu = self.INPUT_DIM
        ny = self.OUTPUT_DIM
        output = np.hstack((np.eye(ny), np.zeros((ny, nz - ny))))
        weight6 = np.diag(np.concatenate((model1_weight, model1_weight)))

        transitions: list[Array] = []
        inputs: list[Array] = []
        constants: list[Array] = []
        frozen_rates = np.zeros(N)

        for step in range(N):
            nominal = self._nominal_state(step, current_state, current_rate)
            nominal_translation = nominal[:6]
            nominal_rate = current_rate if step == 0 else float(nominal[7])
            frozen_rates[step] = nominal_rate
            delta_yaw = self.model.dt * nominal_rate
            translation_A, translation_B = self.model.rotation_aware_matrices(
                delta_yaw
            )

            A = np.zeros((nz, nz))
            B = np.zeros((nz, nu))
            c = np.zeros(nz)

            A[:6, :6] = translation_A
            A[:6, 8:11] = -weight6 @ translation_B
            B[:6, :3] = translation_B

            los_rate, los_jacobian = _line_of_sight_rate_and_jacobian(
                nominal_translation, cfg.line_of_sight_epsilon
            )
            A[6, :6] = self.model.dt * los_jacobian
            A[6, 6] = 1.0
            A[6, 7] = -self.model.dt
            c[6] = self.model.dt * (
                los_rate - los_jacobian @ nominal_translation
            )

            A[7, 7] = self.model.yaw.a_r
            B[7, 3] = self.model.yaw.b_r
            B[8:11, :3] = np.eye(3)
            B[11, 3] = 1.0

            transitions.append(A)
            inputs.append(B)
            constants.append(c)

        self.Sx = np.zeros((N * ny, nz))
        self.Su = np.zeros((N * ny, N * nu))
        self.Sc = np.zeros(N * ny)
        state_gain = np.eye(nz)
        input_gain = np.zeros((nz, N * nu))
        affine = np.zeros(nz)
        for step, (A, B, c) in enumerate(zip(transitions, inputs, constants)):
            state_gain = A @ state_gain
            input_gain = A @ input_gain
            input_gain[:, step * nu : (step + 1) * nu] += B
            affine = A @ affine + c
            rows = slice(step * ny, (step + 1) * ny)
            self.Sx[rows] = output @ state_gain
            self.Su[rows] = output @ input_gain
            self.Sc[rows] = output @ affine

        self.rate_matrix = np.zeros((N * nu, N * nu))
        self.effective_force_matrix = np.zeros((3 * N, N * nu))
        self.yaw_moment_selector = np.zeros((N, N * nu))
        input_weight = np.diag(model1_weight)
        for step in range(N):
            input_slice = slice(step * nu, (step + 1) * nu)
            self.rate_matrix[input_slice, input_slice] = np.eye(nu)
            force_rows = slice(step * 3, (step + 1) * 3)
            self.effective_force_matrix[force_rows, step * nu : step * nu + 3] = np.eye(3)
            self.yaw_moment_selector[step, step * nu + 3] = 1.0
            if step > 0:
                self.rate_matrix[
                    input_slice, (step - 1) * nu : step * nu
                ] = -np.eye(nu)
                self.effective_force_matrix[
                    force_rows, (step - 1) * nu : (step - 1) * nu + 3
                ] = -input_weight
        self.frozen_yaw_rates = frozen_rates
        self._prediction_weight = model1_weight.copy()

    def _cost(
        self,
        free_prediction: Array,
        reference_position: Array,
        previous_input: Array,
    ) -> tuple[Array, Array]:
        cfg = self.config
        N = cfg.horizon
        stage_diagonal = np.concatenate(
            (
                cfg.position_weights,
                cfg.velocity_weights,
                [cfg.line_of_sight_angle_weight, cfg.yaw_rate_weight],
            )
        )
        Q_stage = np.diag(stage_diagonal)
        Q_terminal = cfg.terminal_weight_scale * Q_stage
        Q_bar = _block_diagonal(
            [Q_terminal if step == N - 1 else Q_stage for step in range(N)]
        )
        reference_state = np.concatenate((reference_position, np.zeros(5)))
        reference = np.tile(reference_state, N)
        prediction_error = free_prediction - reference

        force_cost = np.kron(np.eye(N), np.diag(cfg.force_weights))
        rate_weights = np.concatenate(
            (cfg.delta_force_weights, [cfg.delta_yaw_moment_weight])
        )
        rate_cost = np.kron(np.eye(N), np.diag(rate_weights))

        rate_reference = np.zeros(N * self.INPUT_DIM)
        rate_reference[: self.INPUT_DIM] = previous_input
        effective_reference = np.zeros(3 * N)
        effective_reference[:3] = self._prediction_weight * previous_input[:3]

        input_hessian = (
            self.Su.T @ Q_bar @ self.Su
            + self.effective_force_matrix.T
            @ force_cost
            @ self.effective_force_matrix
            + cfg.yaw_moment_weight
            * self.yaw_moment_selector.T
            @ self.yaw_moment_selector
            + self.rate_matrix.T @ rate_cost @ self.rate_matrix
        )
        input_gradient = (
            self.Su.T @ Q_bar @ prediction_error
            - self.effective_force_matrix.T @ force_cost @ effective_reference
            - self.rate_matrix.T @ rate_cost @ rate_reference
        )

        n_input = N * self.INPUT_DIM
        n_slack = N * self.SLACKS_PER_STEP
        P = np.zeros((n_input + n_slack, n_input + n_slack))
        q = np.zeros(n_input + n_slack)
        P[:n_input, :n_input] = 2.0 * input_hessian
        P[n_input:, n_input:] = 2.0 * cfg.slack_quadratic_weight * np.eye(
            n_slack
        )
        q[:n_input] = 2.0 * input_gradient
        q[n_input:] = cfg.slack_linear_weight
        P += 1.0e-9 * np.eye(P.shape[0])
        return P, q

    def _constraints(
        self,
        free_prediction: Array,
        previous_input: Array,
    ) -> tuple[Array, Array, Array]:
        cfg = self.config
        N = cfg.horizon
        nu = self.INPUT_DIM
        ny = self.OUTPUT_DIM
        n_input = N * nu
        n_slack = N * self.SLACKS_PER_STEP
        n_variable = n_input + n_slack
        rows: list[Array] = []
        lower: list[float] = []
        upper: list[float] = []
        input_min = np.concatenate((cfg.force_min, [cfg.yaw_moment_min]))
        input_max = np.concatenate((cfg.force_max, [cfg.yaw_moment_max]))
        rate_min = np.concatenate(
            (cfg.delta_force_min, [cfg.delta_yaw_moment_min])
        )
        rate_max = np.concatenate(
            (cfg.delta_force_max, [cfg.delta_yaw_moment_max])
        )

        def append_row(coefficients, lower_bound, upper_bound) -> None:
            rows.append(np.asarray(coefficients, dtype=float))
            lower.append(float(lower_bound))
            upper.append(float(upper_bound))

        for index in range(n_input):
            row = np.zeros(n_variable)
            row[index] = 1.0
            axis = index % nu
            append_row(row, input_min[axis], input_max[axis])

        for index in range(n_input):
            row = np.zeros(n_variable)
            row[:n_input] = self.rate_matrix[index]
            axis = index % nu
            if index < nu:
                append_row(
                    row,
                    previous_input[axis] + rate_min[axis],
                    previous_input[axis] + rate_max[axis],
                )
            else:
                append_row(row, rate_min[axis], rate_max[axis])

        for index in range(n_slack):
            row = np.zeros(n_variable)
            row[n_input + index] = 1.0
            append_row(row, 0.0, cfg.slack_max)

        horizontal_gain = np.tan(
            np.deg2rad(cfg.horizontal_half_fov_deg - cfg.fov_margin_deg)
        )
        vertical_gain = np.tan(
            np.deg2rad(cfg.vertical_half_fov_deg - cfg.fov_margin_deg)
        )
        for step in range(N):
            output_offset = ny * step
            force_forward = self.Su[output_offset]
            force_horizontal = self.Su[output_offset + 1]
            force_vertical = self.Su[output_offset + 2]
            free_forward = free_prediction[output_offset]
            free_horizontal = free_prediction[output_offset + 1]
            free_vertical = free_prediction[output_offset + 2]
            slack_start = n_input + self.SLACKS_PER_STEP * step

            def soft_upper(coefficients, constant, slack_local, limit=0.0) -> None:
                row = np.zeros(n_variable)
                row[:n_input] = coefficients
                row[slack_start + slack_local] = -1.0
                append_row(row, -np.inf, limit - constant)

            soft_upper(
                force_horizontal - horizontal_gain * force_forward,
                free_horizontal - horizontal_gain * free_forward,
                0,
            )
            soft_upper(
                -force_horizontal - horizontal_gain * force_forward,
                -free_horizontal - horizontal_gain * free_forward,
                1,
            )
            soft_upper(
                force_vertical - vertical_gain * force_forward,
                free_vertical - vertical_gain * free_forward,
                2,
            )
            soft_upper(
                -force_vertical - vertical_gain * force_forward,
                -free_vertical - vertical_gain * free_forward,
                3,
            )
            soft_upper(-force_forward, -free_forward, 4, -cfg.forward_distance_min)
            soft_upper(force_forward, free_forward, 5, cfg.forward_distance_max)

            yaw_rate_coefficients = self.Su[output_offset + 7]
            yaw_rate_constant = free_prediction[output_offset + 7]
            row = np.zeros(n_variable)
            row[:n_input] = yaw_rate_coefficients
            append_row(
                row,
                cfg.yaw_rate_min - yaw_rate_constant,
                cfg.yaw_rate_max - yaw_rate_constant,
            )

        return np.vstack(rows), np.asarray(lower), np.asarray(upper)

    def _shift_warm_start(self, decision: Array) -> Array:
        N = self.config.horizon
        n_input = N * self.INPUT_DIM
        inputs = decision[:n_input].reshape(N, self.INPUT_DIM)
        slacks = decision[n_input:].reshape(N, self.SLACKS_PER_STEP)
        return np.concatenate(
            (
                np.vstack((inputs[1:], inputs[-1:])).ravel(),
                np.vstack((slacks[1:], slacks[-1:])).ravel(),
            )
        )

    def _safe_fallback(self, previous: Array, target: Array) -> Array:
        cfg = self.config
        input_min = np.concatenate((cfg.force_min, [cfg.yaw_moment_min]))
        input_max = np.concatenate((cfg.force_max, [cfg.yaw_moment_max]))
        rate_min = np.concatenate(
            (cfg.delta_force_min, [cfg.delta_yaw_moment_min])
        )
        rate_max = np.concatenate(
            (cfg.delta_force_max, [cfg.delta_yaw_moment_max])
        )
        previous = np.clip(previous, input_min, input_max)
        low = np.maximum(input_min, previous + rate_min)
        high = np.minimum(input_max, previous + rate_max)
        return np.minimum(np.maximum(target, low), high)

    def solve(
        self,
        state,
        yaw_rate: float,
        force_previous,
        yaw_moment_previous: float,
        reference_position=None,
        model1_weight=None,
    ) -> YawMPCResult:
        state = np.asarray(state, dtype=float).reshape(-1)
        if state.shape != (6,) or not np.all(np.isfinite(state)):
            raise ValueError("state must be [p_rel(3), v_rel(3)]")
        rate = finite_scalar(yaw_rate, "yaw_rate")
        previous_input = np.concatenate(
            (
                vector3(force_previous, "force_previous"),
                [finite_scalar(yaw_moment_previous, "yaw_moment_previous")],
            )
        )
        weight = (
            np.ones(3)
            if model1_weight is None
            else np.clip(vector3(model1_weight, "model1_weight"), 0.0, 1.0)
        )
        reference = (
            self.config.reference_position
            if reference_position is None
            else vector3(reference_position, "reference_position")
        )

        self._build_prediction_matrices(state, rate, weight)
        augmented_state = np.concatenate(
            (state, [line_of_sight_angle(state[:3]), rate], previous_input)
        )
        free_prediction = self.Sx @ augmented_state + self.Sc
        P, q = self._cost(free_prediction, reference, previous_input)
        constraint_matrix, lower, upper = self._constraints(
            free_prediction, previous_input
        )
        solution = self.solver.solve(
            P,
            q,
            constraint_matrix,
            lower,
            upper,
            warm_start=self._warm_start,
        )

        N = self.config.horizon
        n_input = N * self.INPUT_DIM
        if solution.solved:
            decision = solution.x
            input_sequence = decision[:n_input].reshape(N, self.INPUT_DIM)
            slacks = np.clip(
                decision[n_input:].reshape(N, self.SLACKS_PER_STEP),
                0.0,
                self.config.slack_max,
            )
            input_min = np.concatenate(
                (self.config.force_min, [self.config.yaw_moment_min])
            )
            input_max = np.concatenate(
                (self.config.force_max, [self.config.yaw_moment_max])
            )
            rate_min = np.concatenate(
                (
                    self.config.delta_force_min,
                    [self.config.delta_yaw_moment_min],
                )
            )
            rate_max = np.concatenate(
                (
                    self.config.delta_force_max,
                    [self.config.delta_yaw_moment_max],
                )
            )
            low = np.maximum(input_min, previous_input + rate_min)
            high = np.minimum(input_max, previous_input + rate_max)
            first_input = np.minimum(np.maximum(input_sequence[0], low), high)
            self._warm_start = self._shift_warm_start(decision)
            self._last_feasible_input = first_input.copy()
            used_fallback = False
            status = solution.status
        else:
            if self._last_feasible_input is None:
                target = np.concatenate(
                    (
                        self.model.translation.tau_base,
                        [self.model.yaw.yaw_moment_base],
                    )
                )
                fallback_source = "baseline"
            else:
                target = self._last_feasible_input
                fallback_source = "last_feasible"
            first_input = self._safe_fallback(previous_input, target)
            input_sequence = np.tile(first_input, (N, 1))
            slacks = np.zeros((N, self.SLACKS_PER_STEP))
            self._warm_start = None
            used_fallback = True
            status = f"fallback:{fallback_source}:{solution.status}"

        predicted = (
            free_prediction + self.Su @ input_sequence.ravel()
        ).reshape(N, self.OUTPUT_DIM)
        self._nominal_prediction = predicted.copy()
        return YawMPCResult(
            force=first_input[:3].copy(),
            yaw_moment=float(first_input[3]),
            input_sequence=input_sequence,
            predicted_states=predicted,
            frozen_yaw_rates=self.frozen_yaw_rates.copy(),
            slacks=slacks,
            status=status,
            iterations=solution.iterations,
            objective=solution.objective,
            used_fallback=used_fallback,
            model1_weight=weight.copy(),
            model2_weight=1.0 - weight,
        )
