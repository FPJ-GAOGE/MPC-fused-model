"""Hardware-free closed-loop demonstration of the frozen-rotation yaw MPC."""

from __future__ import annotations

import numpy as np

from .live_integration_example import build_tracker


def main() -> None:
    tracker = build_tracker()
    model = tracker.model
    controller = tracker.controller

    # Target starts forward-right in the body frame.  The plant below uses the
    # absolute-force translation candidate and the linear Fossen yaw model.
    state = np.array([1.10, 0.35, -0.08, 0.0, 0.0, 0.0])
    yaw = 0.0
    yaw_rate = 0.0
    force_previous = np.zeros(3)
    yaw_moment_previous = 0.0

    print("step | p_body [forward right down] | alpha(deg) | r(deg/s) | N(Nm)")
    for step in range(120):
        result = controller.solve(
            state=state,
            yaw_rate=yaw_rate,
            force_previous=force_previous,
            yaw_moment_previous=yaw_moment_previous,
            # The demonstration plant below is exactly candidate model 2, so
            # use its known weight here.  The real tracker estimates this
            # weight online from completed position predictions.
            model1_weight=(0.0, 0.0, 0.0),
        )

        # The actual yaw increment must come from the IMU in real use.  This
        # simulation integrates the model because no IMU exists here.
        delta_yaw = model.dt * yaw_rate
        state = model.predict_model2(state, result.force, delta_yaw)
        yaw = yaw + delta_yaw
        yaw_rate = model.yaw.predict_rate(yaw_rate, result.yaw_moment)
        force_previous = result.force.copy()
        yaw_moment_previous = result.yaw_moment

        if step % 10 == 0 or step == 119:
            alpha_deg = np.rad2deg(np.arctan2(state[1], state[0]))
            print(
                f"{step:4d} | {state[:3]} | {alpha_deg:9.3f} | "
                f"{np.rad2deg(yaw_rate):8.3f} | {result.yaw_moment:7.3f}"
            )

    print("final yaw (deg):", np.rad2deg(yaw))
    print("final state:", state)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    main()
