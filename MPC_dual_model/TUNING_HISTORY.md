# MPC Tuning History

This file records tested parameter sets that may need to be restored exactly.

## 2026-08-06: faster fish tracking

The Unity integration uses symmetric positive and negative limits. No
direction-dependent compensation was added.

| Parameter | Previous value | Current value |
| --- | --- | --- |
| `horizon` | `8` | `10` |
| `reference_position` | `(0.80, 0.0, 0.0)` | `(0.80, 0.0, 0.0)` |
| `position_weights` | `(120.0, 420.0, 220.0)` | `(260.0, 700.0, 380.0)` |
| `velocity_weights` | `(8.0, 12.0, 12.0)` | `(10.0, 16.0, 16.0)` |
| `force_weights` | `(0.04, 0.03, 0.06)` | `(0.025, 0.018, 0.04)` |
| `delta_force_weights` | `(0.8, 0.45, 1.0)` | `(0.45, 0.25, 0.65)` |
| `force_min` | `(-28.0, -24.0, -15.0)` | `(-34.0, -32.0, -20.0)` |
| `force_max` | `(28.0, 24.0, 15.0)` | `(34.0, 32.0, 20.0)` |
| `delta_force_min` | `(-4.0, -3.2, -2.0)` | `(-6.0, -4.8, -3.0)` |
| `delta_force_max` | `(4.0, 3.2, 2.0)` | `(6.0, 4.8, 3.0)` |
| `ForceCommandAdapter.positive_force_at_limit` | `(28.0, 24.0, 15.0)` | `(34.0, 32.0, 20.0)` |
| `slack_quadratic_weight` | default `2.0e4` | `5.0e4` |
| `slack_linear_weight` | default `50.0` | `100.0` |

The current set produced mean absolute forward-distance errors of `0.118 m`
while the fish moved in +X and `0.120 m` while it moved in -X during a 45-second
Unity test. All 901 OSQP samples solved without fallback.

## 2026-08-06: physical force envelope from 7 N thrusters

The `FinsROV.prefab` has four horizontal thrusters mounted at 45 degrees and
four vertical thrusters. Every thruster is configured for symmetric `+7/-7 N`.
Consequently, the maximum pure body force is:

- forward: `4 * 7 * cos(45 deg) = 19.799 N`
- right: `4 * 7 * cos(45 deg) = 19.799 N`
- down: `4 * 7 = 28.0 N`

This set replaces only the force envelope before collecting new tuning data.
The tracking weights remain those from the preceding tuning pass.

| Parameter | Previous value | Current value |
| --- | --- | --- |
| `force_min` | `(-34.0, -32.0, -20.0)` | `(-19.799, -19.799, -28.0)` |
| `force_max` | `(34.0, 32.0, 20.0)` | `(19.799, 19.799, 28.0)` |
| `delta_force_min` | `(-6.0, -4.8, -3.0)` | `(-4.0, -4.0, -5.6)` |
| `delta_force_max` | `(6.0, 4.8, 3.0)` | `(4.0, 4.0, 5.6)` |
| `ForceCommandAdapter.positive_force_at_limit` | `(34.0, 32.0, 20.0)` | `(19.799, 19.799, 28.0)` |

Baseline data with the physical envelope and `position_weights[forward] = 260`:

- duration: `68.7 s` / `1374` control samples
- forward MAE / maximum: `0.133 / 0.200 m`
- reverse MAE / maximum: `0.128 / 0.198 m`
- forward / reverse force saturation: `20.4% / 16.8%`
- QP solve success: `100%`

## 2026-08-06: forward tracking weight iteration 1

Only the forward position weight changes in this iteration. All force and
force-rate limits remain at the 7 N thruster-derived values above.

| Parameter | Previous value | Current value |
| --- | --- | --- |
| `position_weights` | `(260.0, 700.0, 380.0)` | `(520.0, 700.0, 380.0)` |

Iteration 1 result (`69.0 s`, `1381` samples):

- forward MAE / maximum: `0.098 / 0.170 m`
- reverse MAE / maximum: `0.091 / 0.174 m`
- overall forward-error MAE improvement: `28.3%`
- force saturation: `21.7%` (baseline `18.5%`)
- QP solve success: `100%`

## 2026-08-06: forward tracking weight iteration 2

| Parameter | Previous value | Current value |
| --- | --- | --- |
| `position_weights` | `(520.0, 700.0, 380.0)` | `(800.0, 700.0, 380.0)` |

Iteration 2 result (`69.0 s`, `1381` samples):

- forward MAE / maximum: `0.086 / 0.162 m`
- reverse MAE / maximum: `0.082 / 0.197 m`
- force saturation: `29.2%`
- QP solve success: `100%`

Although the overall MAE decreased by another `10.3%`, saturation increased by
`7.5` percentage points and the reverse maximum error became `13.3%` worse.
Therefore iteration 2 was rejected and the setting selected for the `0.18 m/s`
test was restored to `position_weights = (520.0, 700.0, 380.0)`.

## Selected current configuration

| Parameter | Selected value |
| --- | --- |
| `horizon` | `10` |
| `reference_position` | `(0.80, 0.0, 0.0)` |
| `position_weights` | `(1600.0, 700.0, 15000.0)` |
| `velocity_weights` | `(18.0, 16.0, 16.0)` |
| `force_weights` | `(0.012, 0.018, 0.04)` |
| `delta_force_weights` | `(0.08, 0.25, 0.65)` |
| `force_min/max` | `+/- (19.799, 19.799, 28.0) N` |
| `delta_force_min/max` | `+/- (4.0, 4.0, 5.6) N/sample` |
| `slack_quadratic_weight` | `5.0e4` |
| `slack_linear_weight` | `100.0` |

## 2026-08-06: calibration audit and fish-speed reduction

The `CALIBRATION_CHECKLIST.md` audit separates parameters verified for the
Unity ground-truth integration from simulation assumptions and real-hardware
calibration work. In particular, `M_t`, `D_L`, camera extrinsics, measurement
noise, and total vision latency remain uncalibrated placeholders.

The straight-line fish speed was reduced from `0.18 m/s` to `0.12 m/s`. At
`0.18 m/s`, the selected `position_weights[forward] = 520` test used roughly
`17.2 N` mean absolute forward force, about `87%` of the `19.799 N` axis limit,
and saturated for `21.7%` of samples. The lower speed is intended to restore
control headroom before the next weight iteration.

The OSQP time limit was also reduced from `0.08 s` to `0.035 s`, so it is now
strictly shorter than the `0.05 s` control period.

## 2026-08-06: tuning at 0.12 m/s fish speed

Both runs contain `1372` samples over `68.6 s`, cover positive and negative X
motion, and retain the same symmetric 7 N thruster-derived force limits. Only
the forward position weight differs.

| Metric | `qx=520` baseline | `qx=650` candidate |
| --- | --- | --- |
| Positive-X error MAE / maximum | `0.0680 / 0.1037 m` | `0.0620 / 0.0976 m` |
| Negative-X error MAE / maximum | `0.0691 / 0.1149 m` | `0.0632 / 0.1107 m` |
| Overall error MAE | `0.0676 m` | `0.0619 m` |
| Mean absolute forward force | `11.28 N` | `11.50 N` |
| Peak absolute forward force | `17.47 N` | `19.799 N` |
| Forward-force saturation | `0.00%` | `0.15%` |
| QP solve success | `100%` | `100%` |

The `qx=650` candidate improves overall forward error MAE by `8.3%` for only
`0.22 N` additional mean force. Its two approximate saturation samples are
accepted, so the selected position weights are now `(650, 700, 380)`. The
lateral weight is not changed because the straight-X fish path provides no
lateral excitation for a defensible comparison.

## 2026-08-06: superseded target-motion feedforward experiment

UnderwaterVision uses a `26.07276 kg` Rigidbody and DWP2 `WaterObject`
hydrodynamics. Three-axis `+/-6 N` and `+/-12 N` step responses were fitted
directly against the complete velocity response. In MPC forward/right/down
order (Unity `X/Z/-Y`), the selected simulation model is:

- `M_t = diag(26.07276, 26.79684, 26.07276) kg`
- `D_L = diag(93.88006, 143.69195, 280.86849) N/(m/s)`
- fixed initial `tau_base = (0, 0, 1.89299) N`

The axis velocity-fit RMSE values are `0.00276`, `0.00320`, and `0.00211 m/s`,
with `R²` values `0.9986`, `0.9956`, and `0.9928`. These are simulation-only
equivalent parameters, not FineSUB real-hardware calibration results. The raw
fit is stored in `dynamics_20260806_212129.json`.

The tracker estimator was corrected to propagate absolute force as
`B @ (tau - tau_base)`. Target velocity now uses the Unity message timestamp,
and the MPC prediction model includes the target-motion equilibrium force
`tau_base + D_L @ target_velocity + M_t @ target_acceleration`. This is not an
output-side compensation term. The fish follows a straight `3 m` ping-pong
path at up to `0.12 m/s`, with approximately `0.08 m/s²` smooth turnarounds.

The final tuning retained every physical force and force-rate limit. Only the
forward delta-force cost changed from `0.45` to `0.15`; the down-position
weight is `15000` to reject the identified upward environmental bias.

Final result (`73.55 s`, `1471` samples):

- 3D error mean / maximum: `0.00739 / 0.02492 m`
- samples below `0.03 m`: `1471 / 1471` (`100%`)
- turnaround error mean / maximum: `0.01744 / 0.02492 m`
- peak forward force: `14.90 N` of the unchanged `19.799 N` limit
- OSQP success: `100%`

Artifacts are `identified_mpc_tracking_final_decoupled_20260806.csv` and
`identified_mpc_tracking_final_decoupled_20260806.png` in the ROS workspace
`data/mpc_tracking` directory.

This result is retained for comparison only. It used separate Unity world
poses for the fish and ROV, derived absolute target velocity/acceleration, and
smooth endpoint turnarounds. Those inputs exceed the real controller's stated
sensor boundary, so this run is not a valid acceptance result for the current
integration.

## 2026-08-06: observable-only relative-position interface

The active interface now exposes only an ideal body-FRD target-relative
position vector and its message timestamp. The translation MPC no longer
subscribes to ROV world pose, fish world pose, DVL, target absolute velocity,
or target absolute acceleration. Its Kalman filter estimates relative velocity
from relative-position history. The prior saturated force command remains
available internally as an approximation of achieved force.

`M_t` and `D_L` remain fixed at the identified simulation constants for the
entire run. `tau_base` now starts at `(0,0,0) N` and the observable adaptation
is explicitly enabled. The fish endpoint behavior is again an instantaneous
reversal. Acceptance is based on constant-speed steady-state error below
`0.03 m`, not the unavoidable reversal transient.

## 2026-08-06: observable baseline deadlock fix and cost sweep

The original matched-state gate used three-axis norms and required total
position error below `0.06 m` and relative speed below `0.04 m/s`. Once a fish
reversal produced about `0.14 m` error, `tau_base` could no longer update from
the old `-11.4 N` equilibrium to the required positive equilibrium. Baseline
learning therefore deadlocked even though the optimizer continued solving.

Baseline adaptation now evaluates each axis independently, uses rate `0.03`,
position tolerance `0.20 m`, and relative-velocity tolerance `0.08 m/s`, and
clips the learned baseline to the unchanged physical force limits. It still
uses only relative-position-derived state and the preceding applied command;
no fish/ROV world pose or absolute target velocity is exposed. `M_t` and `D_L`
remain fixed.

Each cost candidate contains `1301` samples over `65.0 s`, covers both fish
directions and instantaneous reversals, and has `100%` QP solve success.

| Metric | Deadlocked baseline | Candidate 1 | Candidate 2 |
| --- | ---: | ---: | ---: |
| Forward position / velocity weight | `650 / 10` | `1600 / 18` | `2400 / 22` |
| Forward force / delta-force weight | `0.025 / 0.15` | `0.012 / 0.08` | `0.008 / 0.05` |
| Overall forward-error MAE | `0.07859 m` | `0.01246 m` | `0.01480 m` |
| Overall forward-error 95th percentile | `0.19864 m` | `0.09203 m` | `0.04527 m` |
| Steady forward-error MAE | deadlocked in one direction | `0.00448 m` | `0.01234 m` |
| Steady forward-error 95th percentile | deadlocked in one direction | `0.01185 m` | `0.01980 m` |
| Peak reversal error | `0.24510 m` | `0.14901 m` | `0.11803 m` |
| Forward saturation samples | `65` | `83` | `99` |

Candidate 2 reduces the instantaneous reversal peak but worsens steady error
and produces visibly larger force oscillation. Candidate 1 is selected because
the acceptance target concerns constant-speed tracking and its steady 95th
percentile remains below `0.012 m`, with both learned directional equilibria
converging near `+/-11.3 N`.

## 2026-08-06: baseline update strategy and rate comparison

The bridge now supports two explicit baseline modes. `gated_ema` is the normal
mode. `previous_force` is retained only to reproduce the requested comparison;
it assigns the preceding command directly to `tau_base` every sample.

The direct `previous_force` experiment was stopped after 201 samples because
it was clearly unstable. Its mean 3-D error was `0.14469 m`, compared with
`0.01380 m` for gated EMA. At least one axis saturated in 180/201 samples, and
all three `tau_base` axes traversed their complete physical limits. It must not
be used as the default because feedback corrections are not a slowly varying
motion equilibrium.

Four gated EMA rates were then compared over 1301 samples / 65 seconds each:

| Baseline rate | Overall X MAE | Steady X MAE | Steady X p95 | Saturation samples |
| --- | ---: | ---: | ---: | ---: |
| Fixed `0.03` | `0.01246 m` | `0.00352 m` | `0.00803 m` | `83` |
| Fixed `0.015` | `0.01293 m` | `0.00243 m` | `0.01000 m` | `61` |
| Fixed `0.01` | `0.01980 m` | `0.00157 m` | `0.00470 m` | `118` |
| Adaptive `0.03 -> 0.01` | `0.01389 m` | `0.00128 m` | `0.00367 m` | `95` |

The adaptive rate is selected because acceptance prioritizes constant-speed
error rather than the unavoidable instantaneous-reversal peak. Each axis uses
rate `0.03` while its absolute position error exceeds `0.03 m` or estimated
relative speed exceeds `0.02 m/s`; it switches to `0.01` inside that steady
region. Baseline eligibility remains independently gated per axis at `0.20 m`
and `0.08 m/s`. The unchanged force envelope is `+/-19.799 N` forward/right
and `+/-28 N` down.

## 2026-08-07: independently tuned direct and indirect baseline strategies

The two baseline update strategies were tuned independently in Unity rather
than evaluating `previous_force` with weights selected for `gated_ema`. Every
screening trial started from a fresh UnderwaterVision Play session. The fish
used the same 3 m straight-X PingPong trajectory at 0.12 m/s with instantaneous
reversals every 25 seconds.

Five 40-second direct `previous_force` candidates were tested. Increasing
velocity and force-rate penalties reduced the original immediate instability,
but no candidate produced two continuous seconds with estimated relative speed
inside +/-0.02 m/s. The selected direct weights are:

| Weight | Forward / Right / Down |
| --- | --- |
| Position | `8000 / 1000 / 20000` |
| Velocity | `3000 / 1000 / 20000` |
| Force | `0.2 / 0.5 / 2.0` |
| Delta force | `4 / 8 / 40` |

Three 40-second indirect `gated_ema` candidates were tested around the previous
selection. More aggressive position tracking and additional velocity damping
both worsened either steady error or reversal recovery. The retained indirect
weights remain:

| Weight | Forward / Right / Down |
| --- | --- |
| Position | `1600 / 700 / 15000` |
| Velocity | `18 / 16 / 16` |
| Force | `0.012 / 0.018 / 0.04` |
| Delta force | `0.08 / 0.25 / 0.65` |

The selected candidates were then rerun for 65 seconds / about 1301 samples,
covering two reversals. A fixed constant-speed window excludes eight seconds
after startup and each reversal; it does not select samples based on error.

| Metric | Direct `previous_force` | Indirect `gated_ema` |
| --- | ---: | ---: |
| Full-run X MAE | `0.06987 m` | `0.01699 m` |
| Constant-speed X MAE / p95 | `0.05831 / 0.12454 m` | `0.00203 / 0.00644 m` |
| Constant-speed 3-D MAE / p95 | `0.14034 / 0.25513 m` | `0.00269 / 0.00767 m` |
| Error-norm samples <= 0.03 m | `0.92%` | `86.10%` |
| At least one saturated axis | `46.20%` | `9.98%` |
| Strict matched-speed samples | `0` | `915` |
| QP solve success | `100%` | `100%` |

Indirect baseline adaptation reduces constant-speed X MAE by `96.5%` and 3-D
MAE by `98.1%`. The default remains `gated_ema`; the tuned direct profile is
retained for reproducible comparison, not selected for normal operation.
## 2026-08-07: restore pure model-2 branch and enforce joint thruster envelope

The maintained path remains `MPC_dual_model`. The fused prediction now exactly
uses the two candidate structures:

```text
model 1: x+ = A_d x + B_d (tau - tau_base)
model 2: x+ = A_d x + B_d tau
```

The previous change that subtracted `tau_base` in model 2 was removed. The
baseline is now used only by model 1, observable baseline learning, and safe
fallback; it is not a privileged fish-motion input and never enters model-2
prediction. The bridge publishes both model weights, fusion sample count, and
normalized thruster utilization so that constant-speed behavior can be checked
directly. The maintained fusion also updates the model parameter with rate
`0.20`, so close residuals cannot make the controller switch candidates every
sample. At that time the screening configuration used `minimum_weight=0.05`;
the maintained configuration is now documented in the current-baseline entry
below and uses `minimum_weight=0.01`, so a stable axis can reach `0.99`.

The QP now also constrains the eight FineSUB translation commands. The four
horizontal 45-degree thrusters share the joint envelope
`|F_forward|/19.799 + |F_right|/19.799 <= 1`; the four vertical thrusters retain
`|F_down| <= 28 N`. This replaces the earlier assumption that all axis maxima
could be reached simultaneously. This entry recorded an earlier `0.08 m/s`
screening run; it is not the current Unity scene value. Automatic trials then
opened the live error/weight plot by default and saved it to
`ros2_ws/mpc_tracking_live.png`.

## 2026-08-07: current dual-model curve baseline and diagnostic observability

The effective Unity baseline is the `UnderwaterVision` scene configuration:
horizontal `StraightXWithZSin`, `xTravelDistance=0.75 m`,
`movementSpeed=0.015 m/s`, `zSinAmplitude=0.10 m`, `sinCycles=1`, constant
depth, and instantaneous PingPong reversal. The 65 s `gated_ema` trial is the
current comparison baseline: 1301 samples, 20 Hz, 3-D error mean `0.00857 m`,
P95 `0.02072 m`, constant-speed 3-D error mean `0.00579 m`, P95 `0.01520 m`,
98.23% of samples within `0.03 m`, QP success `100%`, and zero saturation.

The dual-model equations remain exactly:

```text
model 1: x+ = A_d x + B_d (tau - tau_base)
model 2: x+ = A_d x + B_d tau
```

`tau_base` is not target feedforward and is not visible to Unity as an extra
measurement. It is learned only from the observable relative position, its
filtered relative velocity, and the previous applied command approximation;
it enters model 1 and safe fallback only. A per-axis candidate-difference
floor of `1e-7 m2` now holds the previous fusion weight when the two residual
models are indistinguishable at the current measurement scale. Neutral
evidence recovers toward model 1, preventing model-2 noise switching during
constant-speed/constant-depth tracking while preserving model-2 selection for
a clearly better rapid-change prediction.

The diagnostics message keeps the original 33 fields and appends 9 fields:
right/down relative velocity, 3-D speed, three relative accelerations, and the
three candidate-difference scores. Each automatic trial now writes its own
live PNG next to its CSV, while manual launches continue to update
`ros2_ws/mpc_tracking_live.png`.

## 2026-08-07: joint-envelope horizontal curve tuning

The final pressure trajectory is a horizontal XZ sine PingPong path with
`xTravelDistance=0.75 m`, `movementSpeed=0.030 m/s`, `zSinAmplitude=0.08 m`,
and two sine cycles per one-way traversal. Endpoint reversal remains
instantaneous. The selected FRD costs are:

| Weight | Forward / Right / Down |
| --- | --- |
| Position | `1600 / 3000 / 15000` |
| Velocity | `18 / 30 / 16` |
| Effective force | `0.025 / 0.012 / 0.06` |
| Delta force | `0.12 / 0.06 / 0.8` |

The 65-second trial recorded 1302 samples at 20 Hz. Full-run 3-D error mean
was `0.0193 m`; strict matched-relative-speed error mean/P95 was
`0.0096/0.0268 m`. The horizontal joint envelope reached exactly `1.0` for
four startup samples at `0.20--0.35 s` and remained below the limit afterward,
including both instantaneous endpoint reversals. No command exceeded the
joint thruster envelope.

All three model-1 weight medians were approximately `0.99`. In the highest
15 percent estimated-relative-acceleration samples, the mean forward/right
model-1 weights fell to `0.935/0.463`; model 2 had lower forward/right MSE in
that segment. Thus constant-motion behavior remains model-1 dominant while
the rapid-change samples retain a measurable model-2 contribution.

## 2026-08-08: maximum confirmed S-trajectory speed

The Unity project now lives at
`/home/fins/Zhouyuheng_workspace/marus-example`. The selected scene uses
`xTravelDistance=0.75 m`, `movementSpeed=0.11 m/s`, `zSinAmplitude=0.04 m`,
one sine cycle per one-way traversal, constant depth, and instantaneous
PingPong reversal. The selected FRD costs are:

| Weight | Forward / Right / Down |
| --- | --- |
| Position | `10000 / 5000 / 25000` |
| Velocity | `2 / 20 / 12` |
| Effective force | `0.003 / 0.008 / 0.04` |
| Delta force | `0.01 / 0.04 / 0.5` |

The final 65-second trial recorded 1302 samples at 20 Hz. Continuous settled
tracking had 3-D error mean/P95 `0.0178/0.0238 m`. After the first five
seconds, the 626 samples with estimated relative speed at or below `0.04 m/s`
had mean/P95 `0.0174/0.0298 m`. Instantaneous reversals are intentionally not
required to remain below `0.03 m`.

The horizontal joint force envelope was active for `27.6%` of samples but was
never exceeded. A `0.115 m/s` boundary trial produced no continuous settled
samples, and `0.12 m/s` saturated the horizontal envelope for `60.5%` of the
trial without settling. Therefore `0.11 m/s` is the highest confirmed speed
for this trajectory and the current physical force limits.

## 2026-08-08: V4 Pro1 asymmetric limits and racetrack tuning

The current QP replaces the historical symmetric `+/-7 N` approximation with
the V4 Pro1 canonical per-thruster positive/negative `force_n` limits. The
resulting pure-axis bounds are `(-16.2968,-16.2968,-23.0472) N` to
`(16.2968,16.2968,29.5236) N`; all eight asymmetric row constraints remain in
the QP, so diagonal requests are further coupled.

The Unity fish now follows a continuous horizontal racetrack with `0.75 m`
straight sections, `0.20 m` semicircle radius, constant depth, and a `2.0 s`
start delay aligned with relative-target publication. The selected speed is
`0.0975 m/s`, and the selected FRD costs are position
`(10000,14000,25000)`, velocity `(2,20,12)`, effective force
`(0.003,0.002,0.04)`, and delta force `(0.01,0.01,0.5)`.

The final 65-second run recorded 1302 samples. Matched-relative-speed error
mean/P95 was `0.01237/0.02665 m`; low-acceleration P95 was `0.01655 m`,
settled P95 was `0.01032 m`, solver success was `100%`, and the exact
per-thruster horizontal envelope was active for `23.1%` of samples. A
`0.10 m/s` run with the same final weights reached matched P95 `0.03205 m`,
so `0.0975 m/s` is the highest confirmed speed under these limits.
