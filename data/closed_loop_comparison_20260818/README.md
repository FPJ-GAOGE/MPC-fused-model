# Closed-loop comparison data (2026-08-18)

This directory contains the extracted data used to draw:

- `calibration_logs/mpc_fusion_vs_fixed_200cm_20260818.png`
- `calibration_logs/pid_smc_model_comparison_20260818.png`

The `*_case_samples.csv` files retain the selected samples for every n=3 case.
The `*_plotted_curve.csv` files retain the 0.10 s grid, group mean, and Gaussian
smoothed curves used by the plotting scripts. Errors are in centimetres and are
computed relative to `reference_forward_m = 0.857634` m.

The figures contain five unique method groups: PID, SMC, fusion model, fixed
model1, and fixed model2.

The extraction command is:

```bash
uv run python data/analysis/extract_closed_loop_comparison_data_20260818.py
```
