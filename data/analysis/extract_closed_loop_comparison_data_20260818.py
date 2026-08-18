#!/usr/bin/env python3
"""Extract the exact per-case and plotted curves used by the two 2026-08-18 figures."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


MPC_ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = MPC_ROOT / "calibration_logs"
OUTPUT_DIR = MPC_ROOT / "data" / "closed_loop_comparison_20260818"
WINDOW_S = 20.0
BIN_S = 0.10

if str(MPC_ROOT) not in sys.path:
    sys.path.insert(0, str(MPC_ROOT))
if str(LOG_DIR) not in sys.path:
    sys.path.insert(0, str(LOG_DIR))

from calibration_logs.plot_mpc_fusion_vs_fixed_200cm import (  # noqa: E402
    CASES as FIXED_FUSION_CASES,
    load_case as load_fixed_fusion_case,
)
from data.analysis.plot_pid_smc_model_comparison_20260818 import (  # noqa: E402
    load_mpc_cases,
    load_pid_cases,
    load_smc_cases,
)


def _case_start(item: dict[str, Any]) -> float:
    value = item.get("window_start_host_time_s")
    if value is None:
        value = item["start_host_time_s"]
    return float(value)


def _case_end(item: dict[str, Any]) -> float:
    value = item.get("window_end_host_time_s")
    if value is None:
        value = item["end_host_time_s"]
    return float(value)


def _case_rows(item: dict[str, Any], figure: str) -> Iterable[dict[str, Any]]:
    time_s = np.asarray(item["time_s"], dtype=float)
    error_cm = np.asarray(item["error_cm"], dtype=float)
    abs_error_cm = np.asarray(item["abs_error_cm"], dtype=float)
    forward_m = np.asarray(item.get("forward_m", np.full(time_s.shape, np.nan)), dtype=float)
    velocity = np.asarray(item.get("velocity_m_s", np.full(time_s.shape, np.nan)), dtype=float)
    weight = np.asarray(item.get("weight", np.full(time_s.shape, np.nan)), dtype=float)
    for index in range(time_s.size):
        yield {
            "figure": figure,
            "method": str(item["group"]),
            "case": str(item["label"]),
            "source": str(item["trace"]),
            "window_start_host_time_s": _case_start(item),
            "window_end_host_time_s": _case_end(item),
            "sample_index": index,
            "time_s": float(time_s[index]),
            "forward_m": float(forward_m[index]),
            "signed_error_cm": float(error_cm[index]),
            "absolute_error_cm": float(abs_error_cm[index]),
            "velocity_m_s": float(velocity[index]),
            "model1_weight": float(weight[index]),
        }


def _binned_mean(time_s: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    edges = np.arange(0.0, WINDOW_S + BIN_S + 1.0e-9, BIN_S)
    centers = 0.5 * (edges[:-1] + edges[1:])
    result = np.full(centers.shape, np.nan, dtype=float)
    for index in range(centers.size):
        mask = (time_s >= edges[index]) & (time_s < edges[index + 1])
        if np.any(mask):
            result[index] = float(np.mean(values[mask]))
    return centers, result


def _smooth(values: np.ndarray, sigma_bins: float = 2.0) -> np.ndarray:
    radius = max(1, int(np.ceil(3.0 * sigma_bins)))
    offsets = np.arange(-radius, radius + 1, dtype=float)
    kernel = np.exp(-0.5 * (offsets / sigma_bins) ** 2)
    kernel /= np.sum(kernel)
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _plotted_curve(items: list[dict[str, Any]], field: str) -> tuple[np.ndarray, np.ndarray]:
    grid = np.arange(BIN_S / 2.0, WINDOW_S, BIN_S)
    curves = []
    for item in items:
        centers, values = _binned_mean(
            np.asarray(item["time_s"], dtype=float),
            np.asarray(item[field], dtype=float),
        )
        finite = np.isfinite(values)
        if np.count_nonzero(finite) < 2:
            raise RuntimeError(f"not enough finite bins for {item['label']}")
        curves.append(np.interp(grid, centers[finite], values[finite]))
    return grid, _smooth(np.mean(np.asarray(curves), axis=0))


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _case_metadata(item: dict[str, Any]) -> dict[str, Any]:
    result = {
        "label": str(item["label"]),
        "method": str(item["group"]),
        "source": str(item["trace"]),
        "window_start_host_time_s": _case_start(item),
        "window_end_host_time_s": _case_end(item),
        "sample_count": int(len(item["time_s"])),
        "selection_method": str(item.get("selection_method", "")),
    }
    for key in ("camera_source", "stereo_source", "coordinate_frame", "source_position_field"):
        if key in item:
            result[key] = item[key]
    return result


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    figure1_cases = [load_fixed_fusion_case(case) for case in FIXED_FUSION_CASES]
    figure1_groups = {
        method: [item for item in figure1_cases if item["group"] == method]
        for method in ("fusion", "model1_fixed", "model2_fixed")
    }
    figure2_cases = load_pid_cases() + load_smc_cases() + load_mpc_cases()
    figure2_cases = [
        item for item in figure2_cases if item["group"] in {"pid", "smc", "fusion"}
    ]
    figure2_groups = {
        method: [item for item in figure2_cases if item["group"] == method]
        for method in ("pid", "smc", "fusion")
    }

    sample_fields = [
        "figure",
        "method",
        "case",
        "source",
        "window_start_host_time_s",
        "window_end_host_time_s",
        "sample_index",
        "time_s",
        "forward_m",
        "signed_error_cm",
        "absolute_error_cm",
        "velocity_m_s",
        "model1_weight",
    ]
    _write_csv(
        OUTPUT_DIR / "figure1_fusion_fixed_case_samples.csv",
        [row for item in figure1_cases for row in _case_rows(item, "fusion_fixed")],
        sample_fields,
    )
    _write_csv(
        OUTPUT_DIR / "figure2_pid_smc_fusion_case_samples.csv",
        [row for item in figure2_cases for row in _case_rows(item, "pid_smc_fusion")],
        sample_fields,
    )

    grid, fusion_curve = _plotted_curve(figure1_groups["fusion"], "abs_error_cm")
    _, model1_curve = _plotted_curve(figure1_groups["model1_fixed"], "abs_error_cm")
    _, model2_curve = _plotted_curve(figure1_groups["model2_fixed"], "abs_error_cm")
    _write_csv(
        OUTPUT_DIR / "figure1_fusion_fixed_plotted_curve.csv",
        [
            {
                "time_s": float(grid[index]),
                "fusion_absolute_error_cm": float(fusion_curve[index]),
                "model1_fixed_absolute_error_cm": float(model1_curve[index]),
                "model2_fixed_absolute_error_cm": float(model2_curve[index]),
            }
            for index in range(grid.size)
        ],
        [
            "time_s",
            "fusion_absolute_error_cm",
            "model1_fixed_absolute_error_cm",
            "model2_fixed_absolute_error_cm",
        ],
    )

    curve_rows = [{"time_s": float(value)} for value in grid]
    for method, items in figure2_groups.items():
        _, abs_curve = _plotted_curve(items, "abs_error_cm")
        _, signed_curve = _plotted_curve(items, "error_cm")
        for index in range(grid.size):
            curve_rows[index][f"{method}_absolute_error_cm"] = float(abs_curve[index])
            curve_rows[index][f"{method}_signed_error_cm"] = float(signed_curve[index])
    _write_csv(
        OUTPUT_DIR / "figure2_pid_smc_fusion_plotted_curve.csv",
        curve_rows,
        [
            "time_s",
            "pid_absolute_error_cm",
            "pid_signed_error_cm",
            "smc_absolute_error_cm",
            "smc_signed_error_cm",
            "fusion_absolute_error_cm",
            "fusion_signed_error_cm",
        ],
    )

    summary1 = json.loads(
        (LOG_DIR / "mpc_fusion_vs_fixed_200cm_20260818.json").read_text(encoding="utf-8")
    )
    summary2 = json.loads(
        (LOG_DIR / "pid_smc_model_comparison_20260818.json").read_text(encoding="utf-8")
    )
    metadata = {
        "dataset": "closed_loop_comparison_20260818",
        "description": "Extracted per-case samples and exact smoothed mean curves used by the two 2026-08-18 figures.",
        "window_s": WINDOW_S,
        "bin_s": BIN_S,
        "reference_forward_m": 0.857634,
        "error_definition": "signed_error_cm = 100 * (forward_m - reference_forward_m); absolute_error_cm = abs(signed_error_cm)",
        "phase_annotation": {
            "motion_stage_s": [0.0, 15.0],
            "emergency_stop_s": 15.0,
            "post_stop_observation_s": [15.0, 20.0],
        },
        "figures": {
            "figure1": {
                "name": "fusion_vs_fixed",
                "groups": {key: len(value) for key, value in figure1_groups.items()},
                "sample_file": "figure1_fusion_fixed_case_samples.csv",
                "curve_file": "figure1_fusion_fixed_plotted_curve.csv",
                "summary_source": "../../calibration_logs/mpc_fusion_vs_fixed_200cm_20260818.json",
                "cases": [_case_metadata(item) for item in figure1_cases],
            },
            "figure2": {
                "name": "pid_smc_fusion",
                "groups": {key: len(value) for key, value in figure2_groups.items()},
                "sample_file": "figure2_pid_smc_fusion_case_samples.csv",
                "curve_file": "figure2_pid_smc_fusion_plotted_curve.csv",
                "summary_source": "../../calibration_logs/pid_smc_model_comparison_20260818.json",
                "cases": [_case_metadata(item) for item in figure2_cases],
            },
        },
        "unique_method_groups_in_figures": [
            "pid",
            "smc",
            "fusion",
            "model1_fixed",
            "model2_fixed",
        ],
        "unique_method_group_count": 5,
        "method_group_note": "The two figures together contain five unique method groups: PID, SMC, fusion model, fixed model1, and fixed model2.",
        "summary_metrics": {
            "figure1": summary1["group_phase_mean_abs_error_cm"],
            "figure2": summary2["group_overall_metrics"],
        },
    }
    (OUTPUT_DIR / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    readme = """# Closed-loop comparison data (2026-08-18)

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
"""
    (OUTPUT_DIR / "README.md").write_text(readme, encoding="utf-8")

    print(OUTPUT_DIR)
    print(json.dumps(metadata["figures"], ensure_ascii=False))


if __name__ == "__main__":
    main()
