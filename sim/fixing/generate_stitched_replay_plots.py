#!/usr/bin/env python3
"""Generate final stitched-replay plots for Flight 0 vs Flight 1.

Outputs are written to sim/fixing:
- stitched_full_replay.png
- stitched_timing_zoom.png
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

DT = 0.002
G = 9.80665
VEL_CONTROL_START = 55.0
VEL_DESCENT = -1.0
ALT_LANDED = 2.0

SIGMA_A = 0.0645
SIGMA_B = 0.0194
SIGMA_X = 5.46

A = np.array([[1.0, DT, -0.5 * DT * DT], [0.0, 1.0, -DT], [0.0, 0.0, 1.0]], dtype=float)
B = np.array([0.5 * DT * DT, DT, 0.0], dtype=float)
Q = np.array(
    [
        [SIGMA_A * SIGMA_A * DT**4 / 4.0, SIGMA_A * SIGMA_A * DT**3 / 2.0, 0.0],
        [SIGMA_A * SIGMA_A * DT**3 / 2.0, SIGMA_A * SIGMA_A * DT**2, 0.0],
        [0.0, 0.0, SIGMA_B * SIGMA_B * DT],
    ],
    dtype=float,
)
R = SIGMA_X * SIGMA_X


def load(path: Path) -> dict[str, np.ndarray]:
    rows: list[dict[str, str]] = []
    with path.open() as f:
        rows.extend(csv.DictReader(f))

    def fcol(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=float)

    return {
        "t": fcol("time_ms") * 1e-3,
        "acc": fcol("raw_accel_ms2"),
        "baro": fcol("raw_baro_m"),
        "vel": fcol("velocity_ms"),
        "state": np.array([r["state"] for r in rows], dtype=object),
    }


def baro_vel_preapo(data: dict[str, np.ndarray], dt: float = 0.01, win_s: float = 0.25) -> tuple[np.ndarray, np.ndarray]:
    t = data["t"]
    b = data["baro"]
    tu = np.arange(t[0], t[-1], dt)
    bu = np.interp(tu, t, b)
    n = max(5, int(round(win_s / dt)))
    if n % 2 == 0:
        n += 1
    k = np.hanning(n)
    k /= k.sum()
    bs = np.convolve(bu, k, mode="same")
    apo = int(np.argmax(bs))
    tu = tu[: apo + 1]
    bs = bs[: apo + 1]
    bv = np.gradient(bs, dt)
    return tu, bv


def control_logged(data: dict[str, np.ndarray]) -> float:
    i = np.where(data["state"] == "CONTROL")[0]
    return float(data["t"][i[0]])


def run_firmware_like(t_meas: np.ndarray, a_meas: np.ndarray, b_meas: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    tu = np.arange(t_meas[0], t_meas[-1] + 1e-12, DT)
    au = np.interp(tu, t_meas, a_meas)
    idx = np.round((t_meas - t_meas[0]) / DT).astype(int)

    x = np.array([0.0, 0.0, 0.0], dtype=float)
    P = np.diag([1e-6, 1e-6, 1e-6]).astype(float)

    mode = "BOOST"
    t_control = float("nan")

    x_hist = np.zeros((len(tu), 3), dtype=float)

    mi = 0
    for k, aa in enumerate(au):
        x = A @ x + B * aa
        P = A @ P @ A.T + Q

        if mode == "BOOST":
            if (x[1] < VEL_CONTROL_START) and (x[0] > ALT_LANDED) and (aa < 0.0):
                mode = "CONTROL"
                t_control = float(tu[k])
        elif mode == "CONTROL":
            if x[1] < VEL_DESCENT:
                mode = "DESCENT"

        while mi < len(idx) and idx[mi] == k:
            z = b_meas[mi]
            if mode in ("CONTROL", "DESCENT") and abs(aa) < 4.0 * G:
                y = z - x[0]
                S = P[0, 0] + R
                K = np.array([P[0, 0] / S, P[1, 0] / S, P[2, 0] / S], dtype=float)
                x = x + K * y
                I = np.array([[1.0 - K[0], 0.0, 0.0], [-K[1], 1.0, 0.0], [-K[2], 0.0, 1.0]], dtype=float)
                P = I @ P
            mi += 1

        x_hist[k] = x

    return tu, au, x_hist, t_control


def stitch_with_baro_continuity(ref: dict[str, np.ndarray], target: dict[str, np.ndarray], tau: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, int]:
    dtm = 0.01

    t_ref = np.arange(0.0, min(ref["t"][-1], tau) + 1e-12, dtm)
    pre_a = np.interp(t_ref, ref["t"], ref["acc"])
    pre_b = np.interp(t_ref, ref["t"], ref["baro"])

    t_tgt = np.arange(0.0, target["t"][-1] + 1e-12, dtm)
    a_tgt = np.interp(t_tgt, target["t"], target["acc"])
    b_tgt = np.interp(t_tgt, target["t"], target["baro"])

    delta_b = float(pre_b[-1] - b_tgt[0])

    out_a = np.concatenate([pre_a, a_tgt])
    out_b = np.concatenate([pre_b, b_tgt + delta_b])
    out_t = np.arange(0.0, len(out_a) * dtm, dtm)

    return out_t, out_a, out_b, delta_b, len(pre_a)


def estimate_tau_seed(f0: dict[str, np.ndarray], f1: dict[str, np.ndarray]) -> float:
    t0, bv0 = baro_vel_preapo(f0)
    t1, bv1 = baro_vel_preapo(f1)
    return float(t0[np.argmax(bv0)] - t1[np.argmax(bv1)])


def choose_tau_for_control_match(f0: dict[str, np.ndarray], f1: dict[str, np.ndarray]) -> tuple[float, float]:
    u0, _, _, tc0 = run_firmware_like(f0["t"], f0["acc"], f0["baro"])
    _ = u0
    best = None
    for tau in np.linspace(0.05, 0.60, 56):
        ts, as_, bs, _, _ = stitch_with_baro_continuity(f0, f1, float(tau))
        _, _, _, tc = run_firmware_like(ts, as_, bs)
        if not np.isfinite(tc):
            continue
        err = abs(tc - tc0)
        if best is None or err < best[0]:
            best = (err, float(tau), float(tc))
    if best is None:
        raise RuntimeError("Unable to determine tau")
    return best[1], tc0


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    out_dir = Path(__file__).resolve().parent

    f0 = load(root / "flight_data" / "flight_0_20260405_194019.csv")
    f1 = load(root / "flight_data" / "flight_1_20260405_200837.csv")

    tau_seed = estimate_tau_seed(f0, f1)
    tau_opt, tc0 = choose_tau_for_control_match(f0, f1)

    ts, as_, bs, delta_b, npre = stitch_with_baro_continuity(f0, f1, tau_opt)

    u0, a0u, x0, tc0m = run_firmware_like(f0["t"], f0["acc"], f0["baro"])
    u1, a1u, x1, tc1 = run_firmware_like(f1["t"], f1["acc"], f1["baro"])
    us, asu, xs, tcs = run_firmware_like(ts, as_, bs)

    # Full replay plot (position/velocity/accel)
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    for name, u, x, c in [
        ("Flight0", u0, x0, "tab:blue"),
        ("Flight1", u1, x1, "tab:orange"),
        (f"Flight1 stitched tau={tau_opt:.2f}", us, xs, "tab:green"),
    ]:
        m = u <= 8.0
        ax[0].plot(u[m], x[m, 0], color=c, lw=2, label=f"{name} replay altitude")

    ax[0].plot(f0["t"][f0["t"] <= 8], f0["baro"][f0["t"] <= 8], color="tab:blue", ls="--", alpha=0.4, label="Flight0 raw baro")
    ax[0].plot(f1["t"][f1["t"] <= 8], f1["baro"][f1["t"] <= 8], color="tab:orange", ls="--", alpha=0.4, label="Flight1 raw baro")
    ax[0].set_ylabel("Position / Altitude (m)")
    ax[0].set_title("Full replay comparison with continuity-corrected stitched pre-roll")
    ax[0].grid(alpha=0.25)
    ax[0].legend(fontsize=8, ncol=2)

    for name, u, x, c, tc in [
        ("Flight0", u0, x0, "tab:blue", tc0m),
        ("Flight1", u1, x1, "tab:orange", tc1),
        (f"Flight1 stitched tau={tau_opt:.2f}", us, xs, "tab:green", tcs),
    ]:
        m = u <= 8.0
        ax[1].plot(u[m], x[m, 1], color=c, lw=2, label=f"{name} replay velocity")
        ax[1].axvline(tc, color=c, ls=":", alpha=0.6)

    ax[1].axhline(VEL_CONTROL_START, color="k", ls="-.", alpha=0.6, label="VEL_CONTROL_START")
    ax[1].set_ylabel("Velocity (m/s)")
    ax[1].grid(alpha=0.25)
    ax[1].legend(fontsize=8, ncol=2)

    for name, u, a, c in [
        ("Flight0", u0, a0u, "tab:blue"),
        ("Flight1", u1, a1u, "tab:orange"),
        (f"Flight1 stitched tau={tau_opt:.2f}", us, asu, "tab:green"),
    ]:
        m = u <= 8.0
        ax[2].plot(u[m], a[m], color=c, lw=1.8, label=f"{name} replay accel input")

    ax[2].set_ylabel("Acceleration (m/s^2)")
    ax[2].set_xlabel("Time from BOOST log start (s)")
    ax[2].grid(alpha=0.25)
    ax[2].legend(fontsize=8, ncol=2)

    fig.tight_layout()
    out_full = out_dir / "stitched_full_replay.png"
    fig.savefig(out_full, dpi=170)
    plt.close(fig)

    # Timing zoom plot
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    for name, u, x, c in [
        ("Flight0", u0, x0, "tab:blue"),
        ("Flight1", u1, x1, "tab:orange"),
        (f"Flight1 stitched tau={tau_opt:.2f}", us, xs, "tab:green"),
    ]:
        m = u <= 3.5
        ax[0].plot(u[m], x[m, 1], color=c, lw=2, label=name)

    ax[0].axhline(VEL_CONTROL_START, color="k", ls="-.", alpha=0.6, label="VEL_CONTROL_START")
    for tc, c in [(tc0m, "tab:blue"), (tc1, "tab:orange"), (tcs, "tab:green")]:
        ax[0].axvline(tc, color=c, ls=":", alpha=0.6)

    ax[0].set_ylabel("Model velocity (m/s)")
    ax[0].set_title("Control timing match with stitched pre-roll")
    ax[0].grid(alpha=0.25)
    ax[0].legend(fontsize=8)

    tu = np.arange(0.0, 3.5, 0.01)
    for name, t, a, c in [
        ("Flight0", f0["t"], f0["acc"], "tab:blue"),
        ("Flight1", f1["t"], f1["acc"], "tab:orange"),
        (f"Flight1 stitched tau={tau_opt:.2f}", ts, as_, "tab:green"),
    ]:
        au = np.interp(tu, t, a)
        dv = np.cumsum(au) * 0.01
        ax[1].plot(tu, dv, color=c, lw=2, label=name)

    for tc, c in [(tc0m, "tab:blue"), (tc1, "tab:orange"), (tcs, "tab:green")]:
        ax[1].axvline(tc, color=c, ls=":", alpha=0.6)

    ax[1].set_ylabel("Integrated dv (m/s)")
    ax[1].set_xlabel("Time (s)")
    ax[1].grid(alpha=0.25)
    ax[1].legend(fontsize=8)

    fig.tight_layout()
    out_zoom = out_dir / "stitched_timing_zoom.png"
    fig.savefig(out_zoom, dpi=170)
    plt.close(fig)

    print(f"tau_seed={tau_seed:.3f}")
    print(f"tau_opt={tau_opt:.3f}")
    print(f"baro_offset={delta_b:.3f}")
    print(f"pre_samples={npre}")
    print(f"control_model_f0={tc0m:.3f}")
    print(f"control_model_f1={tc1:.3f}")
    print(f"control_model_stitched={tcs:.3f}")
    print(f"control_logged_f0={control_logged(f0):.3f}")
    print(f"control_logged_f1={control_logged(f1):.3f}")
    print(f"wrote={out_full}")
    print(f"wrote={out_zoom}")


if __name__ == "__main__":
    main()
