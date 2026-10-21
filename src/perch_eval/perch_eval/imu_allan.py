#!/usr/bin/env python3
"""Allan deviation analysis of a D435i IMU recording made by imu_log.py.

Reads the CSV, puts the samples on a uniform grid using the camera clock,
computes the overlapping Allan deviation per axis for the gyro and the
accelerometer, and extracts the three numbers an IMU-fusion estimator wants:

  N  noise density        the white-noise level; sigma(tau) = N / sqrt(tau)
                          on the -1/2 slope. Units: rad/s/sqrt(Hz) (gyro),
                          m/s^2/sqrt(Hz) (accel). Read at tau = 1 s.
  B  bias instability     the floor of the curve; the best the sensor can do
                          by averaging. Reported as min(sigma) and as
                          min(sigma)/0.664 (the usual 1/f convention).
  K  bias random walk     how fast the zero point wanders; sigma(tau) =
                          K * sqrt(tau/3) on the +1/2 slope. Units:
                          rad/s^2/sqrt(Hz) (gyro), m/s^3/sqrt(Hz) (accel).
                          Read at tau = 3 s.

Usage:
  python3 -m perch_eval.imu_allan RECORDING.csv [--out DIR] [--max-tau S]
  python3 -m perch_eval.imu_allan --self-test

Decisions made here, and why (these are the parts worth being able to defend):

* Time base is the camera clock (ts_ms), never host arrival time. The
  overnight run showed host jitter of up to 20 ms on a 5 ms stream.
* ts_ms is a 32-bit microsecond counter and wraps every 4294.967 s. It is
  unwrapped here. Any backward step of more than half the counter range is
  a wrap; nothing else is.
* The samples are placed on a uniform grid at the measured median period,
  not the nominal rate. The accel on this unit runs at ~202 Hz, not 200,
  and the period shifts with temperature.
* Gaps of a few periods (a sample lost inside the camera) are bridged by
  linear interpolation so the grid stays exact in time. Gaps longer than
  --max-gap split the recording; the longest continuous piece is analysed.
* The mean of each axis is removed before integrating. A constant does not
  change the Allan variance, but integrating gravity (9.8 m/s^2) for eight
  hours without removing it destroys float64 precision in the differences.
* N, B and K come from one weighted least-squares fit of the physical model
  sigma^2(tau) = N^2/tau + 0.441*B^2 + K^2*tau/3 to the whole usable curve,
  with non-negative coefficients. This replaced reading N at tau = 1 s and
  K at tau = 3 s off the curve, and replaced picking slope regions by eye or
  by local gradient: on synthetic data the single-point and slope-region
  methods scattered K by +-12 %, because the +1/2 region lives where the
  statistics are weakest, while the whole-curve fit uses every point. The
  slope-region N and the raw sigma(1 s) are still printed as cross-checks.
* Every fitted number carries a 1-sigma uncertainty. Adjacent Allan points
  share almost all their data, so the covariance is inflated by the number
  of points per octave; the self-test confirmed the formal value is 2-3x too
  small without that.
* K is reported as *resolved* only if the random-walk term dominates the
  model somewhere inside the usable tau range. Otherwise it is an upper
  bound and is labelled as one. B is "resolved" only if the curve's minimum
  is not the last usable point.
* The error band is the standard non-overlapping estimate
  1/sqrt(2*(n_clusters - 1)); it is conservative for the overlapping
  estimator. Points with a relative error above --max-rel-err are shown
  but never used in the fit.
* --self-test generates a synthetic IMU with known N and K and checks that
  both are recovered within their own 2-sigma, before any real number is
  trusted; it also checks the unwrap and gap bridging on a synthetic clock.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

WRAP_US = 2 ** 32
AXES = ("x", "y", "z")
BIAS_INSTABILITY_FACTOR = 0.664  # sqrt(2 ln 2 / pi)

UNITS = {
    "gyro": {"rate": "rad/s", "N": "rad/s/sqrt(Hz)", "K": "rad/s^2/sqrt(Hz)"},
    "accel": {"rate": "m/s^2", "N": "m/s^2/sqrt(Hz)", "K": "m/s^3/sqrt(Hz)"},
}


# ----------------------------------------------------------------------------
# loading and time base
# ----------------------------------------------------------------------------

def load_recording(path: Path) -> dict:
    """Return {'accel': {...}, 'gyro': {...}} with ts_us, frame, xyz arrays."""
    import pandas as pd  # only needed for real files, not the self-test

    df = pd.read_csv(
        path,
        usecols=["stream", "frame_number", "ts_ms", "x", "y", "z"],
        dtype={
            "stream": "category",
            "frame_number": np.int64,
            "ts_ms": np.float64,
            "x": np.float32,
            "y": np.float32,
            "z": np.float32,
        },
    )
    out = {}
    for s in ("accel", "gyro"):
        d = df[df["stream"] == s]
        if len(d) == 0:
            continue
        out[s] = {
            "frame": d["frame_number"].to_numpy(),
            "ts_us": np.rint(d["ts_ms"].to_numpy() * 1e3).astype(np.int64),
            "xyz": d[["x", "y", "z"]].to_numpy(dtype=np.float64),
        }
    return out


def unwrap_32bit(ts_us: np.ndarray) -> tuple[np.ndarray, int]:
    """Undo wraps of a 32-bit microsecond counter. Returns (ts, n_wraps)."""
    d = np.diff(ts_us)
    wraps = d < -(WRAP_US // 2)
    corr = np.concatenate([[0], np.cumsum(wraps)]) * WRAP_US
    return ts_us + corr, int(wraps.sum())


@dataclass
class GridInfo:
    period_us: float
    n_in: int
    n_grid: int
    n_wraps: int
    n_gaps_bridged: int
    n_samples_bridged: int
    n_repeats: int
    n_segments: int
    segment_used: int
    segment_start_frame: int
    segment_end_frame: int
    duration_s: float


def to_uniform_grid(ts_us: np.ndarray, frame: np.ndarray, xyz: np.ndarray,
                    max_gap_s: float) -> tuple[np.ndarray, GridInfo]:
    """Place samples on a uniform grid at the measured median period.

    Short gaps are bridged by linear interpolation. Gaps longer than
    max_gap_s split the data; the longest segment is returned.
    """
    ts, n_wraps = unwrap_32bit(ts_us)
    d = np.diff(ts)
    period = float(np.median(d))
    max_gap_us = max_gap_s * 1e6

    # split on long gaps (a startup clock jump, a pause, anything abnormal)
    breaks = np.flatnonzero(d > max_gap_us) + 1
    bounds = np.concatenate([[0], breaks, [len(ts)]])
    seg_lengths = np.diff(bounds)
    seg = int(np.argmax(seg_lengths))
    a, b = int(bounds[seg]), int(bounds[seg + 1])

    ts_s, xyz_s, fr_s = ts[a:b], xyz[a:b], frame[a:b]
    ds = np.diff(ts_s)
    k = np.rint(ds / period).astype(np.int64)      # periods between samples
    n_repeats = int((k < 1).sum())                 # repeats / backwards steps
    k = np.maximum(k, 1)
    gaps = k > 1
    idx = np.concatenate([[0], np.cumsum(k)])       # grid index of each sample
    n_grid = int(idx[-1]) + 1
    grid = np.arange(n_grid)
    out = np.empty((n_grid, 3))
    for a_i in range(3):
        out[:, a_i] = np.interp(grid, idx, xyz_s[:, a_i])

    info = GridInfo(
        period_us=period,
        n_in=len(ts),
        n_grid=n_grid,
        n_wraps=n_wraps,
        n_gaps_bridged=int(gaps.sum()),
        n_samples_bridged=int((k[gaps] - 1).sum()),
        n_repeats=n_repeats,
        n_segments=len(seg_lengths),
        segment_used=seg + 1,
        segment_start_frame=int(fr_s[0]),
        segment_end_frame=int(fr_s[-1]),
        duration_s=(n_grid - 1) * period / 1e6,
    )
    return out, info


# ----------------------------------------------------------------------------
# Allan deviation
# ----------------------------------------------------------------------------

def tau_list(n: int, tau0: float, max_tau: float | None, per_decade: int = 20) -> np.ndarray:
    """Log-spaced averaging factors m, from 1 up to n/10 (or max_tau)."""
    m_max = n // 10
    if max_tau is not None:
        m_max = min(m_max, int(max_tau / tau0))
    if m_max < 2:
        raise ValueError("recording too short for an Allan curve")
    decades = math.log10(m_max)
    m = np.unique(np.rint(np.logspace(0, decades, int(decades * per_decade) + 1)).astype(np.int64))
    return m[m >= 1]


def allan_deviation(x: np.ndarray, tau0: float, m_list: np.ndarray):
    """Overlapping Allan deviation of a rate signal x (n, 3).

    Returns tau (nt,), adev (nt, 3), rel_err (nt,).
    """
    n = len(x)
    # integrate the mean-removed signal; theta[0] = 0
    theta = np.empty((n + 1, 3))
    theta[0] = 0.0
    np.cumsum(x - x.mean(axis=0), axis=0, out=theta[1:])
    theta *= tau0

    taus, adev, rel = [], [], []
    for m in m_list:
        if 2 * m >= n:
            break
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        avar = np.einsum("ij,ij->j", d, d) / (d.shape[0] * 2.0 * (m * tau0) ** 2)
        taus.append(m * tau0)
        adev.append(np.sqrt(avar))
        n_clusters = n / m
        rel.append(1.0 / math.sqrt(2.0 * max(n_clusters - 1.0, 1.0)))
    return np.array(taus), np.array(adev), np.array(rel)


# ----------------------------------------------------------------------------
# parameter extraction
# ----------------------------------------------------------------------------

@dataclass
class SlopeFit:
    value_at_1s: float      # 10**intercept of the slope-constrained fit
    tau_lo: float
    tau_hi: float
    n_points: int


def longest_true_run(mask: np.ndarray) -> np.ndarray:
    """Boolean mask of the longest contiguous run of True."""
    best_len, best_start = 0, -1
    i = 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            if j - i > best_len:
                best_len, best_start = j - i, i
            i = j
        else:
            i += 1
    out = np.zeros_like(mask)
    if best_len:
        out[best_start:best_start + best_len] = True
    return out


def fit_slope_region(tau, sigma, rel_err, slope, tau_lo=None, tau_hi=None,
                     tol=0.15, max_rel_err=0.2, min_points=3) -> SlopeFit | None:
    """Fit log10(sigma) = c + slope*log10(tau) over the longest run of points
    whose local slope matches. Returns value at tau = 1 s (10**c)."""
    lt, ls = np.log10(tau), np.log10(sigma)
    local = np.gradient(ls, lt)
    sel = (np.abs(local - slope) < tol) & (rel_err < max_rel_err)
    if tau_lo is not None:
        sel &= tau >= tau_lo
    if tau_hi is not None:
        sel &= tau <= tau_hi
    sel = longest_true_run(sel)
    if sel.sum() < min_points:
        return None
    c = float(np.mean(ls[sel] - slope * lt[sel]))
    return SlopeFit(10 ** c, float(tau[sel].min()), float(tau[sel].max()), int(sel.sum()))


FLICKER_FACTOR = 2.0 * math.log(2.0) / math.pi   # 0.441: flat term of 1/f noise is this * B^2


def fit_noise_model(tau, sigma, rel_err, max_rel_err):
    """Weighted least squares of the physical model over the whole usable curve:

        sigma^2(tau) = a/tau + b + c*tau,   a = N^2, b = 0.441*B^2, c = K^2/3

    It is linear in (a, b, c), so this is a weighted linear least squares with
    a non-negativity constraint, solved by trying every subset of terms and
    keeping the best fit whose coefficients are all >= 0. Weights equalise the
    relative error of each point (the variance's relative error is ~2*rel_err),
    so the dense short-tau points cannot drown the sparse long-tau ones.

    Uncertainties: adjacent Allan points share almost all of their data, so
    the N points of the curve are far fewer than N independent measurements.
    Points within an octave of tau are treated as one — the covariance is
    inflated by the number of points per octave. Checked against the
    self-test's scatter across axes and seeds; without it the formal
    1-sigma is 2-3x too small.
    """
    use = rel_err < max_rel_err
    if use.sum() < 6:
        return None
    t, s2, r = tau[use], sigma[use] ** 2, rel_err[use]
    w = 1.0 / (2.0 * r * s2)                       # 1 / absolute error of sigma^2
    per_octave = math.log10(2.0) / max(float(np.median(np.diff(np.log10(t)))), 1e-9)
    cov_inflation = max(per_octave, 1.0)
    cols = {"a": 1.0 / t, "b": np.ones_like(t), "c": t}
    best = None
    for subset in (("a",), ("b",), ("c",), ("a", "b"), ("a", "c"), ("b", "c"), ("a", "b", "c")):
        A = np.column_stack([cols[k] for k in subset]) * w[:, None]
        coef, *_ = np.linalg.lstsq(A, s2 * w, rcond=None)
        if np.any(coef < 0):
            continue
        model = sum(cc * cols[k] for cc, k in zip(coef, subset))
        resid = np.sqrt(np.mean((np.log10(np.maximum(model, 1e-300)) - np.log10(s2)) ** 2)) / 2.0
        if best is None or resid < best["resid"]:
            # 1-sigma uncertainty of each coefficient from the weighted normal
            # equations; valid because the weights are 1/(absolute error)
            cov = np.linalg.pinv(A.T @ A) * cov_inflation
            full = {k: 0.0 for k in cols}
            full.update(dict(zip(subset, coef)))
            sd = {k: 0.0 for k in cols}
            sd.update({k: math.sqrt(max(cov[i, i], 0.0)) for i, k in enumerate(subset)})
            best = {"a": full["a"], "b": full["b"], "c": full["c"],
                    "sd_a": sd["a"], "sd_b": sd["b"], "sd_c": sd["c"],
                    "resid": float(resid), "n": int(use.sum())}
    return best


def extract_parameters(tau, adev, rel_err, max_rel_err):
    """Per-axis N, B, K from an Allan curve. Returns list of dicts (x, y, z).

    Primary numbers come from the whole-curve model fit. The slope-region
    fit for N and the raw sigma(1 s) are kept as cross-checks: when the
    three agree the -1/2 region is clean; when they do not, look at the plot.
    """
    results = []
    usable = rel_err < max_rel_err
    for a in range(3):
        s = adev[:, a]
        # floor of the curve over the statistically usable part
        i_min = int(np.argmin(np.where(usable, s, np.inf)))
        tau_min, s_min = float(tau[i_min]), float(s[i_min])
        floor_resolved = bool(usable[i_min] and i_min < int(usable.sum()) - 1)

        model = fit_noise_model(tau, s, rel_err, max_rel_err)
        n_slope = fit_slope_region(tau, s, rel_err, -0.5, tau_hi=tau_min, max_rel_err=max_rel_err)
        i1 = int(np.argmin(np.abs(np.log10(tau))))

        if model:
            a_, b_, c_ = model["a"], model["b"], model["c"]
            N = math.sqrt(a_) if a_ > 0 else None
            K = math.sqrt(3.0 * c_) if c_ > 0 else None
            B_model = math.sqrt(b_ / FLICKER_FACTOR) if b_ > 0 else None
            # 1-sigma of the square roots by the delta method: d sqrt(a) = da / (2 sqrt a)
            N_sd = model["sd_a"] / (2.0 * N) if N else None
            K_sd = 3.0 * model["sd_c"] / (2.0 * K) if K else None
            # K is only *resolved* if the random-walk term actually dominates
            # somewhere inside the usable range; otherwise it is an upper bound
            t_hi = float(tau[usable].max())
            k_resolved = bool(K and c_ * t_hi > (a_ / t_hi + b_))
            resid, npts = model["resid"], model["n"]
        else:
            N = K = B_model = N_sd = K_sd = None
            resid, npts, k_resolved = None, 0, False

        results.append({
            "N": N,
            "N_sd": N_sd,
            "N_slope_fit": n_slope.value_at_1s if n_slope else None,
            "N_slope_tau_range_s": [n_slope.tau_lo, n_slope.tau_hi] if n_slope else None,
            "sigma_at_1s": float(s[i1]),
            "B_min_sigma": s_min,
            "B_tau_s": tau_min,
            "B": s_min / BIAS_INSTABILITY_FACTOR,
            "B_model": B_model,
            "B_floor_resolved": floor_resolved,
            "K": K,
            "K_sd": K_sd,
            "K_resolved": k_resolved,
            "model_fit_rms_log10": resid,
            "model_fit_points": npts,
        })
    return results


# ----------------------------------------------------------------------------
# reporting
# ----------------------------------------------------------------------------

def fmt(v, unit="", sd=None):
    if v is None:
        return "not resolved"
    pm = f" (±{sd / v:.1%})" if sd else ""
    return f"{v:.4g}{pm} {unit}".rstrip()


def print_report(name, info: GridInfo, x: np.ndarray, params, max_tau):
    u = UNITS[name]
    print(f"\n{name}")
    print(f"  grid       {info.n_grid} samples at {info.period_us:.1f} us "
          f"({1e6 / info.period_us:.2f} Hz), {info.duration_s / 3600:.2f} h")
    print(f"  clock      {info.n_wraps} wraps unwrapped; {info.n_gaps_bridged} gaps "
          f"({info.n_samples_bridged} samples) bridged; {info.n_repeats} repeats; "
          f"segment {info.segment_used} of {info.n_segments} used "
          f"(frames {info.segment_start_frame}-{info.segment_end_frame})")
    mean = x.mean(axis=0)
    print(f"  mean       x {mean[0]:+.5f}  y {mean[1]:+.5f}  z {mean[2]:+.5f} {u['rate']}"
          + (f"   |mean| {np.linalg.norm(mean):.4f} (gravity check)" if name == "accel" else ""))
    print(f"  max tau    {max_tau:.0f} s")
    for a, p in zip(AXES, params):
        print(f"  {a}: N {fmt(p['N'], u['N'], p['N_sd'])}   cross-checks: slope fit {fmt(p['N_slope_fit'])}, "
              f"sigma(1s) {p['sigma_at_1s']:.4g}")
        print(f"     B {p['B']:.4g} {u['rate']}  (min sigma {p['B_min_sigma']:.4g} at tau {p['B_tau_s']:.3g} s"
              + (f"; model {p['B_model']:.4g}" if p["B_model"] else "; model: no flat term needed")
              + ("" if p["B_floor_resolved"] else "; floor NOT resolved within max tau") + ")")
        print(f"     K {fmt(p['K'], u['K'], p['K_sd'])}"
              + ("" if p["K_resolved"] else "  (UPPER BOUND: random walk never dominates within max tau)" if p["K"]
                 else "  (model needs no random-walk term within max tau)"))
        if p["model_fit_rms_log10"] is not None:
            print(f"     model fit over {p['model_fit_points']} points, "
                  f"rms log10 residual {p['model_fit_rms_log10']:.3f} "
                  f"({(10 ** p['model_fit_rms_log10'] - 1) * 100:.1f} %)")


def summary_block(params):
    """Max over axes, which is the conservative single number estimators want."""
    def mx(key):
        vals = [p[key] for p in params if p[key] is not None]
        return max(vals) if vals else None
    return {"N_max": mx("N"), "B_max": mx("B"), "K_max": mx("K")}


def plot(curves: dict, out_png: Path, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(curves), figsize=(6.5 * len(curves), 5), squeeze=False)
    for ax, (name, c) in zip(axes[0], curves.items()):
        tau, adev, rel, params = c["tau"], c["adev"], c["rel_err"], c["params"]
        u = UNITS[name]
        for a, color in zip(range(3), ("C0", "C1", "C2")):
            s = adev[:, a]
            ax.loglog(tau, s, color=color, label=AXES[a])
            ax.fill_between(tau, s * (1 - rel), s * (1 + rel), color=color, alpha=0.15, lw=0)
            p = params[a]
            if p["N"] or p["K"] or p["B_model"]:
                model = np.zeros_like(tau)
                if p["N"]:
                    model += p["N"] ** 2 / tau
                if p["B_model"]:
                    model += FLICKER_FACTOR * p["B_model"] ** 2
                if p["K"]:
                    model += p["K"] ** 2 * tau / 3.0
                ax.loglog(tau, np.sqrt(model), color=color, ls="--", lw=1)
            ax.plot(p["B_tau_s"], p["B_min_sigma"], "o", color=color, ms=4)
        ax.axvline(1.0, color="gray", lw=0.5)
        ax.axvline(3.0, color="gray", lw=0.5)
        ax.set_xlabel("tau (s)")
        ax.set_ylabel(f"Allan deviation ({u['rate']})")
        ax.set_title(f"{name}   solid: measured   dashed: fitted N, B, K model")
        ax.grid(True, which="both", lw=0.3)
        ax.legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"\nplot      {out_png}")


# ----------------------------------------------------------------------------
# self-test with known parameters
# ----------------------------------------------------------------------------

def synth_imu(fs, hours, N, K, seed=0):
    """White noise with density N plus a bias random walk with density K."""
    rng = np.random.default_rng(seed)
    n = int(fs * hours * 3600)
    dt = 1.0 / fs
    white = rng.standard_normal((n, 3)) * N * math.sqrt(fs)
    walk = np.cumsum(rng.standard_normal((n, 3)) * K * math.sqrt(dt), axis=0)
    return white + walk


def self_test(args):
    fs, hours = 200.0, 3.0
    truth = {"gyro": (1.2e-3, 2.0e-5), "accel": (1.5e-2, 3.0e-4)}
    ok = True
    print("self-test: synthetic IMU, 3 h at 200 Hz, known N and K, no flicker term")
    for name, (N, K) in truth.items():
        x = synth_imu(fs, hours, N, K, seed=1 if name == "gyro" else 2)
        tau0 = 1.0 / fs
        tau, adev, rel = allan_deviation(x, tau0, tau_list(len(x), tau0, None))
        params = extract_parameters(tau, adev, rel, args.max_rel_err)
        for a, p in zip(AXES, params):
            n_err = (p["N"] / N - 1) if p["N"] else float("nan")
            k_err = (p["K"] / K - 1) if p["K"] else float("nan")
            # pass if the truth lies within 2 sigma of the fit (and never worse than 5 % for N)
            n_ok = p["N"] is not None and abs(n_err) < max(0.05, 2 * p["N_sd"] / p["N"])
            k_ok = p["K"] is not None and p["K_resolved"] and abs(k_err) < 2 * p["K_sd"] / p["K"]
            flag = "ok" if (n_ok and k_ok) else "FAIL"
            ok &= flag == "ok"
            print(f"  {name} {a}: N {fmt(p['N'], sd=p['N_sd'])} (true {N:.4g}, {n_err:+.1%})   "
                  f"K {fmt(p['K'], sd=p['K_sd'])} (true {K:.4g}, {k_err:+.1%})   "
                  f"fit rms {p['model_fit_rms_log10']:.3f}   {flag}")
    # the time-base code, on a synthetic clock with one wrap and two gaps
    period = 4943
    ts = np.arange(0, 2_000_000, dtype=np.int64) * period
    ts = np.delete(ts, [5000, 5001, 70000])            # a 2-sample gap and a 1-sample gap
    ts_w = ts % WRAP_US                                # wrap the counter
    xyz = np.random.default_rng(3).standard_normal((len(ts), 3))
    frame = np.arange(len(ts))
    grid, info = to_uniform_grid(ts_w, frame, xyz, max_gap_s=1.0)
    wrap_ok = info.n_wraps == (ts[-1] // WRAP_US)
    gap_ok = info.n_gaps_bridged == 2 and info.n_samples_bridged == 3 and info.n_grid == 2_000_000
    print(f"  time base: {info.n_wraps} wraps {'ok' if wrap_ok else 'FAIL'}; "
          f"{info.n_gaps_bridged} gaps / {info.n_samples_bridged} samples bridged, "
          f"grid {info.n_grid} {'ok' if gap_ok else 'FAIL'}")
    ok &= wrap_ok and gap_ok
    print("self-test", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="?", type=Path, help="recording from imu_log.py")
    ap.add_argument("--out", type=Path, help="output folder (default: next to the CSV)")
    ap.add_argument("--max-tau", type=float, default=None,
                    help="longest averaging time in s (default: duration/10)")
    ap.add_argument("--max-gap", type=float, default=1.0,
                    help="gap in s above which the recording is split (default 1.0)")
    ap.add_argument("--max-rel-err", type=float, default=0.2,
                    help="points with a larger relative error are not used in fits")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test(args)
    if not args.csv:
        ap.error("a CSV path or --self-test is required")

    out_dir = args.out or args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.csv.stem

    print(f"reading   {args.csv}")
    rec = load_recording(args.csv)
    result = {"source": str(args.csv), "units": UNITS, "sensors": {}, "summary": {}}
    curves = {}
    for name in ("gyro", "accel"):
        if name not in rec:
            continue
        r = rec[name]
        grid, info = to_uniform_grid(r["ts_us"], r["frame"], r["xyz"], args.max_gap)
        tau0 = info.period_us / 1e6
        m = tau_list(len(grid), tau0, args.max_tau)
        tau, adev, rel = allan_deviation(grid, tau0, m)
        params = extract_parameters(tau, adev, rel, args.max_rel_err)
        print_report(name, info, grid, params, tau[-1])
        result["sensors"][name] = {
            "grid": asdict(info),
            "mean": grid.mean(axis=0).tolist(),
            "per_axis": dict(zip(AXES, params)),
            "max_over_axes": summary_block(params),
            "curve": {"tau_s": tau.tolist(), "adev": adev.tolist(), "rel_err": rel.tolist()},
        }
        result["summary"][name] = summary_block(params)
        curves[name] = {"tau": tau, "adev": adev, "rel_err": rel, "params": params}

    out_json = out_dir / f"{stem}_allan.json"
    out_json.write_text(json.dumps(result, indent=2))
    print(f"\njson      {out_json}")
    plot(curves, out_dir / f"{stem}_allan.png", stem)

    print("\nsummary (max over axes; the conservative values for an estimator config)")
    for name, s in result["summary"].items():
        u = UNITS[name]
        print(f"  {name}: noise density {fmt(s['N_max'], u['N'])}   "
              f"random walk {fmt(s['K_max'], u['K'])}   bias instability {fmt(s['B_max'], u['rate'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
