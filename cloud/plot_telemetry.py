"""Plot and analyze MC Aero telemetry NDJSON.

Usage: python3 plot_telemetry.py <dir-of-ndjson> [out-dir]

Produces PNGs (RSC, motion, per-bearing rpm/thrust time series, and a
thrust-vs-rpm characterization grid with linear fits) and prints a fit report.
"""

import glob
import json
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def num(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return float(v)
    return None


def load(directory):
    files = sorted(glob.glob(os.path.join(directory, "**", "*.ndjson"), recursive=True))
    if not files:
        files = sorted(glob.glob(os.path.join(directory, "*.ndjson")))
    recs = []
    for path in files:
        with open(path, "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    try:
                        recs.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
    recs.sort(key=lambda r: r.get("timestampMs") or 0)
    return recs, len(files)


def rel_seconds(recs):
    t0 = next((r["timestampMs"] for r in recs if isinstance(r.get("timestampMs"), (int, float))), 0)
    return [((r.get("timestampMs") or t0) - t0) / 1000.0 for r in recs]


def main(indir, outdir):
    os.makedirs(outdir, exist_ok=True)
    recs, nfiles = load(indir)
    if not recs:
        print("no records in", indir)
        return
    t = rel_seconds(recs)

    # ---- RSC target speed by axis ----
    axes = ["mainLift", "forwardBack", "yaw", "leftRight", "upDown"]
    fig, ax = plt.subplots(figsize=(11, 4))
    for axis in axes:
        y = [num((r.get("actuators") or {}).get("rsc", {}).get(axis, {}).get("getTargetSpeed")) for r in recs]
        if any(v is not None for v in y):
            ax.plot(t, [v if v is not None else np.nan for v in y], label=axis, linewidth=1)
    ax.set_title("RSC target speed by axis")
    ax.set_xlabel("time (s)"); ax.set_ylabel("rpm"); ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "rsc.png"), dpi=110); plt.close(fig)

    # ---- motion ----
    height = [num((r.get("sensors") or {}).get("altitude", {}).get("height")) for r in recs]
    vspeed = [num((r.get("sensors") or {}).get("altitude", {}).get("verticalSpeed")) for r in recs]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    a1.plot(t, [v if v is not None else np.nan for v in height], color="tab:blue")
    a1.set_ylabel("height (Y)"); a1.grid(True, alpha=0.3); a1.set_title("Altitude & vertical speed")
    a2.plot(t, [v if v is not None else np.nan for v in vspeed], color="tab:orange")
    a2.set_ylabel("vertical speed"); a2.set_xlabel("time (s)"); a2.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "motion.png"), dpi=110); plt.close(fig)

    # ---- velocities ----
    axis_series = defaultdict(lambda: [np.nan] * len(recs))
    for i, r in enumerate(recs):
        for v in (r.get("sensors") or {}).get("velocity", []) or []:
            a = str(v.get("axis"))
            val = num(v.get("velocity"))
            if val is not None:
                axis_series[a][i] = val
    fig, ax = plt.subplots(figsize=(11, 4))
    for a in sorted(axis_series):
        ax.plot(t, axis_series[a], label="v" + a, linewidth=1)
    ax.set_title("Velocity by axis"); ax.set_xlabel("time (s)"); ax.set_ylabel("blocks/s")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "velocity.png"), dpi=110); plt.close(fig)

    # ---- bearings: gather ----
    names = []
    rpm = defaultdict(list)
    thr = defaultdict(list)
    rpm_t = defaultdict(list)
    for i, r in enumerate(recs):
        for b in (r.get("actuators") or {}).get("bearings", []) or []:
            nm = b.get("name", "?")
            if nm not in names:
                names.append(nm)
            rr = num(b.get("getRotationSpeed"))
            tt = num(b.get("getThrust"))
            if rr is not None:
                rpm[nm].append(rr); rpm_t[nm].append(t[i])
            if rr is not None and tt is not None:
                thr[nm].append((rr, tt))
    names.sort()

    # rpm vs time
    fig, ax = plt.subplots(figsize=(11, 5))
    for nm in names:
        ax.plot(rpm_t[nm], rpm[nm], label=nm.replace("_propeller_bearing_", ""), linewidth=0.8)
    ax.set_title("Bearing rotation speed vs time"); ax.set_xlabel("time (s)"); ax.set_ylabel("rpm")
    ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "bearing_rpm.png"), dpi=110); plt.close(fig)

    # ---- sweep spin-up transients (if this capture is a sweep) ----
    sweep_recs = [r for r in recs if isinstance(r.get("sweep"), dict)]
    if sweep_recs:
        steps = defaultdict(list)
        for r in sweep_recs:
            s = r["sweep"]
            steps[(str(s.get("axis")), num(s.get("target")) or 0)].append(r)
        # for each axis, take the step to its largest target
        best_target = {}
        for axis, target in steps:
            if target and target > best_target.get(axis, 0):
                best_target[axis] = target
        fig, ax = plt.subplots(figsize=(11, 5))
        for axis, target in sorted(best_target.items()):
            group = sorted(steps[(axis, target)], key=lambda r: num(r["sweep"].get("elapsedMs")) or 0)
            series = defaultdict(list)
            for r in group:
                e = (num(r["sweep"].get("elapsedMs")) or 0) / 1000.0
                for b in (r.get("actuators") or {}).get("bearings", []) or []:
                    rr = num(b.get("getRotationSpeed"))
                    if rr is not None:
                        series[b.get("role") or b.get("name")].append((e, rr))
            # plot the driven bearing with the largest final |rpm|
            best_b, best_val = None, 0.0
            for nm, pts in series.items():
                if pts and abs(pts[-1][1]) > best_val:
                    best_b, best_val = nm, abs(pts[-1][1])
            if best_b:
                pts = series[best_b]
                ax.plot([p[0] for p in pts], [p[1] for p in pts],
                        label=f"{axis}: {best_b} -> {target:.0f}", linewidth=1)
        ax.set_title("Spin-up: bearing rpm vs time since step (largest step per axis)")
        ax.set_xlabel("time since step (s)"); ax.set_ylabel("rpm")
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "spinup.png"), dpi=110); plt.close(fig)

    # thrust vs rpm characterization grid + linear fits
    ncols = 2
    nrows = (len(names) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3 * nrows))
    axes = np.atleast_1d(axes).ravel()
    print("=" * 64)
    print(f"files={nfiles} records={len(recs)}")
    print("thrust = slope*rpm + intercept   (per bearing)")
    print("-" * 64)
    for idx, nm in enumerate(names):
        pts = thr[nm]
        ax = axes[idx]
        if len(pts) >= 5:
            x = np.array([p[0] for p in pts]); y = np.array([p[1] for p in pts])
            ax.scatter(x, y, s=4, alpha=0.4)
            slope = intercept = r2 = float("nan")
            if np.ptp(x) > 1e-6:
                slope, intercept = np.polyfit(x, y, 1)
                pred = slope * x + intercept
                ss_res = np.sum((y - pred) ** 2)
                ss_tot = np.sum((y - np.mean(y)) ** 2)
                r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
                xs = np.linspace(x.min(), x.max(), 50)
                ax.plot(xs, slope * xs + intercept, color="red", linewidth=1)
            ax.set_title(f"{nm}\nslope={slope:.1f} T/rpm  R2={r2:.3f}", fontsize=8)
            print(f"{nm:32s} slope={slope:8.2f}  intercept={intercept:9.2f}  R2={r2:6.3f}  n={len(pts)}")
        else:
            ax.set_title(f"{nm} (insufficient data)", fontsize=8)
        ax.set_xlabel("rpm", fontsize=7); ax.set_ylabel("thrust", fontsize=7); ax.grid(True, alpha=0.3)
    for j in range(len(names), len(axes)):
        axes[j].axis("off")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "thrust_vs_rpm.png"), dpi=110); plt.close(fig)
    print("=" * 64)
    print("saved plots to", outdir)


if __name__ == "__main__":
    indir = sys.argv[1] if len(sys.argv) > 1 else "."
    outdir = sys.argv[2] if len(sys.argv) > 2 else "analysis"
    main(indir, outdir)
