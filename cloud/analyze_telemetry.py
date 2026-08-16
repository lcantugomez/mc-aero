"""Summarize MC Aero telemetry NDJSON captured in S3.

Usage: python3 analyze_telemetry.py <dir-of-ndjson-files>

Reads every *.ndjson file in the directory, sorts records by timestamp, and
prints a summary: time span / rate, RSC command vs actual, per-bearing rpm and
thrust ranges, gearshift activity, velocities, and errors.
"""

import glob
import json
import math
import os
import sys
from collections import defaultdict


def as_number(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def rng(values):
    nums = [v for v in values if v is not None]
    if not nums:
        return None
    return (min(nums), max(nums))


def fmt_rng(r):
    if r is None:
        return "n/a"
    lo, hi = r
    return f"{lo:.3f} .. {hi:.3f}"


def main(directory):
    files = sorted(glob.glob(os.path.join(directory, "**", "*.ndjson"), recursive=True))
    if not files:
        files = sorted(glob.glob(os.path.join(directory, "*.ndjson")))
    records = []
    for path in files:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass

    if not records:
        print("no records found in", directory)
        return

    records.sort(key=lambda r: r.get("timestampMs") or 0)
    n = len(records)
    ts = [r.get("timestampMs") for r in records if isinstance(r.get("timestampMs"), (int, float))]
    seqs = [r.get("sequence") for r in records if isinstance(r.get("sequence"), (int, float))]

    print("=" * 60)
    print(f"files={len(files)}  records={n}")
    if ts:
        span = (max(ts) - min(ts)) / 1000.0
        rate = (n - 1) / span if span > 0 else 0
        print(f"time span = {span:.1f}s   avg rate = {rate:.1f} Hz")
    if seqs:
        gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b - a not in (0, 1))
        print(f"sequence range = {min(seqs)} .. {max(seqs)}   non-contiguous steps = {gaps}")
    print("modes:", sorted({str(r.get("mode")) for r in records}))

    # --- RSCs (per axis) ---------------------------------------------------
    axes = ["mainLift", "forwardBack", "yaw", "leftRight", "upDown"]

    def rsc_field(axis, field):
        return rng([
            as_number((r.get("actuators") or {}).get("rsc", {}).get(axis, {}).get(field))
            for r in records
        ])

    print("\n--- RSCs (target / actual ranges) ---")
    for axis in axes:
        print(f"  {axis:12s} target[{fmt_rng(rsc_field(axis, 'getTargetSpeed'))}]"
              f"  actual[{fmt_rng(rsc_field(axis, 'getSpeed'))}]")

    # --- bearings ----------------------------------------------------------
    rpm = defaultdict(list)
    thrust = defaultdict(list)
    active = defaultdict(set)
    for r in records:
        for b in (r.get("actuators") or {}).get("bearings", []) or []:
            name = b.get("name", "?")
            rpm[name].append(as_number(b.get("getRotationSpeed")))
            thrust[name].append(as_number(b.get("getThrust")))
            active[name].add(b.get("isActive"))
    print("\n--- BEARINGS (rpm / thrust ranges) ---")
    for name in sorted(rpm):
        print(f"  {name:32s} rpm[{fmt_rng(rng(rpm[name]))}]  thrust[{fmt_rng(rng(thrust[name]))}]  active={sorted(str(x) for x in active[name])}")

    # --- sweep fits (if this capture is an actuator sweep) -----------------
    sweep_recs = [r for r in records if isinstance(r.get("sweep"), dict)]
    if sweep_recs:
        print("\n--- SWEEP: settled command -> bearing response (G = rpm/target, k = thrust/rpm) ---")
        groups = defaultdict(list)
        for r in sweep_recs:
            s = r["sweep"]
            groups[(str(s.get("axis")), as_number(s.get("target")))].append(r)
        for axis, target in sorted(groups, key=lambda key: (key[0], key[1] or 0)):
            if not target or target == 0:
                continue
            group = groups[(axis, target)]
            elapsed = [as_number(r["sweep"].get("elapsedMs")) or 0 for r in group]
            cutoff = 0.5 * max(elapsed) if elapsed else 0
            settled = [r for r in group if (as_number(r["sweep"].get("elapsedMs")) or 0) >= cutoff]
            per_bearing = defaultdict(lambda: [[], []])
            for r in settled:
                for b in (r.get("actuators") or {}).get("bearings", []) or []:
                    label = b.get("role") or b.get("name")
                    rr, tt = as_number(b.get("getRotationSpeed")), as_number(b.get("getThrust"))
                    if rr is not None:
                        per_bearing[label][0].append(rr)
                    if tt is not None:
                        per_bearing[label][1].append(tt)
            driven = {
                nm: v for nm, v in per_bearing.items()
                if v[0] and abs(sum(v[0]) / len(v[0])) > 1.0
            }
            if driven:
                print(f"  {axis} @ {target:.0f}:")
                for nm, (rpms, thrusts) in sorted(driven.items()):
                    mr = sum(rpms) / len(rpms)
                    mt = (sum(thrusts) / len(thrusts)) if thrusts else float("nan")
                    g = mr / target
                    k = mt / mr if abs(mr) > 1e-6 else float("nan")
                    print(f"    {nm:22s} rpm={mr:8.2f} (G={g:5.2f})  thrust={mt:10.2f}  k={k:8.2f}")

        # spin-up time constant tau per axis: pool every step's normalized
        # first-order decay ln((rf-rpm)/(rf-r0)) = -t/tau and least-squares fit.
        print("\n--- SWEEP: spin-up time constant tau (first-order) ---")
        axis_points = defaultdict(list)
        for (axis, target), group in groups.items():
            if not target or target == 0:
                continue
            group = sorted(group, key=lambda r: as_number(r["sweep"].get("elapsedMs")) or 0)
            series = defaultdict(list)
            for r in group:
                e = (as_number(r["sweep"].get("elapsedMs")) or 0) / 1000.0
                for b in (r.get("actuators") or {}).get("bearings", []) or []:
                    rr = as_number(b.get("getRotationSpeed"))
                    if rr is not None:
                        series[b.get("role") or b.get("name")].append((e, rr))
            best_nm, best_val = None, 0.0
            for nm, pts in series.items():
                if pts and abs(pts[-1][1]) > best_val:
                    best_nm, best_val = nm, abs(pts[-1][1])
            pts = series.get(best_nm) or []
            if len(pts) < 6:
                continue
            r0 = pts[0][1]
            tail = pts[len(pts) // 2:]
            rf = sum(v for _, v in tail) / len(tail)
            denom = rf - r0
            if abs(denom) < 1.0:
                continue
            for e, rr in pts:
                frac = (rf - rr) / denom
                if e > 0 and 0.05 < frac < 0.98:
                    axis_points[axis].append((e, math.log(frac)))
        for axis in ["mainLift", "forwardBack", "yaw", "leftRight", "upDown"]:
            pts = axis_points.get(axis) or []
            if len(pts) < 5:
                print(f"  {axis:12s} tau: insufficient transient data")
                continue
            nsum = len(pts)
            sx = sum(p[0] for p in pts)
            sy = sum(p[1] for p in pts)
            sxx = sum(p[0] * p[0] for p in pts)
            sxy = sum(p[0] * p[1] for p in pts)
            d = nsum * sxx - sx * sx
            slope = (nsum * sxy - sx * sy) / d if abs(d) > 1e-9 else 0
            if slope < 0:
                tau = -1.0 / slope
                print(f"  {axis:12s} tau ~ {tau:.2f} s   (settle ~{3 * tau:.1f} s, n={nsum})")
            else:
                print(f"  {axis:12s} tau: could not fit")

    # --- motion ------------------------------------------------------------
    print("\n--- MOTION ---")
    print("  height        :", fmt_rng(rng([as_number((r.get("sensors") or {}).get("altitude", {}).get("height")) for r in records])))
    print("  verticalSpeed :", fmt_rng(rng([as_number((r.get("sensors") or {}).get("altitude", {}).get("verticalSpeed")) for r in records])))
    axis_vals = defaultdict(list)
    for r in records:
        for v in (r.get("sensors") or {}).get("velocity", []) or []:
            axis_vals[str(v.get("axis"))].append(as_number(v.get("velocity")))
    for axis in sorted(axis_vals):
        print(f"  velocity {axis:8s}:", fmt_rng(rng(axis_vals[axis])))

    # --- errors ------------------------------------------------------------
    err_counts = [r.get("errorCount") for r in records if isinstance(r.get("errorCount"), (int, float))]
    err_keys = set()
    for r in records:
        for k in (r.get("errors") or {}):
            err_keys.add(k)
    print("\n--- ERRORS ---")
    print("  errorCount range:", fmt_rng(rng(err_counts)))
    print("  distinct error keys:", len(err_keys))
    for k in sorted(err_keys):
        print("    -", k)
    print("=" * 60)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
