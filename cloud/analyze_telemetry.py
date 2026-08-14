"""Summarize MC Aero telemetry NDJSON captured in S3.

Usage: python3 analyze_telemetry.py <dir-of-ndjson-files>

Reads every *.ndjson file in the directory, sorts records by timestamp, and
prints a summary: time span / rate, RSC command vs actual, per-bearing rpm and
thrust ranges, gearshift activity, velocities, and errors.
"""

import glob
import json
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

    # --- RSC ---------------------------------------------------------------
    def rsc_field(field):
        return rng([as_number((r.get("actuators") or {}).get("liftController", {}).get(field)) for r in records])

    print("\n--- MAIN LIFT (RSC) ---")
    print("  commandedSpeed:", fmt_rng(rsc_field("commandedSpeed")))
    print("  getTargetSpeed:", fmt_rng(rsc_field("getTargetSpeed")))
    print("  getSpeed      :", fmt_rng(rsc_field("getSpeed")))
    tgt = rsc_field("getTargetSpeed")
    if tgt and abs(tgt[1] - tgt[0]) < 1e-6:
        print(f"  -> RSC target held constant at {tgt[0]:.1f} (main lift untouched)")

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

    # --- gearshifts --------------------------------------------------------
    gs_speed = defaultdict(list)
    gs_left = defaultdict(set)
    gs_right = defaultdict(set)
    gs_mode = defaultdict(set)
    gs_axis = defaultdict(set)
    for r in records:
        for g in (r.get("actuators") or {}).get("gearshifts", []) or []:
            name = g.get("name", "?")
            gs_speed[name].append(as_number(g.get("getSpeed")))
            gs_left[name].add(g.get("isLeftPowered"))
            gs_right[name].add(g.get("isRightPowered"))
            gs_mode[name].add(str(g.get("getMode")))
            gs_axis[name].add(str(g.get("getSourceAxis")))
    print("\n--- GEARSHIFTS (directional controls used this run) ---")
    for name in sorted(gs_speed):
        used = (True in gs_left[name]) or (True in gs_right[name])
        print(f"  {name:26s} speed[{fmt_rng(rng(gs_speed[name]))}]  L={sorted(str(x) for x in gs_left[name])} R={sorted(str(x) for x in gs_right[name])}  used={used}")

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
