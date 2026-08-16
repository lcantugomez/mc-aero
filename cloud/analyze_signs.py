"""Determine control-axis signs from an open-loop manual sign-test flight.

For each directional RSC axis, find the segments where only that axis was
commanded, then measure the resulting world velocity, rotate it into the body
frame using heading, and report the sign the controller should use, plus a
sanity check that the world->body transform (heading convention) is correct.

Usage: python3 analyze_signs.py <dir-of-ndjson>

Heading convention assumed by the controller: 0deg=+Z(south), 90deg=+X(east), CW
  body-forward (world) = (sin psi, cos psi)
  body-right   (world) = (cos psi, -sin psi)
"""

import glob
import json
import math
import os
import sys
from collections import defaultdict

DIR_AXES = ["forwardBack", "leftRight", "yaw", "upDown"]
CMD_THRESHOLD = 10.0  # |target| considered "commanded"


def num(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return float(v)
    return None


def get(d, *keys):
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur


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
    return recs


def velocity(rec):
    v = {"x": 0.0, "y": 0.0, "z": 0.0}
    for s in get(rec, "sensors", "velocity") or []:
        ax = str(s.get("axis")).lower()
        val = num(s.get("velocity"))
        if ax in v and val is not None:
            v[ax] = val
    return v


def main(directory):
    recs = load(directory)
    if not recs:
        print("no records in", directory)
        return

    # per axis, per command-sign: accumulate samples
    samples = defaultdict(lambda: defaultdict(list))  # axis -> sign -> [rec-derived]
    for r in recs:
        targets = {a: num(get(r, "actuators", "rsc", a, "getTargetSpeed")) or 0.0 for a in DIR_AXES}
        # dominant directional axis (others must be ~0)
        active = [a for a in DIR_AXES if abs(targets[a]) > CMD_THRESHOLD]
        if len(active) != 1:
            continue
        axis = active[0]
        sign = 1 if targets[axis] > 0 else -1
        samples[axis][sign].append(r)

    print("=" * 64)
    print("SIGN TEST — measured motion per commanded axis")
    print("=" * 64)

    def mean(xs):
        xs = [x for x in xs if x is not None]
        return sum(xs) / len(xs) if xs else float("nan")

    for axis in DIR_AXES:
        for sign in (1, -1):
            group = samples[axis].get(sign) or []
            if len(group) < 5:
                continue
            headings = [num(get(r, "sensors", "navigation", "getHeading")) for r in group]
            psi = math.radians(mean(headings))
            vx = mean([velocity(r)["x"] for r in group])
            vz = mean([velocity(r)["z"] for r in group])
            vy = mean([velocity(r)["y"] for r in group])
            # yaw rate (about +Y) from gimbal
            yrates = []
            for r in group:
                rr = get(r, "sensors", "gimbal", "getAngularRatesRad")
                if isinstance(rr, list) and len(rr) >= 3:
                    yrates.append(num(rr[1]))  # index 2 = Y
            yaw_rate = mean(yrates)

            fwd = vx * math.sin(psi) + vz * math.cos(psi)   # body-forward vel
            right = vx * math.cos(psi) - vz * math.sin(psi)  # body-right vel

            label = f"{axis} (cmd {'+' if sign > 0 else '-'})"
            print(f"\n{label}: n={len(group)} heading={math.degrees(psi):.1f}")
            print(f"  world vel: vx={vx:+.3f} vz={vz:+.3f} vy={vy:+.3f}  yawRate={yaw_rate:+.4f}")
            print(f"  body vel:  forward={fwd:+.3f}  right={right:+.3f}")

            if axis == "forwardBack" and sign > 0:
                dominant = "forward" if abs(fwd) >= abs(right) else "LATERAL(!)"
                rec_sign = 1 if fwd > 0 else -1
                print(f"  -> +forwardBack moves body-{('forward' if fwd>0 else 'backward')}; "
                      f"forwardBackSign = {rec_sign}  (dominant axis: {dominant})")
                if abs(right) > abs(fwd):
                    print("  !! motion is mostly LATERAL -> heading/transform convention is wrong")
            elif axis == "leftRight" and sign > 0:
                dominant = "right" if abs(right) >= abs(fwd) else "FORWARD(!)"
                rec_sign = 1 if right > 0 else -1
                print(f"  -> +leftRight moves body-{('right' if right>0 else 'left')}; "
                      f"leftRightSign = {rec_sign}  (dominant axis: {dominant})")
                if abs(fwd) > abs(right):
                    print("  !! motion is mostly FORWARD -> heading/transform convention is wrong")
            elif axis == "yaw" and sign > 0:
                rec_sign = 1 if yaw_rate > 0 else -1
                print(f"  -> +yaw gives yawRate {yaw_rate:+.4f}; expected feedback consistent when "
                      f"yawRateSign matches. yawSign={rec_sign if yaw_rate!=0 else '?'}")
            elif axis == "upDown" and sign > 0:
                print(f"  -> +upDown moves vy {vy:+.3f} ({'up' if vy>0 else 'down'})")
    print("\n" + "=" * 64)
    print("Use the recommended *Sign values in control_config; if a 'transform wrong'")
    print("warning appears, the heading rotation handedness must be fixed instead.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
