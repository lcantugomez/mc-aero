"""Compare pure-yaw-RSC vs combined yaw+translation, and quantify the parasitic
translation that yaw produces (the reason heading hold fought position).

Usage: python3 analyze_yaw.py <dir-of-ndjson>
"""
import glob, json, math, os, sys
from collections import defaultdict

def num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None

def get(d, *ks):
    cur = d
    for k in ks:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur

def vel(r):
    out = {"x": 0.0, "z": 0.0, "y": 0.0}
    for s in get(r, "sensors", "velocity") or []:
        ax = str(s.get("axis")).lower()
        v = num(s.get("velocity"))
        if ax in out and v is not None:
            out[ax] = v
    return out

def yawrate(r):
    rr = get(r, "sensors", "gimbal", "getAngularRatesRad")
    if isinstance(rr, list) and len(rr) >= 3:
        return num(rr[1])  # +Y component
    return None

directory = sys.argv[1] if len(sys.argv) > 1 else "."
files = sorted(glob.glob(os.path.join(directory, "**", "*.ndjson"), recursive=True)) \
    or sorted(glob.glob(os.path.join(directory, "*.ndjson")))
recs = []
for p in files:
    with open(p, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    recs.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
recs.sort(key=lambda r: r.get("timestampMs") or 0)

def tgt(r, axis):
    return num(get(r, "actuators", "rsc", axis, "getTargetSpeed")) or 0.0

# Manual full-stick inputs are ~256; autopilot never exceeds its clamps
# (yaw<=75, lateral<=100). Use a high threshold to isolate the manual test.
DRIVEN = 150
buckets = defaultdict(list)
for r in recs:
    yw = tgt(r, "yaw")
    lr = tgt(r, "leftRight")
    if abs(yw) < DRIVEN:
        continue
    if abs(lr) < 20:
        buckets["yaw_only"].append(r)
    elif abs(lr) >= DRIVEN and (yw > 0) == (lr > 0):
        buckets["yaw_plus_lat_sameside"].append(r)
    elif abs(lr) >= DRIVEN:
        buckets["yaw_plus_lat_opposite"].append(r)

def summarize(name, rs):
    if len(rs) < 5:
        print(f"{name}: only {len(rs)} samples")
        return
    heads = [num(get(r, "sensors", "navigation", "getHeading")) or 0 for r in rs]
    psis = [math.radians(h) for h in heads]
    speeds, fwds, lats, yrs = [], [], [], []
    for r, psi in zip(rs, psis):
        v = vel(r)
        speeds.append(math.hypot(v["x"], v["z"]))
        fwds.append(v["x"] * math.sin(psi) + v["z"] * math.cos(psi))
        lats.append(v["x"] * math.cos(psi) - v["z"] * math.sin(psi))
        yr = yawrate(r)
        if yr is not None:
            yrs.append(yr)
    def m(a): return sum(a) / len(a) if a else float("nan")
    print(f"\n{name}: n={len(rs)}")
    print(f"  mean yaw rate      = {m(yrs):+.4f} rad/s")
    print(f"  mean world speed   = {m(speeds):.3f} blocks/s   (parasitic translation)")
    print(f"  mean body forward  = {m(fwds):+.3f}")
    print(f"  mean body lateral  = {m(lats):+.3f}")
    if abs(m(yrs)) > 1e-4:
        print(f"  translation/yaw    = {m(speeds)/abs(m(yrs)):.2f} (blocks/s per rad/s)")

print("=" * 60)
print("PURE-YAW-RSC vs YAW+TRANSLATION")
print("=" * 60)
summarize("yaw only (yaw stick alone)", buckets["yaw_only"])
summarize("yaw + translation SAME side (pure-rotation recipe)", buckets["yaw_plus_lat_sameside"])
summarize("yaw + translation OPPOSITE side", buckets["yaw_plus_lat_opposite"])
print("\nIf same-side world speed << yaw-only world speed at similar yaw rate,")
print("the force cancels and it's near-pure rotation, the allocator target.")
