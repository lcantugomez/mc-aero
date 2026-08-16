"""Print the horizontal-hold loop internals over time from autopilot telemetry.

Usage: python3 peek_horizontal.py <dir-of-ndjson> [downsample_seconds]
"""
import glob, json, os, sys

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
    out = {"x": 0.0, "z": 0.0}
    for s in get(r, "sensors", "velocity") or []:
        ax = str(s.get("axis")).lower()
        v = num(s.get("velocity"))
        if ax in out and v is not None:
            out[ax] = v
    return out

directory = sys.argv[1] if len(sys.argv) > 1 else "."
step_s = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

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

hz = [r for r in recs if isinstance(get(r, "autopilot", "horizontal"), dict)
      and get(r, "autopilot", "horizontal")]
hz.sort(key=lambda r: r.get("timestampMs") or 0)
if not hz:
    print("no autopilot.horizontal records found (was horizontal enabled?)")
    sys.exit(0)

t0 = hz[0]["timestampMs"]
print(f"autopilot.horizontal records: {len(hz)}  span={(hz[-1]['timestampMs']-t0)/1000:.1f}s")
print(f"{'t':>6} {'x':>8} {'z':>8} {'tX':>8} {'tZ':>8} {'eX':>6} {'eZ':>6} "
      f"{'vx':>6} {'vz':>6} {'hdg':>6} {'eVf':>6} {'eVl':>6} {'uFB':>6} {'uLR':>6}")
last = -1e9
for r in hz:
    t = (r["timestampMs"] - t0) / 1000.0
    if t - last < step_s:
        continue
    last = t
    h = r["autopilot"]["horizontal"]
    pos = get(r, "sensors", "position") or {}
    tg = get(r, "autopilot", "targets") or {}
    v = vel(r)
    hdg = num(get(r, "sensors", "navigation", "getHeading")) or 0
    def f(x): return (num(x) if num(x) is not None else 0.0)
    print(f"{t:6.1f} {f(pos.get('x')):8.2f} {f(pos.get('z')):8.2f} "
          f"{f(tg.get('x')):8.2f} {f(tg.get('z')):8.2f} "
          f"{f(h.get('xError')):6.2f} {f(h.get('zError')):6.2f} "
          f"{v['x']:6.2f} {v['z']:6.2f} {hdg:6.1f} "
          f"{f(h.get('bodyForwardVelocityError')):6.2f} {f(h.get('bodyLateralVelocityError')):6.2f} "
          f"{f(h.get('forwardBackCommand')):6.1f} {f(h.get('leftRightCommand')):6.1f}")
