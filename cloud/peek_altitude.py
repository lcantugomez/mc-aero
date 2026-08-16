"""Print the altitude-hold loop internals over time from autopilot telemetry.

Usage: python3 peek_altitude.py <dir-of-ndjson> [downsample_seconds]
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

directory = sys.argv[1] if len(sys.argv) > 1 else "."
step_s = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

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

alt = [r for r in recs if isinstance(get(r, "autopilot", "altitude"), dict)]
alt.sort(key=lambda r: r.get("timestampMs") or 0)
if not alt:
    print("no autopilot.altitude records found")
    sys.exit(0)

t0 = alt[0]["timestampMs"]
print(f"autopilot.altitude records: {len(alt)}  span={ (alt[-1]['timestampMs']-t0)/1000:.1f}s")
print(f"{'t(s)':>6} {'h':>8} {'target':>7} {'err':>7} {'vy':>7} {'vyCmd':>6} {'evy':>7} {'Kvy':>6} {'cmd':>7} {'integ':>8} sat")
last = -1e9
for r in alt:
    t = (r["timestampMs"] - t0) / 1000.0
    if t - last < step_s:
        continue
    last = t
    a = r["autopilot"]["altitude"]
    tg = get(r, "autopilot", "targets", "altitude")
    h = get(r, "sensors", "altitude", "height")
    print(f"{t:6.1f} {(h or 0):8.2f} {(tg or 0):7.2f} {(num(a.get('error')) or 0):7.2f} "
          f"{(num(a.get('velocityError')) is not None and 0 or 0):0.0f}".rjust(0)
          + f"{(num(get(r,'sensors','altitude','verticalSpeed')) or 0):7.3f} "
          f"{(num(a.get('vyCommand')) or 0):6.2f} {(num(a.get('velocityError')) or 0):7.3f} "
          f"{(num(a.get('Kvy')) or 0):6.2f} {(num(a.get('command')) or 0):7.2f} "
          f"{(num(a.get('integral')) or 0):8.1f} {str(a.get('saturated'))[:1]}")
