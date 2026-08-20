# /// script
# requires-python = ">=3.10"
# ///
"""Inspect the stuck-at-170 goto: guidance state, altitude target vs height,
mainLift command (saturated?), s_y error, over time."""
import sys, os, glob, json

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/t"
recs = []
for fn in glob.glob(os.path.join(d, "*.ndjson")):
    with open(fn) as fh:
        for line in fh:
            line = line.strip()
            if line:
                try: recs.append(json.loads(line))
                except Exception: pass

def g(r, *p):
    cur = r
    for k in p:
        if not isinstance(cur, dict): return None
        cur = cur.get(k)
    return cur

rows = []
for r in recs:
    ap = g(r, "autopilot") or {}
    rows.append((
        r.get("timestampMs"), r.get("mode"),
        g(ap, "guidance", "state"),
        g(ap, "targets", "altitude"),
        g(r, "sensors", "altitude", "getHeight"),
        g(r, "actuators", "rsc", "mainLift", "getTargetSpeed"),
        g(r, "actuators", "rsc", "mainLift", "getSpeed"),
        g(ap, "errors", "s_y"),
        g(ap, "targets", "heading"),
        g(r, "sensors", "navigation", "getHeading"),
        g(ap, "guidance", "distance"),
    ))
rows = [x for x in rows if x[0] is not None]
rows.sort(key=lambda x: x[0])
print("records:", len(recs), " rows:", len(rows))
modes = {}
states = {}
for x in rows:
    modes[x[1]] = (modes.get(x[1]) or 0) + 1
    states[x[2]] = (states.get(x[2]) or 0) + 1
print("mode counts:", modes)
print("guidance state counts:", states)

def f(v): return ("%.1f" % v) if isinstance(v, (int, float)) else "-"
print("\n ts  mode  gstate  tgtAlt  height  mainCmd  mainSpd  s_y  tgtHdg  hdg  dist")
step = max(1, len(rows) // 25)
for i in range(0, len(rows), step):
    x = rows[i]
    print("%s %-9s %-7s %6s %6s %7s %7s %5s %6s %5s %6s" % (
        str(x[0])[-6:], str(x[1]), str(x[2]), f(x[3]), f(x[4]), f(x[5]), f(x[6]),
        f(x[7]), f(x[8]), f(x[9]), f(x[10])))

# max mainLift command seen while in climb/autopilot
climb_cmds = [x[5] for x in rows if x[2] == "climb" and isinstance(x[5], (int, float))]
if climb_cmds:
    print("\nmainLift cmd during CLIMB: min %.1f  max %.1f" % (min(climb_cmds), max(climb_cmds)))
heights = [x[4] for x in rows if isinstance(x[4], (int, float))]
if heights:
    print("height range: %.1f .. %.1f" % (min(heights), max(heights)))
