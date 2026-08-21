# /// script
# requires-python = ">=3.10"
# ///
"""Look at heading behavior through a mission: actual heading vs commanded target
heading vs the 'nose-at-goal' bearing, to see the cruise-entry 180 flip."""
import sys, os, glob, json, math

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cr"
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

def wrap(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a

rows = []
for r in recs:
    ap = g(r, "autopilot") or {}
    rows.append((
        r.get("timestampMs"),
        g(ap, "guidance", "state"),
        g(r, "sensors", "navigation", "getHeading"),
        g(ap, "targets", "heading"),
        g(r, "sensors", "position", "comX"),
        g(r, "sensors", "position", "comZ"),
        g(ap, "guidance", "distance"),
    ))
rows = [x for x in rows if x[0] is not None and x[1] is not None]
rows.sort(key=lambda x: x[0])
print("autopilot rows:", len(rows))
if not rows:
    sys.exit(0)

st = {}
for x in rows:
    st[x[1]] = (st.get(x[1]) or 0) + 1
print("state counts:", st)

def f(v): return ("%.0f" % v) if isinstance(v, (int, float)) else "-"
print("\n  ts     state    heading  tgtHdg  hdgErr  comX    comZ    dist")
prev = None
step = max(1, len(rows)//30)
for i in range(0, len(rows), step):
    x = rows[i]
    herr = None
    if isinstance(x[2], (int, float)) and isinstance(x[3], (int, float)):
        herr = wrap(x[2] - x[3])
    print("  %s %-8s %7s %7s %6s %7s %8s %6s" % (
        str(x[0])[-6:], str(x[1]), f(x[2]), f(x[3]), f(herr), f(x[4]), f(x[5]), f(x[6])))

# show every state transition with heading + target at the moment
print("\n=== state transitions (heading / target at change) ===")
last = None
for x in rows:
    if x[1] != last:
        print("  -> %-8s  heading=%s  tgtHdg=%s  dist=%s" % (x[1], f(x[2]), f(x[3]), f(x[6])))
        last = x[1]
