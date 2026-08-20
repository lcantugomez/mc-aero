# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""
Recover per-channel effectiveness SIGNS (and rough magnitudes) from a pulse run.

Multiple linear regression of the body response onto all 5 RSC commands at once:
    accel_axis ~ b0 + sum_ch (coef_ch) * cmd_ch
The coef signs are the true force-per-command signs (handedness+gearing); magnitudes
should match the analytical G/m (linear accel) and G_M/I_psi (yaw). Handles channel
overlap, so a perfectly isolated pulse is not required.
"""
import sys, os, glob, json
import numpy as np

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pulse"
recs = []
for fn in glob.glob(os.path.join(d, "*.ndjson")):
    with open(fn) as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    recs.append(json.loads(line))
                except Exception:
                    pass

def g(r, *p):
    cur = r
    for k in p:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur

# --- structure probe on first record ---
r0 = recs[0] if recs else {}
print("records:", len(recs))
print("top keys:", sorted(r0.keys()))
act = g(r0, "actuators")
print("actuators keys:", sorted(act.keys()) if isinstance(act, dict) else act)
rsc = g(r0, "actuators", "rsc")
if isinstance(rsc, dict):
    print("rsc channels:", sorted(rsc.keys()))
    anych = next(iter(rsc))
    print("rsc[%s]:" % anych, rsc[anych])
print("velocity sample:", g(r0, "sensors", "velocity"))
print("gimbal rates:", g(r0, "sensors", "gimbal", "getAngularRatesRad"))
print()

CHANNELS = ["mainLift", "forwardBack", "leftRight", "yaw", "upDown"]

def cmd_of(r, ch):
    e = g(r, "actuators", "rsc", ch)
    if isinstance(e, dict):
        for key in ("getTargetSpeed", "target", "getSpeed", "speed"):
            if isinstance(e.get(key), (int, float)):
                return e[key]
    return None

def vel_map(r):
    out = {}
    for s in g(r, "sensors", "velocity") or []:
        a = str(s.get("axis")).lower()
        if isinstance(s.get("velocity"), (int, float)):
            out[a] = s["velocity"]
    return out

rows = []
for r in recs:
    ts = r.get("timestampMs")
    cmds = [cmd_of(r, c) for c in CHANNELS]
    v = vel_map(r)
    rates = g(r, "sensors", "gimbal", "getAngularRatesRad")
    if ts is None or any(c is None for c in cmds):
        continue
    if not all(a in v for a in ("x", "y", "z")):
        continue
    ry = rates[1] if isinstance(rates, list) and len(rates) > 1 else None
    rz = rates[2] if isinstance(rates, list) and len(rates) > 2 else None
    rows.append((ts, cmds, v["x"], v["y"], v["z"], ry, rz))
rows.sort(key=lambda x: x[0])
print("usable rows:", len(rows))
if len(rows) < 20:
    sys.exit(0)

ts = np.array([r[0] for r in rows], float) / 1000.0
C = np.array([r[1] for r in rows], float)          # (n,5) commands
vx = np.array([r[2] for r in rows], float)
vy = np.array([r[3] for r in rows], float)
vz = np.array([r[4] for r in rows], float)
ry = np.array([r[5] if r[5] is not None else np.nan for r in rows], float)
rz = np.array([r[6] if r[6] is not None else np.nan for r in rows], float)

print("\ncommand ranges (min/max) per channel:")
for j, c in enumerate(CHANNELS):
    print("  %-11s %8.1f .. %8.1f" % (c, C[:, j].min(), C[:, j].max()))

# central-difference accelerations
def deriv(x):
    dv = np.gradient(x, ts)
    return dv
ax, ay, az = deriv(vx), deriv(vy), deriv(vz)
ayaw = deriv(ry)  # yaw rate about +y -> yaw angular accel
azaw = deriv(rz)

def regress(y):
    m = np.isfinite(y)
    X = np.hstack([C[m], np.ones((m.sum(), 1))])  # + intercept
    coef, *_ = np.linalg.lstsq(X, y[m], rcond=None)
    return coef  # [5 channel coefs, intercept]

print("\n=== multiple regression: response ~ commands (coef sign = effectiveness sign) ===")
print("response  | " + "  ".join("%10s" % c for c in CHANNELS) + " |   intercept")
for label, resp in [("a_x (fwd/aft)", ax), ("a_y (up)", ay), ("a_z (left)", az),
                    ("yawacc_ry", ayaw), ("yawacc_rz", azaw)]:
    co = regress(resp)
    print("%-12s| " % label + "  ".join("%10.3g" % v for v in co[:5]) + " | %10.3g" % co[5])

print("\nInterpretation: for each channel (column), the largest-magnitude row is its")
print("dominant axis; that coef's SIGN is the force-per-+command sign. yaw column")
print("should show both an a_z term (lateral coupling) and a yaw-accel term.")
