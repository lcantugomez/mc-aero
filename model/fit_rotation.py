# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""Empirically fit the world->body rotation used by the velocity sensors.
bodyVel = Rot(theta) * worldVel, with theta = alpha + s*heading. Solve for s
(+/-1) and alpha, then emit the correct s_x/s_z position-error transform so the
position frame matches the velocity frame (kills the position-hold spiral)."""
import sys, os, glob, json, math
import numpy as np

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rot"
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

def velmap(r):
    out = {}
    for s in g(r, "sensors", "velocity") or []:
        a = str(s.get("axis")).lower()
        if isinstance(s.get("velocity"), (int, float)): out[a] = s["velocity"]
    return out

rows = []
for r in recs:
    pos = g(r, "sensors", "position") or {}
    v = velmap(r)
    h = g(r, "sensors", "navigation", "getHeading")
    ts = r.get("timestampMs")
    if ts is None or not isinstance(h, (int, float)): continue
    if pos.get("comX") is None or pos.get("comZ") is None: continue
    if "x" not in v or "z" not in v: continue
    rows.append((ts, pos["comX"], pos["comZ"], v["x"], v["z"], h))
rows.sort(key=lambda x: x[0])
print("rows:", len(rows))
if len(rows) < 30:
    sys.exit(0)

t = np.array([r[0] for r in rows]) / 1000.0
cx = np.array([r[1] for r in rows]); cz = np.array([r[2] for r in rows])
bx = np.array([r[3] for r in rows]); bz = np.array([r[4] for r in rows])   # sensor body vel
h = np.array([r[5] for r in rows])
wX = np.gradient(cx, t); wZ = np.gradient(cz, t)                            # world vel (nav deriv)

speedW = np.hypot(wX, wZ); speedB = np.hypot(bx, bz)
mask = (speedW > 0.15) & (speedB > 0.15)
print("usable moving samples:", int(mask.sum()))
angW = np.arctan2(wZ[mask], wX[mask])
angB = np.arctan2(bz[mask], bx[mask])
hh = np.deg2rad(h[mask])

def circ_mean_std(a):
    c, s = np.mean(np.cos(a)), np.mean(np.sin(a))
    return math.atan2(s, c), math.sqrt(max(0.0, -2*math.log(min(1.0, max(1e-9, math.hypot(c, s))))))

best = None
for s in (+1, -1):
    resid = angB - angW - s * hh          # = alpha (if model holds)
    alpha, sd = circ_mean_std(resid)
    print("s=%+d :  alpha = %+.1f deg   circ_sd = %.1f deg" % (s, math.degrees(alpha), math.degrees(sd)))
    if best is None or sd < best[2]:
        best = (s, alpha, sd)

s, alpha, sd = best
print("\nBEST: bodyVel = Rot(theta)*worldVel,  theta = alpha + s*heading")
print("      s = %+d,  alpha = %+.2f deg,  fit circ_sd = %.1f deg" % (s, math.degrees(alpha), math.degrees(sd)))

# Position error must transform the same way: s_body = Rot(theta)*(dX,dZ)
#   Rot(theta) = [[cos,-sin],[sin,cos]] with theta = alpha + s*heading  (heading in rad)
# => s_x = cos*dX - sin*dZ ; s_z = sin*dX + cos*dZ
# Fold s*heading into per-tick sin/cos; alpha is a constant offset (headingOffsetDeg-like).
print("\nRuntime transform (heading h in deg):")
print("  th   = math.rad(%.2f) + (%d) * math.rad(h)" % (math.degrees(alpha), s))
print("  s_x  =  math.cos(th)*dX - math.sin(th)*dZ")
print("  s_z  =  math.sin(th)*dX + math.cos(th)*dZ")
print("  (dX = comX - targetX, dZ = comZ - targetZ)")

# sanity: report residual angle between predicted body dir and actual
th = alpha + s * hh
pbx = np.cos(th) * wX[mask] - np.sin(th) * wZ[mask]
pbz = np.sin(th) * wX[mask] + np.cos(th) * wZ[mask]
angP = np.arctan2(pbz, pbx)
err = np.arctan2(np.sin(angP - angB), np.cos(angP - angB))
print("\nprediction vs actual body-vel direction: mean|err| = %.1f deg" % np.degrees(np.mean(np.abs(err))))
