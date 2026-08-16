#!/usr/bin/env python3
# Clean-portion analysis of a pure-yaw run: drop the distance-glitch tail, then
# reconstruct nav position and CoM position and measure how much each still moves
# during the yaw. Also fit the lever arm nav = com + R(psi)*r.
import sys, os, glob, json, math

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/yaw77"
DIST_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
LODE_X, LODE_Z = -30.0, 1046.0
OFF_FWD, OFF_RIGHT = 9.0, 1.0

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

def g(r, *path):
    cur = r
    for k in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur

rows = []
for r in recs:
    nav = g(r, "sensors", "navigation") or {}
    h, b, dist, vo = nav.get("getHeading"), nav.get("getBearing"), nav.get("getDistanceToTarget"), nav.get("getVerticalOffsetToTarget")
    if all(isinstance(v, (int, float)) for v in (h, b, dist)) and dist <= DIST_MAX:
        rows.append((r.get("timestampMs"), h, b, dist, vo if isinstance(vo, (int, float)) else 0.0))
rows.sort(key=lambda x: (x[0] or 0))

def spread(vals):
    return (min(vals), max(vals), max(vals) - min(vals))

print("clean rows (dist<=%.0f): %d of %d records" % (DIST_MAX, len(rows), len(recs)))
if not rows:
    sys.exit(0)

dist = [r[3] for r in rows]
print("distance min/max      : %.2f / %.2f" % (min(dist), max(dist)))

navx, navz, comx, comz = [], [], [], []
for ts, h, b, ds, vo in rows:
    sq = ds*ds - vo*vo
    horiz = math.sqrt(sq) if sq > 0 else 0.0
    th = math.radians(h - b)
    x = LODE_X - horiz*math.sin(th)
    z = LODE_Z - horiz*math.cos(th)
    psi = math.radians(h)
    rx = OFF_FWD*math.sin(psi) + OFF_RIGHT*math.cos(psi)
    rz = OFF_FWD*math.cos(psi) - OFF_RIGHT*math.sin(psi)
    navx.append(x); navz.append(z)
    comx.append(x - rx); comz.append(z - rz)

print()
print("NAV X span : %.2f   (%.2f..%.2f)" % (spread(navx)[2], spread(navx)[0], spread(navx)[1]))
print("NAV Z span : %.2f   (%.2f..%.2f)" % (spread(navz)[2], spread(navz)[0], spread(navz)[1]))
print("CoM X span : %.2f   (%.2f..%.2f)  [offset %g/%g]" % (spread(comx)[2], spread(comx)[0], spread(comx)[1], OFF_FWD, OFF_RIGHT))
print("CoM Z span : %.2f   (%.2f..%.2f)" % (spread(comz)[2], spread(comz)[0], spread(comz)[1]))
print()
print("If CoM span << NAV span, the offset is working. If similar/larger, the")
print("residual is real drift or a heading-dependent bearing error, not lever arm.")

# ---- fit nav = center + R(psi)*r  (r = fwd,right) via LS on [1,sin,cos] ----
h = [r[1] for r in rows]
S = [math.sin(math.radians(v)) for v in h]
C = [math.cos(math.radians(v)) for v in h]
n = len(rows)
ones = [1.0]*n

def solve3(cols, y):
    A = [[0.0]*3 for _ in range(3)]; bb = [0.0]*3
    for i in range(n):
        xi = [cols[0][i], cols[1][i], cols[2][i]]
        for a in range(3):
            bb[a] += xi[a]*y[i]
            for c in range(3):
                A[a][c] += xi[a]*xi[c]
    M = [A[k][:] + [bb[k]] for k in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda rr: abs(M[rr][col]))
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        if abs(pv) < 1e-12:
            return None
        M[col] = [v/pv for v in M[col]]
        for rr in range(3):
            if rr != col:
                f = M[rr][col]; M[rr] = [M[rr][k]-f*M[col][k] for k in range(4)]
    return [M[0][3], M[1][3], M[2][3]]

ax = solve3([ones, S, C], navx)
bz = solve3([ones, S, C], navz)
print()
print("=== lever-arm fit on clean data ===")
if ax and bz:
    fwd = (ax[1] + bz[2]) / 2.0
    right = (ax[2] - bz[1]) / 2.0
    print("fitted center (true CoM) : X=%.2f  Z=%.2f" % (ax[0], bz[0]))
    print("fitted lever arm         : fwd=%.2f  right=%.2f  (radius=%.2f)" % (fwd, right, math.hypot(fwd, right)))
    print("current config           : fwd=9  right=1  (radius=9.06)")
    res = []
    for i in range(n):
        px = ax[0] + fwd*S[i] + right*C[i]
        pz = bz[0] + fwd*C[i] - right*S[i]
        res.append(math.hypot(navx[i]-px, navz[i]-pz))
    res.sort()
    print("fit residual (nav-circle): mean=%.2f  median=%.2f  max=%.2f blocks" % (sum(res)/len(res), res[len(res)//2], res[-1]))
