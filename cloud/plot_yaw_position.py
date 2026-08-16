#!/usr/bin/env python3
# Plot reconstructed nav position vs time during a yaw run, to confirm the motion
# is oscillatory (i.e. rotation about some axis). Drops the distance-glitch tail.
#   uv run --with matplotlib --with numpy python cloud/plot_yaw_position.py /tmp/yaw77 analysis
import sys, os, glob, json, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = sys.argv[1] if len(sys.argv) > 1 else "/tmp/yaw77"
outdir = sys.argv[2] if len(sys.argv) > 2 else "analysis"
DIST_MAX = float(sys.argv[3]) if len(sys.argv) > 3 else 150.0
LODE_X, LODE_Z = -30.0, 1046.0
os.makedirs(outdir, exist_ok=True)

recs = []
for fn in glob.glob(os.path.join(src, "*.ndjson")):
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

rows = []
for r in recs:
    nav = g(r, "sensors", "navigation") or {}
    h, b, dist = nav.get("getHeading"), nav.get("getBearing"), nav.get("getDistanceToTarget")
    vo = nav.get("getVerticalOffsetToTarget")
    ts = r.get("timestampMs")
    if all(isinstance(v, (int, float)) for v in (h, b, dist, ts)) and dist <= DIST_MAX:
        vo = vo if isinstance(vo, (int, float)) else 0.0
        rows.append((ts, h, b, dist, vo))
rows.sort(key=lambda x: x[0])
if not rows:
    print("no usable rows")
    sys.exit(0)

t0 = rows[0][0]
t, X, Z, H = [], [], [], []
for ts, h, b, dist, vo in rows:
    sq = dist*dist - vo*vo
    horiz = math.sqrt(sq) if sq > 0 else 0.0
    th = math.radians(h - b)
    t.append((ts - t0) / 1000.0)
    X.append(LODE_X - horiz*math.sin(th))
    Z.append(LODE_Z - horiz*math.cos(th))
    H.append(h)

fig, ax = plt.subplots(3, 1, figsize=(11, 11))
ax[0].plot(t, X, label="nav X", color="tab:blue")
ax[0].plot(t, Z, label="nav Z", color="tab:orange")
ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("world position (blocks)")
ax[0].set_title("Reconstructed nav position vs time (%d pts, dist<=%.0f)" % (len(t), DIST_MAX))
ax[0].legend(); ax[0].grid(True, alpha=0.3)

ax[1].plot(t, H, color="tab:green")
ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("heading (deg)")
ax[1].set_title("Heading vs time"); ax[1].grid(True, alpha=0.3)

ax[2].plot(X, Z, color="tab:red", lw=0.8)
ax[2].scatter([X[0]], [Z[0]], c="green", s=40, label="start", zorder=5)
ax[2].scatter([X[-1]], [Z[-1]], c="black", s=40, label="end", zorder=5)
ax[2].set_xlabel("nav X"); ax[2].set_ylabel("nav Z")
ax[2].set_title("Path in X-Z plane (a circle => rotation about an axis)")
ax[2].axis("equal"); ax[2].legend(); ax[2].grid(True, alpha=0.3)

fig.tight_layout()
out = os.path.join(outdir, "yaw_position.png")
fig.savefig(out, dpi=110)
print("wrote " + out)
print("X range %.2f..%.2f  Z range %.2f..%.2f  span %.2f/%.2f"
      % (min(X), max(X), min(Z), max(Z), max(X)-min(X), max(Z)-min(Z)))
