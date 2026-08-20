#!/usr/bin/env python3
# Plot a horizontal-hold autopilot run: comX(t), comZ(t), and the CoM path vs the
# target. An orbit (circle around target) => force applied ~perpendicular to error
# (rotated body axes). A line through the target => normal 1D underdamping.
#   uv run --with matplotlib python cloud/plot_hold.py /tmp/ap90 analysis
import sys, os, glob, json, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ap90"
outdir = sys.argv[2] if len(sys.argv) > 2 else "analysis"
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
    if r.get("mode") != "autopilot":
        continue
    pos = g(r, "sensors", "position") or {}
    t = g(r, "autopilot", "targets") or {}
    cx, cz = pos.get("comX"), pos.get("comZ")
    ts = r.get("timestampMs")
    if all(isinstance(v, (int, float)) for v in (cx, cz, ts)):
        rows.append((ts, cx, cz, t.get("x"), t.get("z")))
rows.sort(key=lambda x: x[0])
if not rows:
    print("no autopilot rows")
    sys.exit(0)

t0 = rows[0][0]
T = [(r[0]-t0)/1000.0 for r in rows]
X = [r[1] for r in rows]
Z = [r[2] for r in rows]
tgtX = next((r[3] for r in rows if isinstance(r[3], (int, float))), sum(X)/len(X))
tgtZ = next((r[4] for r in rows if isinstance(r[4], (int, float))), sum(Z)/len(Z))

fig, ax = plt.subplots(1, 3, figsize=(16, 5))
ax[0].plot(T, X, color="tab:blue"); ax[0].axhline(tgtX, color="k", ls="--", lw=0.8, label="target")
ax[0].set_title("CoM X vs time"); ax[0].set_xlabel("s"); ax[0].set_ylabel("X"); ax[0].grid(True, alpha=0.3); ax[0].legend()
ax[1].plot(T, Z, color="tab:orange"); ax[1].axhline(tgtZ, color="k", ls="--", lw=0.8, label="target")
ax[1].set_title("CoM Z vs time"); ax[1].set_xlabel("s"); ax[1].set_ylabel("Z"); ax[1].grid(True, alpha=0.3); ax[1].legend()
ax[2].plot(X, Z, color="tab:red", lw=0.8)
ax[2].scatter([tgtX], [tgtZ], c="k", marker="+", s=120, label="target")
ax[2].scatter([X[0]], [Z[0]], c="green", s=40, label="start")
ax[2].scatter([X[-1]], [Z[-1]], c="black", s=40, label="end")
ax[2].set_title("CoM path (orbit => rotated forcing)"); ax[2].set_xlabel("X"); ax[2].set_ylabel("Z")
ax[2].axis("equal"); ax[2].grid(True, alpha=0.3); ax[2].legend()
fig.tight_layout()
out = os.path.join(outdir, "hold_path.png")
fig.savefig(out, dpi=110)
print("wrote " + out)
print("X span %.2f  Z span %.2f  around target (%.1f, %.1f)" % (max(X)-min(X), max(Z)-min(Z), tgtX, tgtZ))
