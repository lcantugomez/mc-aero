# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""Fit thrust vs RPM per bearing from a sweep. Compares linear (T=k*w) vs
quadratic (T=a*w^2+b*w) fits to see the new curve shape and remap gains."""
import sys, os, glob, json
import numpy as np

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sw"
recs = []
for fn in glob.glob(os.path.join(d, "*.ndjson")):
    with open(fn) as fh:
        for line in fh:
            line = line.strip()
            if line:
                try: recs.append(json.loads(line))
                except Exception: pass
print("records:", len(recs))
if not recs:
    sys.exit(0)

def g(r, *p):
    cur = r
    for k in p:
        if not isinstance(cur, dict): return None
        cur = cur.get(k)
    return cur

# ---- structure probe ----
act = g(recs[0], "actuators") or {}
print("actuators keys:", sorted(act.keys()) if isinstance(act, dict) else act)
bl = act.get("bearings")
if isinstance(bl, list) and bl:
    print("bearing[0] keys:", sorted(bl[0].keys()))
    print("bearing[0]:", {k: bl[0][k] for k in bl[0]})
rsc = act.get("rsc")
if isinstance(rsc, dict):
    ch = next(iter(rsc))
    print("rsc[%s] keys:" % ch, sorted(rsc[ch].keys()))
    print("rsc[%s]:" % ch, rsc[ch])

# ---- collect (speed, thrust) per bearing across the sweep ----
# try common field names
SPEED_KEYS = ["getSpeed", "getRotationSpeed", "getAngularSpeed", "speed", "rpm"]
THRUST_KEYS = ["getThrust", "thrust"]

def pick(entry, keys):
    for k in keys:
        v = entry.get(k)
        if isinstance(v, (int, float)):
            return v, k
    return None, None

series = {}  # bearing name/role -> list of (speed, thrust)
speed_key_used, thrust_key_used = None, None
for r in recs:
    for b in (g(r, "actuators", "bearings") or []):
        if not isinstance(b, dict):
            continue
        name = b.get("role") or b.get("name") or "?"
        s, sk = pick(b, SPEED_KEYS)
        t, tk = pick(b, THRUST_KEYS)
        if s is None or t is None:
            continue
        speed_key_used, thrust_key_used = sk, tk
        series.setdefault(name, []).append((s, t))

print("\nspeed field:", speed_key_used, " thrust field:", thrust_key_used)
if not series:
    print("!! no (speed,thrust) pairs found in actuators.bearings -- need different fields")
    sys.exit(0)

def r2(T, pred):
    ss = np.sum((T - T.mean())**2) + 1e-12
    return 1 - np.sum((T - pred)**2) / ss

print("\n=== per-bearing: bin by |rpm|, average thrust (kills ramp hysteresis) ===")
for name in sorted(series):
    pts = [(s, t) for (s, t) in series[name] if abs(s) > 1e-6]
    if len(pts) < 20:
        print("  %-18s only %d pts (not swept)" % (name, len(pts)))
        continue
    w = np.array([p[0] for p in pts], float)
    T = np.array([p[1] for p in pts], float)
    s = np.sign(np.sum(w))  # dominant sign of rpm for this bearing
    w, T = w * s, T * s     # work in positive-rpm space
    # bin
    nb = 14
    edges = np.linspace(0, w.max(), nb + 1)
    bw, bT = [], []
    for i in range(nb):
        m = (w >= edges[i]) & (w < edges[i+1])
        if m.sum() >= 3:
            bw.append(w[m].mean()); bT.append(T[m].mean())
    bw, bT = np.array(bw), np.array(bT)
    if len(bw) < 5:
        print("  %-18s too few bins" % name); continue
    # fits on binned means
    kl = float(np.sum(bw*bT)/np.sum(bw*bw))
    cl = float(np.sum(bw*bw*bT)/np.sum(bw**4))          # pure quad T=c*w^2
    A = np.vstack([bw*bw, bw]).T
    aq, bq = np.linalg.lstsq(A, bT, rcond=None)[0]
    print("\n  %-18s bins=%d  rpm max=%.0f  thrust max=%.0f" % (name, len(bw), bw.max(), bT.max()))
    print("      linear  T=%.3f*w              R2=%.4f" % (kl, r2(bT, kl*bw)))
    print("      pure-q  T=%.5f*w^2           R2=%.4f" % (cl, r2(bT, cl*bw*bw)))
    print("      quad    T=%.5f*w^2 + %.3f*w  R2=%.4f" % (aq, bq, r2(bT, aq*bw*bw+bq*bw)))
    print("      curve (rpm -> thrust):")
    print("      " + "  ".join("%.0f:%.0f" % (bw[i], bT[i]) for i in range(len(bw))))
