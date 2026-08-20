#!/usr/bin/env python3
# Diagnose a horizontal-hold autopilot run near the lodestone.
# Hypothesis: near-overhead the reconstruction is singular -> horiz ~ 0, bearing
# ill-defined, so comX/comZ jitter and the loop can't settle.
import sys, os, glob, json, math

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ap90"
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

def stats(v):
    v = [x for x in v if isinstance(x, (int, float))]
    if not v:
        return None
    m = sum(v)/len(v)
    sd = (sum((x-m)**2 for x in v)/len(v))**0.5
    sv = sorted(v)
    return (min(v), max(v), m, sd, sv[len(sv)//2])

def circ_sd(v):
    v = [x for x in v if isinstance(x, (int, float))]
    if not v:
        return None
    xs = sum(math.cos(math.radians(a)) for a in v)/len(v)
    ys = sum(math.sin(math.radians(a)) for a in v)/len(v)
    R = min(1.0, max(1e-12, math.hypot(xs, ys)))
    return math.degrees(math.sqrt(-2*math.log(R)))

rows = []
for r in recs:
    if r.get("mode") != "autopilot":
        continue
    nav = g(r, "sensors", "navigation") or {}
    pos = g(r, "sensors", "position") or {}
    ap = r.get("autopilot") or {}
    h = ap.get("horizontal") or {}
    t = ap.get("targets") or {}
    yawrsc = g(r, "actuators", "rsc", "forwardBack") or {}
    lrrsc = g(r, "actuators", "rsc", "leftRight") or {}
    dist = nav.get("getDistanceToTarget")
    voff = nav.get("getVerticalOffsetToTarget")
    horiz = None
    if isinstance(dist, (int, float)) and isinstance(voff, (int, float)):
        sq = dist*dist - voff*voff
        horiz = math.sqrt(sq) if sq > 0 else 0.0
    rows.append(dict(
        ts=r.get("timestampMs"), dist=dist, voff=voff, horiz=horiz,
        bearing=nav.get("getBearing"), heading=nav.get("getHeading"),
        comX=pos.get("comX"), comZ=pos.get("comZ"),
        navX=pos.get("x"), navZ=pos.get("z"),
        tgtX=t.get("x"), tgtZ=t.get("z"),
        eX=h.get("xError"), eZ=h.get("zError"),
        uFB=h.get("forwardBackCommand"), uLR=h.get("leftRightCommand"),
    ))
rows.sort(key=lambda x: (x["ts"] or 0))

print("records=%d  autopilot rows=%d" % (len(recs), len(rows)))
if not rows:
    sys.exit(0)

def col(name):
    return [r[name] for r in rows]

def line(label, s):
    if s:
        print("  %-16s min %.2f  max %.2f  mean %.2f  sd %.2f  med %.2f" % (label, s[0], s[1], s[2], s[3], s[4]))
    else:
        print("  %-16s (no data)" % label)

print("\n=== distance geometry (is it overhead the lodestone?) ===")
line("dist(slant)", stats(col("dist")))
line("vOff(vertical)", stats(col("voff")))
line("horiz(to lode)", stats(col("horiz")))
print("  --> if horiz is small vs vOff, the craft is nearly straight above it.")

print("\n=== bearing behavior (singularity => wild swings) ===")
print("  bearing circular sd : %.1f deg" % (circ_sd(col("bearing")) or -1))
print("  heading circular sd : %.1f deg" % (circ_sd(col("heading")) or -1))

print("\n=== CoM position estimate (jitter of the proxy) ===")
line("comX", stats(col("comX")))
line("comZ", stats(col("comZ")))
line("navX", stats(col("navX")))
line("navZ", stats(col("navZ")))

print("\n=== controller error and command ===")
line("xError", stats(col("eX")))
line("zError", stats(col("eZ")))
line("uForwardBack", stats(col("uFB")))
line("uLeftRight", stats(col("uLR")))

# tick-to-tick jump in comX/comZ = proxy noise
jumps = []
for i in range(1, len(rows)):
    a, b = rows[i-1], rows[i]
    if all(isinstance(v, (int, float)) for v in (a["comX"], b["comX"], a["comZ"], b["comZ"])):
        jumps.append(math.hypot(b["comX"]-a["comX"], b["comZ"]-a["comZ"]))
if jumps:
    js = stats(jumps)
    print("\n=== tick-to-tick CoM jump (proxy noise, blocks/tick) ===")
    print("  mean %.3f  median %.3f  max %.3f" % (js[2], js[4], js[1]))

import math as _m
# Angle between actual CoM velocity (finite diff) and the position error vector
# (error points toward target). Converging ~0deg, orbiting ~90deg, diverging ~180.
angs = []
signed = []
for i in range(1, len(rows)):
    a, b = rows[i-1], rows[i]
    if all(isinstance(v, (int, float)) for v in (a["comX"], b["comX"], a["comZ"], b["comZ"], b["eX"], b["eZ"])):
        vx, vz = b["comX"]-a["comX"], b["comZ"]-a["comZ"]
        ex, ez = b["eX"], b["eZ"]
        nv, ne = _m.hypot(vx, vz), _m.hypot(ex, ez)
        if nv > 1e-4 and ne > 1e-3:
            dot = (vx*ex + vz*ez)/(nv*ne)
            dot = max(-1.0, min(1.0, dot))
            angs.append(_m.degrees(_m.acos(dot)))
            cross = vx*ez - vz*ex
            signed.append(1 if cross > 0 else -1)
if angs:
    angs_s = sorted(angs)
    print("\n=== velocity vs error angle (assumption-free) ===")
    print("  mean %.1f deg  median %.1f deg  (0=converging, 90=orbit, 180=diverging)"
          % (sum(angs)/len(angs), angs_s[len(angs_s)//2]))
    print("  rotation sense (majority): %s" % ("CCW" if sum(signed) > 0 else "CW"))

print("\n=== samples every ~1/12 ===")
step = max(1, len(rows)//12)
for i in range(0, len(rows), step):
    r = rows[i]
    def f(x):
        return ("%.1f" % x) if isinstance(x, (int, float)) else "--"
    print("  horiz=%6s brg=%7s comX=%7s comZ=%8s eX=%6s eZ=%6s uFB=%6s uLR=%6s"
          % (f(r["horiz"]), f(r["bearing"]), f(r["comX"]), f(r["comZ"]), f(r["eX"]), f(r["eZ"]), f(r["uFB"]), f(r["uLR"])))
