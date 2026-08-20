# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""Diagnose LQI position-hold: does the craft move toward or away from target?
Compares actual world CoM velocity direction to the desired-toward-target
direction; ~0deg = converging, ~90 = orbit, ~180 = runaway (wrong sign/rotation)."""
import sys, os, glob, json, math
import numpy as np

d = sys.argv[1] if len(sys.argv) > 1 else "/tmp/trans"
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
    pos = g(r, "sensors", "position") or {}
    en = ap.get("enable") or {}
    cmd = ap.get("command") or {}
    err = ap.get("errors") or {}
    tgt = ap.get("targets") or {}
    rows.append(dict(
        ts=r.get("timestampMs"), mode=r.get("mode"),
        posEn=en.get("position"), hdgEn=en.get("heading"),
        comX=pos.get("comX"), comZ=pos.get("comZ"),
        tgtX=tgt.get("x"), tgtZ=tgt.get("z"),
        s_x=err.get("s_x"), s_z=err.get("s_z"), r=err.get("r"),
        uLR=cmd.get("leftRight"), uFB=cmd.get("forwardBack"), uYaw=cmd.get("yaw"),
        heading=g(r,"sensors","navigation","getHeading"),
    ))
rows = [x for x in rows if x["ts"] is not None]
rows.sort(key=lambda x: x["ts"])
print("records:", len(recs), " rows:", len(rows))
auto = [x for x in rows if x["mode"] == "autopilot"]
print("autopilot rows:", len(auto))
if len(auto) < 10:
    print("mode counts:", {m: sum(1 for x in rows if x["mode"]==m) for m in set(x["mode"] for x in rows)})
    sys.exit(0)

print("position enabled:", set(x["posEn"] for x in auto), " heading enabled:", set(x["hdgEn"] for x in auto))
print("target X/Z:", auto[0]["tgtX"], auto[0]["tgtZ"])

def arr(k): return np.array([x[k] if isinstance(x[k],(int,float)) else np.nan for x in auto], float)
t = arr("ts")/1000.0
cx, cz = arr("comX"), arr("comZ")
tx, tz = arr("tgtX"), arr("tgtZ")
uLR, uFB, uYaw = arr("uLR"), arr("uFB"), arr("uYaw")

# distance to target over time
dist = np.hypot(cx - tx, cz - tz)
print("\ndist-to-target: start %.2f  end %.2f  min %.2f  max %.2f" % (dist[0], dist[-1], np.nanmin(dist), np.nanmax(dist)))
print("=> %s" % ("RUNAWAY (grows)" if dist[-1] > dist[0] + 2 else "bounded"))

# commands
print("\ncommand ranges: leftRight %.0f..%.0f  forwardBack %.0f..%.0f  yaw %.0f..%.0f"
      % (np.nanmin(uLR), np.nanmax(uLR), np.nanmin(uFB), np.nanmax(uFB), np.nanmin(uYaw), np.nanmax(uYaw)))
def frac_sat(u):
    m = np.isfinite(u); 
    return 100.0*np.mean(np.abs(u[m])>=255) if m.any() else 0
print("saturation %%: leftRight %.0f  forwardBack %.0f  yaw %.0f" % (frac_sat(uLR), frac_sat(uFB), frac_sat(uYaw)))

# velocity toward target: actual world CoM velocity vs desired (target-current)
vx = np.gradient(cx, t); vz = np.gradient(cz, t)
angs = []
for i in range(len(auto)):
    dx, dz = tx[i]-cx[i], tz[i]-cz[i]   # desired direction (toward target)
    nv, ne = math.hypot(vx[i],vz[i]), math.hypot(dx,dz)
    if np.isfinite(nv) and np.isfinite(ne) and nv>0.02 and ne>0.5:
        c = max(-1,min(1,(vx[i]*dx+vz[i]*dz)/(nv*ne)))
        angs.append(math.degrees(math.acos(c)))
if angs:
    a = sorted(angs)
    print("\nactual-velocity vs toward-target angle: mean %.0f  median %.0f deg (0=converge,90=orbit,180=runaway)"
          % (sum(angs)/len(angs), a[len(a)//2]))

# body error vs command sanity: leftRight should oppose s_z (u = -K s_z path)
s_z = arr("s_z"); 
m = np.isfinite(s_z) & np.isfinite(uLR)
if m.sum()>5:
    cc = np.corrcoef(s_z[m], uLR[m])[0,1]
    print("corr(s_z, leftRight cmd) = %.2f  (want strong; sign tells direction)" % cc)
s_x = arr("s_x")
m2 = np.isfinite(s_x) & np.isfinite(uFB)
if m2.sum()>5:
    print("corr(s_x, forwardBack cmd) = %.2f" % np.corrcoef(s_x[m2], uFB[m2])[0,1])

# samples
print("\nsamples:")
step=max(1,len(auto)//12)
for i in range(0,len(auto),step):
    x=auto[i]
    def f(v): return ("%.1f"%v) if isinstance(v,(int,float)) else "--"
    print("  d=%5s comX=%7s comZ=%8s s_x=%6s s_z=%6s uFB=%6s uLR=%6s uYaw=%6s"
          %(f(dist[i]),f(x["comX"]),f(x["comZ"]),f(x["s_x"]),f(x["s_z"]),f(x["uFB"]),f(x["uLR"]),f(x["uYaw"])))
