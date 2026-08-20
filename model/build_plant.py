# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy"]
# ///
"""
MC Aero — analytical MIMO plant builder + discrete LQI synthesis.

Implements the derivation in docs/Analytical_MIMO_LQR_LQI_Plant_Model...md using
the parameters frozen in docs/plant_params.md. Produces continuous A,B, augments
with 4 integral states, checks controllability, discretizes at the control period,
solves the discrete LQI Riccati, and exports K + metadata to model/plant_K.lua.

Run:  uv run model/build_plant.py
"""
import numpy as np
from scipy.linalg import expm, solve_discrete_are

# ------------------------------------------------------------------ parameters
# Body frame (velocity-sensor frame, right-handed): x aft (forward=-x), y up,
# z left. Yaw psi about +y. Positions r in blocks from the CoM block.
M      = 714.0        # mass
I_PSI  = 69116.0      # yaw inertia Iyy (assembled)
G      = 11.0         # gravity (gimbal getGravity ~ 10.998)
DT     = 0.05         # control period (20 Hz)
# Command -> propeller-omega steady gain. RSC "target speed" drives ~0.10x the omega
# the k values were fit against (measured from clean pulse slopes: upDown 0.10,
# forwardBack 0.094, yaw 0.094, leftRight 0.13). First-cut global scale; refine
# per-channel with the automated open-loop sequencer.
CMD_GAIN = 0.10

# Physical bearings. sgn = sign of force along +d per +RPM command (handedness +
# gearing). PROVISIONAL until the per-channel RPM-step test; flagged below.
# d is the body-frame unit thrust direction (from the assembled dump).
BEARINGS = [
    # name,            channel,       r=(x,y,z),      d=(x,y,z),     k,      sgn, provisional
    ("main_lift_L",   "mainLift",    (0.0, 3.0, 11.0),  (0,1,0),  168.7,  +1, True),   # held const in test, sign assumed
    ("main_lift_R",   "mainLift",    (0.0, 3.0,-11.0),  (0,1,0),  168.7,  +1, True),
    ("forward_back",  "forwardBack", (21.0,0.0, 0.0),   (1,0,0),   78.4,  -1, False),  # pulse: +cmd -> forward (-x)
    ("translation_L", "leftRight",   (0.0, 0.0, 11.0),  (0,0,1),   15.1,  -1, False),  # pulse: +cmd -> right (-z)
    ("translation_R", "leftRight",   (0.0, 0.0,-11.0),  (0,0,-1),  15.1,  +1, False),
    ("yaw_L",         "yaw",         (14.0,0.0, 6.0),   (0,0,1),   15.1,  +1, False),  # pulse: G_M negative confirmed
    ("yaw_R",         "yaw",         (14.0,0.0,-6.0),   (0,0,-1),  15.1,  -1, False),
    ("up_down",       "upDown",      (0.0, 7.0, 0.0),   (0,1,0),   42.7,  +1, False),  # pulse: +cmd -> up
]

# Independent command channels (order defines the input vector u).
CHANNELS = ["mainLift", "forwardBack", "leftRight", "yaw", "upDown"]
TAU = {"mainLift": 0.59, "forwardBack": 0.51, "leftRight": 0.32, "yaw": 0.32, "upDown": 0.44}

# State ordering (n = 8 + N): [s_x, s_y, s_z, psi, v_x, v_y, v_z, r, w_1..w_N]
STATE = ["s_x", "s_y", "s_z", "psi", "v_x", "v_y", "v_z", "r"] + [f"w_{c}" for c in CHANNELS]
N = len(CHANNELS)
n = 8 + N

# ------------------------------------------------------------------ effectiveness
def cross_y(r, d):
    # (r x d)_y = r_z*d_x - r_x*d_z   (yaw about +y)
    return r[2] * d[0] - r[0] * d[2]

GF = np.zeros((3, N))   # body force per unit omega, per channel
GM = np.zeros(N)        # yaw moment per unit omega, per channel
for name, ch, r, d, k, sgn, prov in BEARINGS:
    j = CHANNELS.index(ch)
    d = np.array(d, float)
    r = np.array(r, float)
    GF[:, j] += sgn * k * d
    GM[j]    += sgn * k * cross_y(r, d)

# ------------------------------------------------------------------ continuous A,B
A = np.zeros((n, n))
B = np.zeros((n, N))
# kinematics: ds = v, dpsi = r
A[0, 4] = 1.0
A[1, 5] = 1.0
A[2, 6] = 1.0
A[3, 7] = 1.0
# translational + yaw dynamics from actuator RPM states
for j in range(N):
    col = 8 + j
    # CMD_GAIN folded into effectiveness so the actuator state is command-equivalent
    # (steady state = command), which makes the onboard propagation trivial.
    A[4, col] = CMD_GAIN * GF[0, j] / M
    A[5, col] = CMD_GAIN * GF[1, j] / M
    A[6, col] = CMD_GAIN * GF[2, j] / M
    A[7, col] = CMD_GAIN * GM[j] / I_PSI
    tau = TAU[CHANNELS[j]]
    A[col, col] = -1.0 / tau
    B[col, j]   = 1.0 / tau

# ------------------------------------------------------------------ LQI augment
# tracked outputs: s_x, s_y, s_z, psi (rows 0..3). xi_dot = -C_r x (regulator form)
Cr = np.zeros((4, n))
Cr[0, 0] = Cr[1, 1] = Cr[2, 2] = Cr[3, 3] = 1.0
nI = n + 4
AI = np.zeros((nI, nI))
AI[:n, :n] = A
AI[n:, :n] = -Cr
BI = np.zeros((nI, N))
BI[:n, :] = B
STATE_I = STATE + ["xi_sx", "xi_sy", "xi_sz", "xi_psi"]

# ------------------------------------------------------------------ checks
def ctrb_rank(a, b):
    n_ = a.shape[0]
    M_ = b.copy()
    blk = b.copy()
    for _ in range(1, n_):
        blk = a @ blk
        M_ = np.hstack([M_, blk])
    return np.linalg.matrix_rank(M_, tol=1e-6), n_

print("=== effectiveness (per channel) ===")
print("channel      G_Fx     G_Fy     G_Fz      G_M(yaw)")
for j, c in enumerate(CHANNELS):
    print("%-11s %8.2f %8.2f %8.2f  %10.2f" % (c, GF[0, j], GF[1, j], GF[2, j], GM[j]))

r_open, _ = ctrb_rank(A, B)
r_aug, _ = ctrb_rank(AI, BI)
print("\n=== controllability ===")
print("open plant (A,B):    rank %d / %d  %s" % (r_open, n, "OK" if r_open == n else "*** DEFICIENT ***"))
print("augmented (A_I,B_I): rank %d / %d  %s" % (r_aug, nI, "OK" if r_aug == nI else "*** DEFICIENT ***"))

# hover nominal for the main-lift channel (needs |G_Fy| for mainLift)
gfy_main = abs(GF[1, CHANNELS.index("mainLift")])
w_hover = M * G / (CMD_GAIN * gfy_main) if gfy_main > 1e-9 else float("nan")
print("\nhover nominal main-lift command* = m*g / (CMD_GAIN*G_Fy_main) = %.2f" % w_hover)

# ------------------------------------------------------------------ discretize
Maug = np.zeros((nI + N, nI + N))
Maug[:nI, :nI] = AI
Maug[:nI, nI:] = BI
Md = expm(Maug * DT)
Ad = Md[:nI, :nI]
Bd = Md[:nI, nI:]

# ------------------------------------------------------------------ Q, R (normalized, Bryson)
# Bryson-rule normalized weights. LARGER xmax => gentler (lower gain) on that
# state. These were relaxed after the first flight showed bang-bang saturation:
# the velocity and yaw-rate weights are loosened so commands stay proportional.
xmax = {
    "s_x": 5.0, "s_y": 3.0, "s_z": 5.0, "psi": np.deg2rad(20),
    "v_x": 6.0, "v_y": 3.0, "v_z": 6.0, "r": 4.0,
}
for c in CHANNELS:
    xmax[f"w_{c}"] = 256.0
# integral states: weight modestly (units block*s / rad*s)
xi_max = {"xi_sx": 8.0, "xi_sy": 5.0, "xi_sz": 8.0, "xi_psi": np.deg2rad(20)}
qdiag = []
for s in STATE_I:
    if s in xmax:
        qdiag.append(1.0 / xmax[s] ** 2)
    else:
        qdiag.append(0.1 / xi_max[s] ** 2)  # integral action, modest
Q = np.diag(qdiag)
# Per-channel control penalty (RAISE = gentler). Tuned from flight feedback:
# mainLift softened (altitude was a touch hot), yaw stiffened (heading slid in
# too slowly). upDown/forwardBack/leftRight left at the nominal.
RHO_CH = {
    "mainLift": 225.0,   # softer altitude
    "forwardBack": 150.0,
    "leftRight": 150.0,
    "yaw": 100.0,        # slightly more aggressive heading
    "upDown": 150.0,
}
R = np.diag([RHO_CH[c] / 256.0 ** 2 for c in CHANNELS])

# ------------------------------------------------------------------ discrete LQI
P = solve_discrete_are(Ad, Bd, Q, R)
K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

cl = np.linalg.eigvals(Ad - Bd @ K)
print("\n=== closed-loop (discrete) spectral radius ===")
print("max |eig| = %.5f  %s" % (np.max(np.abs(cl)), "STABLE" if np.max(np.abs(cl)) < 1.0 else "*** UNSTABLE ***"))
print("\n=== |K| max per input row (lower = gentler) ===")
for i, c in enumerate(CHANNELS):
    print("  %-11s  max|K| = %8.1f" % (c, np.max(np.abs(K[i]))))

# ------------------------------------------------------------------ export Lua
def lua_matrix(mat):
    rows = []
    for row in mat:
        rows.append("    { " + ", ".join("%.8g" % v for v in row) + " },")
    return "{\n" + "\n".join(rows) + "\n  }"

prov = any(b[6] for b in BEARINGS)
lua = []
lua.append("-- AUTO-GENERATED by model/build_plant.py — do not edit by hand.")
lua.append("-- Discrete LQI gain K (u = -K * x_I). Provisional signs: %s" % prov)
lua.append("return {")
lua.append('  meta = {')
lua.append('    dt = %.4f, mass = %.1f, I_psi = %.1f,' % (DT, M, I_PSI))
lua.append('    provisionalSigns = %s,' % ("true" if prov else "false"))
lua.append('    stateOrder = { "' + '", "'.join(STATE_I) + '" },')
lua.append('    inputOrder = { "' + '", "'.join(CHANNELS) + '" },')
lua.append('    hoverMainOmega = %.4f,' % w_hover)
lua.append('  },')
lua.append("  K = " + lua_matrix(K) + ",")
lua.append("}")
out_path = __file__.rsplit("/", 1)[0] + "/../aircraft/plant_K.lua"
with open(out_path, "w") as f:
    f.write("\n".join(lua) + "\n")
print("\nwrote %s  (K is %d x %d)" % (out_path, K.shape[0], K.shape[1]))
