import sys, numpy as np
rep, trace, cap = sys.argv[1:4]; mount_deg = float(sys.argv[4]) if len(sys.argv) > 4 else 45
t0 = next(int(l.split()[1]) for l in open(trace) if l.startswith('P'))
P = {}; roles = None
for l in open(cap + '/poses.txt'):
    s = l.split()
    if s[0] == 'R': roles = (int(s[1]), int(s[2])); continue
    if s[2] != '1': continue
    P.setdefault(int(s[1]), []).append([float(x) for x in [s[0]] + s[4:16]])
P = {k: np.array(v) for k, v in P.items()}
def at(dev, t):
    a = P[dev]; i = np.searchsorted(a[:, 0], t)
    if i <= 0 or i >= len(a): return None
    M = a[i, 1:13].reshape(3, 4); U, _, Vt = np.linalg.svd(M[:, :3]); return U @ Vt, M[:, 3]
def qm(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def proj(S):
    U, _, Vt = np.linalg.svd(S); R = U @ Vt
    return R if np.linalg.det(R) > 0 else U @ np.diag([1, 1, -1]) @ Vt
def ang(R): return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
def axang(R):
    a = ang(R); v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]); n = np.linalg.norm(v)
    return a, v / n if n else v
c, s = np.cos(np.radians(mount_deg)), np.sin(np.radians(mount_deg))
M = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])  # IMU <- model
np.set_printoptions(precision=4, suppress=True)
data = {0: [], 1: []}
for l in open(rep):
    if not l.startswith('FIX'): continue
    f = l.split(); hand = 1 if f[2] == 'right' else 0
    try: x = np.array(f[4:7], float); q = np.array(f[10:14], float)
    except (ValueError, IndexError): continue
    t = (t0 + float(f[1]) * 1e6) * 1000
    d = at(roles[hand], t); h = at(0, t)
    if d is None or h is None: continue
    data[hand].append((d, h, qm(*q) @ M, x))
allr = data[0] + data[1]
G = np.eye(3); Cs = {0: np.eye(3), 1: np.eye(3)}
for it in range(30):
    for hand in (0, 1):
        Cs[hand] = proj(sum(d[0].T @ h[0] @ G @ h[0].T @ Ro for d, h, Ro, x in data[hand]))
    G = proj(sum(h[0].T @ d[0] @ Cs[hand] @ Ro.T @ h[0] for hand in (0, 1) for d, h, Ro, x in data[hand]))
for hand in (0, 1):
    e = [ang(Cs[hand].T @ d[0].T @ h[0] @ G @ h[0].T @ Ro) for d, h, Ro, x in data[hand]]
    e0 = [ang(Cs[hand].T @ d[0].T @ Ro) for d, h, Ro, x in data[hand]]
    a, v = axang(Cs[hand] @ M.T)
    print('LR'[hand], 'n', len(e), 'rot residual p50 %.1f p90 %.1f deg (without G: p50 %.1f)' % (np.median(e), np.percentile(e, 90), np.median([ang(proj(sum(d[0].T @ Ro for d, h, Ro, x in data[hand])).T @ d[0].T @ Ro) for d, h, Ro, x in data[hand]])))
    print('  device <- IMU: %.1f deg about %s' % (a, v))
a, v = axang(G); print('head-frame rotation correction G: %.2f deg about %s' % (a, v))
# translation: p_d + R_d o_hand = p_h + R_h (delta + G x_h)
rows = []; rhs = []
for hand in (0, 1):
    for d, h, Ro, x in data[hand]:
        Rd, pd = d; Rh, ph = h
        xh = Rh.T @ (x - ph)
        A = np.zeros((3, 9)); A[:, 3*hand:3*hand+3] = Rd; A[:, 6:9] = -Rh
        rows.append(A); rhs.append(ph + Rh @ (G @ xh) - pd)
A = np.vstack(rows); b = np.concatenate(rhs)
keep = np.ones(len(b), bool)
for it in range(3):
    sol, *_ = np.linalg.lstsq(A[keep], b[keep], rcond=None)
    r = np.linalg.norm((A @ sol - b).reshape(-1, 3), axis=1); keep = np.repeat(r < 3 * np.median(r), 3)
n0 = len(data[0])
print('IMU point in device frame L', sol[0:3], 'R', sol[3:6], 'm')
print('head-frame translation correction delta', sol[6:9], 'm')
print('pos residual L p50 %.1f p90 %.1f mm, R p50 %.1f p90 %.1f mm' % (np.median(r[:n0]) * 1e3, np.percentile(r[:n0], 90) * 1e3, np.median(r[n0:]) * 1e3, np.percentile(r[n0:], 90) * 1e3))
