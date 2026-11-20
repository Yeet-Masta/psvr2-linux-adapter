import sys, struct, numpy as np
d = sys.argv[1]
roles = None; P = {}
for l in open(d + '/poses.txt'):
    f = l.split()
    if f[0] == 'R': roles = {'L': int(f[1]), 'R': int(f[2])}; continue
    dev = int(f[1])
    if f[2] != '1': continue
    v = list(map(float, f[4:]))
    P.setdefault(dev, []).append([int(f[0])] + v)
G = {'L': [], 'R': []}
for l in open(d + '/hid.txt'):
    if not l.startswith('IN') or 'Sense' not in l: continue
    h = 'R' if '(R)' in l else 'L'
    f = l.split()
    b = bytes(int(x, 16) for x in f[-1].split('\\x')[1:])
    if len(b) < 29 or b[0] != 0x31: continue
    g = struct.unpack_from('<3h', b, 17); a = struct.unpack_from('<3h', b, 23)
    G[h].append([int(f[1])] + list(g) + list(a))
np.set_printoptions(precision=4, suppress=True)
def kabsch(A, B):  # find Q minimizing |Q A - B|, A,B 3xN
    H = B @ A.T; U, S, Vt = np.linalg.svd(H); return U @ Vt
for h in 'RL':
    p = np.array(P[roles[h]], dtype=float); g = np.array(G[h], dtype=float)
    t = p[:, 0]; M = p[:, 1:13].reshape(-1, 3, 4)[:, :, :3]; w = p[:, 16:19]
    wb = np.einsum('nji,nj->ni', M, w)  # R^T w
    tg = g[:, 0]; gy = g[:, 1:4]; ac = g[:, 4:7]
    print('==', h, 'poses', len(t), 'gyro', len(tg), 'gyro rate %.0f Hz' % (len(tg) / ((tg[-1] - tg[0]) / 1e9)))
    best = None
    for lag in np.arange(-80, 81, 2) * 1e6:
        tt = tg - lag
        ok = (tt > t[0]) & (tt < t[-1])
        wi = np.stack([np.interp(tt[ok], t, wb[:, k]) for k in range(3)], 1)
        gi = gy[ok]
        m = np.linalg.norm(wi, axis=1) > 0.5
        A = gi[m].T; B = wi[m].T
        Q = kabsch(A / np.linalg.norm(A, axis=0), B / np.linalg.norm(B, axis=0))
        r = Q @ A; s = (r * B).sum() / (r * r).sum()
        err = np.linalg.norm(s * r - B, axis=0).mean()
        if best is None or err < best[0]: best = (err, lag, Q, s, m.sum())
    err, lag, Q, s, n = best
    print('lag %+.0f ms  n %d  mean err %.3f rad/s  scale %.6g rad/s/LSB (%.1f dps full)' % (lag / 1e6, n, err, s, s * 32768 * 57.2958))
    print('Q (device <- raw), det %.3f' % np.linalg.det(Q)); print(Q)
    # angular rate magnitude check
    print('  |w| p50 %.2f rad/s' % np.median(np.linalg.norm(wb, axis=1)))
