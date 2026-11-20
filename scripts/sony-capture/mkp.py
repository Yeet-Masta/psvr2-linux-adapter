# Merge F lines with P lines built from Sony's poses (posepoll) -> trace.
import sys, numpy as np
f_path, cap, lag_ms = sys.argv[1], sys.argv[2], float(sys.argv[3])
P = {}
roles = None
for l in open(cap + '/poses.txt'):
    s = l.split()
    if s[0] == 'R': roles = (int(s[1]), int(s[2])); continue
    if s[2] != '1': continue
    P.setdefault(int(s[1]), []).append([float(x) for x in [s[0]] + s[4:16]])
P = {k: np.array(v) for k, v in P.items()}
def quat(R):
    w = np.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
    x = np.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2
    y = np.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2
    z = np.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2
    x = np.copysign(x, R[2,1] - R[1,2]); y = np.copysign(y, R[0,2] - R[2,0]); z = np.copysign(z, R[1,0] - R[0,1])
    return w, x, y, z
def at(dev, t_ns):
    a = P[dev]; i = np.searchsorted(a[:, 0], t_ns)
    if i <= 0 or i >= len(a) or a[i, 0] - a[i-1, 0] > 50e6: return None
    return a[i] if a[i, 0] - t_ns < t_ns - a[i-1, 0] else a[i-1]
Q = np.array([[1,0,0],[0,0,1],[0,-1,0]], float)  # device <- raw IMU (gyro fit)
out = sys.stdout
for l in open(f_path):
    out.write(l)
    fu = int(l.split()[1]); t = fu * 1000 - lag_ms * 1e6
    hp = at(0, t)
    if hp is None: continue
    M = hp[1:13].reshape(3, 4)
    w, x, y, z = quat(M[:, :3])
    parts = ['P %d 1 1 ' % fu, ' 1 %.6f %.6f %.6f %.7f %.7f %.7f %.7f 0 0 0 ' % (M[0,3], M[1,3], M[2,3], w, x, y, z)]
    for dev in roles:
        cp = at(dev, t)
        if cp is None: parts.append(' 0 1 0 0 0 0 224 '); continue
        R = cp[1:13].reshape(3, 4)[:, :3] @ Q  # world <- raw IMU
        parts.append(' 1 %.7f %.7f %.7f %.7f 0 224 ' % quat(R))
    out.write('|'.join(parts) + '\n')
