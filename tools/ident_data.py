"""Re-derive the charts for the identification chapter from a pool_identify session.

Replicates identification.cpp's drag cruise selection, drag fit and inertia lag check
on the session's recording.csv, checks them against its report.yaml, and writes
public/data/ident_session.json.
usage: ident_data.py <session dir> <out.json>
"""
import csv, json, sys
import numpy as np
import yaml

d, out = sys.argv[1], sys.argv[2]
G = 9.80665
MASS = 34.307
rep = yaml.safe_load(open(f'{d}/report.yaml'))
segs = {int(r['id']): r for r in csv.DictReader(open(f'{d}/segments.csv'))}
rows = list(csv.DictReader(open(f'{d}/recording.csv')))
cols = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'upx', 'upy', 'upz', 'jx', 'jy', 'jz', 'jroll', 'jpitch', 'jyaw']
S = {k: np.array([float(r[k]) for r in rows]) for k in cols + ['t', 'dt']}
seg = np.array([int(r['segment']) for r in rows])
t0 = S['t'][0]

B = rep['statics']['net_buoyancy_N']
W = MASS * G
rcob = np.array(rep['statics']['cob_relative'])
up = np.stack([S['upx'], S['upy'], S['upz']], 1)
force_hold = -(B - W) * up
torque_hold = np.cross(up, B * rcob)
J = np.stack([S[k] for k in ['jx', 'jy', 'jz', 'jroll', 'jpitch', 'jyaw']], 1)
excess = J / S['dt'][:, None] - np.concatenate([force_hold, torque_hold], 1)
vel = {0: S['vx'], 1: S['vy'], 2: S['vz'], 5: S['wz']}

def runs(axis):
    return [i for i, s in segs.items() if s['kind'] == 'run' and int(s['axis']) == axis]

def drag_samples(axis):
    xs, ys = [], []
    for i in runs(axis):
        m = np.where(seg == i)[0]
        sgn = float(segs[i]['direction'])
        s = sgn * vel[axis][m]
        if len(s) == 0: continue
        smax = np.percentile(s, 90)
        if smax < 0.02: continue
        c = m[s >= 0.9 * smax]
        if len(c) < 5: continue
        c = c[1:-1]
        a = np.abs(vel[axis][c]); med = np.median(a)
        c = c[np.abs(a - med) <= 0.07 * med]
        xs += list(vel[axis][c]); ys += list(excess[c, axis])
    return np.array(xs), np.array(ys)

def drag_fit(x, y):
    q1, q2 = x, np.abs(x) * x
    s11, s12, s22, b1, b2 = (q1 * q1).sum(), (q1 * q2).sum(), (q2 * q2).sum(), (q1 * y).sum(), (q2 * y).sum()
    det = s11 * s22 - s12 * s12
    D1, D2 = (b1 * s22 - b2 * s12) / det, (s11 * b2 - s12 * b1) / det
    if D1 < 0: D1, D2 = 0, b2 / s22
    if D2 < 0: D2, D1 = 0, b1 / s11
    return D1, D2

def inertia(axis, D1, D2, lag):
    Js, dvs = [], []
    for i in runs(axis):
        m = np.where(seg == i)[0]
        if len(m) < 4: continue
        sgn = float(segs[i]['direction'])
        v = vel[axis][m]; s = sgn * v; t = S['t'][m]; dt = S['dt'][m]
        smax = s.max()
        if smax < 0.02: continue
        C = np.concatenate([[0.0], np.cumsum(excess[m, axis] * dt)])
        tc = np.concatenate([[t[0] - dt[0]], t])
        Cat = lambda tt: np.interp(tt, tc, C)  # flat outside the segment
        wins = []
        i0 = np.argmax(s >= 0.05 * smax) if (s >= 0.05 * smax).any() else None
        if i0 is not None and i0 > 0:
            after = np.where(s[i0:] >= 0.9 * smax)[0]
            if len(after):
                i1 = i0 + after[0]
                wins.append((i0 - 1, i1))
                hi = np.where(s >= 0.9 * smax)[0]
                j0 = hi[-1]
                low = np.where(s[j0:] <= 0.05 * smax)[0]
                if len(low) and j0 > i1:
                    wins.append((j0, j0 + low[0]))
        for a, b in wins:
            vb = 0.5 * (v[a:b] + v[a + 1:b + 1])
            drag = ((D1 * vb + D2 * np.abs(vb) * vb) * dt[a + 1:b + 1]).sum()
            Js.append(Cat(t[b] - lag) - Cat(t[a] - lag) - drag)
            dvs.append(v[b] - v[a])
    Js, dvs = np.array(Js), np.array(dvs)
    M = (Js * dvs).sum() / (dvs * dvs).sum()
    r2 = 1 - ((Js - M * dvs) ** 2).sum() / ((Js - Js.mean()) ** 2).sum()
    return M, r2, len(Js)

data = {'statics': rep['statics'], 'W': W, 'axes': {}, 'thrusters': rep['thrusters']}
for axis, name in [(0, 'surge'), (1, 'sway'), (5, 'yaw')]:
    x, y = drag_samples(axis)
    D1, D2 = drag_fit(x, y)
    r = rep[name]
    lags = [round(0.05 * k, 2) for k in range(13)]
    lagfit = [inertia(axis, r['linear_damping'], r['quadratic_damping'], L) for L in lags]
    print(name, 'drag', round(D1, 2), round(D2, 2), 'report', round(r['linear_damping'], 2), round(r['quadratic_damping'], 2),
          '| M lag0', round(lagfit[0][0], 2), 'report', round(r['total_inertia'], 2), 'windows', lagfit[0][2], 'r2', round(lagfit[0][1], 3))
    data['axes'][name] = {
        'samples': [[round(a, 4), round(b, 3)] for a, b in zip(x, y)],
        'D1': r['linear_damping'], 'D2': r['quadratic_damping'], 'D1_prior': r['linear_damping_prior'], 'D2_prior': r['quadratic_damping_prior'],
        'drag_r2': r['drag_r2'], 'levels': r['speed_levels'],
        'M': r['total_inertia'], 'added': r['added_mass'], 'inertia_r2': r['inertia_r2'],
        'lag': [{'lag': L, 'M': round(f[0], 3), 'r2': round(f[1], 4)} for L, f in zip(lags, lagfit)],
    }
for name in ['heave', 'roll', 'pitch']:
    data['axes'][name] = {k: rep[name].get(k) for k in ['drag_accepted', 'added_mass_accepted', 'note']}
# Session timeline at ~2 Hz: time, block, body velocity and yaw rate.
kinds = {i: (s['kind'], int(s['axis']), s['label']) for i, s in segs.items()}
tl = []
for k in range(0, len(rows), 4):
    kind, axis, label = kinds[seg[k]]
    tl.append([round(S['t'][k] - t0, 2), int(seg[k]), round(S['vx'][k], 3), round(S['vy'][k], 3), round(S['vz'][k], 3), round(S['wz'][k], 3)])
data['timeline'] = tl
data['segments'] = {i: {'kind': s['kind'], 'axis': int(s['axis']), 'label': s['label'], 'speed': float(s['speed'])} for i, s in segs.items()}
json.dump(data, open(out, 'w'))
print('wrote', out)
