// Small vector / quaternion helpers. Vectors are [x, y, z]; quaternions are
// [w, x, y, z] mapping body to world (Eigen's convention in riptide_mpc).

export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const normalize = (a) => {
  const n = norm(a);
  return n > 1e-12 ? scale(a, 1 / n) : [0, 0, 0];
};
export const zero3 = () => [0, 0, 0];
export const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

export const qid = () => [1, 0, 0, 0];
export const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
export const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qnormalize = (q) => {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
};
// Rotate v by unit quaternion q.
export function qrot(q, v) {
  const [w, x, y, z] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}
export const qrotInv = (q, v) => qrot(qconj(q), v);

export function qexp(v) {
  const angle = norm(v);
  if (angle < 1e-12) return qnormalize([1, v[0] / 2, v[1] / 2, v[2] / 2]);
  const s = Math.sin(angle / 2) / angle;
  return [Math.cos(angle / 2), v[0] * s, v[1] * s, v[2] * s];
}
// Rotation vector of q, shortest rotation.
export function qlog(input) {
  let q = qnormalize(input);
  if (q[0] < 0) q = q.map((c) => -c);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-12) return [2 * q[1], 2 * q[2], 2 * q[3]];
  const k = (2 * Math.atan2(s, q[0])) / s;
  return [q[1] * k, q[2] * k, q[3] * k];
}
export function angularDistance(a, b) {
  const d = qmul(qconj(a), b);
  return 2 * Math.atan2(Math.hypot(d[1], d[2], d[3]), Math.abs(d[0]));
}
export const yawQuat = (yaw) => [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)];
// tf2 setRPY: R = Rz(yaw) Ry(pitch) Rx(roll)
export function rpyQuat(r, p, y) {
  return qnormalize(qmul(qmul(yawQuat(y), [Math.cos(p / 2), 0, Math.sin(p / 2), 0]), [Math.cos(r / 2), Math.sin(r / 2), 0, 0]));
}
export function quatToRpy(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(clamp(2 * (w * y - z * x), -1, 1));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}
export const twist = (q) => 2 * Math.atan2(q[3], q[0]);
export const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// Dense row-major helpers on Float64Array.
export function cholSolveInPlace(A, n, b) {
  // A (n x n, row-major) is overwritten with its Cholesky factor; b becomes the solution.
  for (let j = 0; j < n; j++) {
    let s = A[j * n + j];
    for (let k = 0; k < j; k++) s -= A[j * n + k] * A[j * n + k];
    if (s <= 0) return false;
    const d = Math.sqrt(s);
    A[j * n + j] = d;
    for (let i = j + 1; i < n; i++) {
      let t = A[i * n + j];
      for (let k = 0; k < j; k++) t -= A[i * n + k] * A[j * n + k];
      A[i * n + j] = t / d;
    }
  }
  for (let i = 0; i < n; i++) {
    let t = b[i];
    for (let k = 0; k < i; k++) t -= A[i * n + k] * b[k];
    b[i] = t / A[i * n + i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let t = b[i];
    for (let k = i + 1; k < n; k++) t -= A[k * n + i] * b[k];
    b[i] = t / A[i * n + i];
  }
  return true;
}

export function invert6(M) {
  // M: array of 6 rows; returns inverse via Gauss-Jordan.
  const n = 6, a = M.map((r) => r.slice()), inv = M.map((_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    [inv[c], inv[p]] = [inv[p], inv[c]];
    const d = a[c][c];
    for (let j = 0; j < n; j++) { a[c][j] /= d; inv[c][j] /= d; }
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = a[r][c];
      if (f === 0) continue;
      for (let j = 0; j < n; j++) { a[r][j] -= f * a[c][j]; inv[r][j] -= f * inv[c][j]; }
    }
  }
  return inv;
}

export const matVec6 = (M, v) => M.map((row) => row.reduce((s, m, j) => s + m * v[j], 0));
