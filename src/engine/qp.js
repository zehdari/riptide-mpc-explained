// Port of riptide_mpc box_qp.cpp:
//   minimize 0.5 x'Hx + g'x  s.t. lb <= x <= ub
// Projected Newton (Bertsekas 1982): Newton step on the free set, projected Armijo line search.
// H is a row-major Float64Array (n x n). x holds the warm start on entry.
import { cholSolveInPlace } from './math.js';

export function solveBoxQp(H, g, lb, ub, x, n, maxIterations = 50, tolerance = 1e-8, trace = null) {
  const objective = (v) => {
    let s = 0;
    for (let i = 0; i < n; i++) {
      let hv = 0;
      const row = i * n;
      for (let j = 0; j < n; j++) hv += H[row + j] * v[j];
      s += v[i] * (0.5 * hv + g[i]);
    }
    return s;
  };
  for (let i = 0; i < n; i++) x[i] = Math.min(ub[i], Math.max(lb[i], x[i]));
  const result = { iterations: 0, converged: false };
  const gradient = new Float64Array(n);
  const candidate = new Float64Array(n);
  const free = new Int32Array(n);
  if (trace) trace.push(Array.from(x));
  for (; result.iterations < maxIterations; result.iterations++) {
    for (let i = 0; i < n; i++) {
      let s = g[i];
      const row = i * n;
      for (let j = 0; j < n; j++) s += H[row + j] * x[j];
      gradient[i] = s;
    }
    const eps = 1e-9;
    let m = 0, stationarity = 0;
    for (let i = 0; i < n; i++) {
      const atLower = x[i] <= lb[i] + eps && gradient[i] > 0;
      const atUpper = x[i] >= ub[i] - eps && gradient[i] < 0;
      if (!atLower && !atUpper) {
        free[m++] = i;
        stationarity = Math.max(stationarity, Math.abs(gradient[i]));
      }
    }
    if (m === 0 || stationarity < tolerance) {
      result.converged = true;
      break;
    }
    const Hff = new Float64Array(m * m);
    const df = new Float64Array(m);
    for (let a = 0; a < m; a++) {
      df[a] = -gradient[free[a]];
      const row = free[a] * n;
      for (let b = 0; b < m; b++) Hff[a * m + b] = H[row + free[b]];
    }
    cholSolveInPlace(Hff, m, df);
    const f0 = objective(x);
    let alpha = 1;
    for (let backtrack = 0; backtrack < 30; backtrack++, alpha *= 0.5) {
      candidate.set(x);
      for (let a = 0; a < m; a++) {
        const i = free[a];
        candidate[i] = Math.min(ub[i], Math.max(lb[i], x[i] + alpha * df[a]));
      }
      let decrease = 0;
      for (let i = 0; i < n; i++) decrease += gradient[i] * (candidate[i] - x[i]);
      if (objective(candidate) <= f0 + 1e-4 * decrease) break;
    }
    let change = 0;
    for (let i = 0; i < n; i++) change = Math.max(change, Math.abs(candidate[i] - x[i]));
    x.set(candidate);
    if (trace) trace.push(Array.from(x));
    if (change < 1e-12) {
      result.converged = true;
      break;
    }
  }
  return result;
}
