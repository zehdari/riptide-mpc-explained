// Port of riptide_mpc mpc.cpp (MpcController). Single-shooting, condensed
// Gauss-Newton MPC: roll the Fossen model forward over the horizon with the
// warm-started thruster commands, linearize each stage by finite differences on
// the state manifold, build a dense QP in the commands only, solve it with
// box bounds (projected Newton), apply the first command. One SQP iteration per
// tick by default (real-time iteration).
import { FossenModel, ThrusterReplica } from './model.js';
import { MotionProfile } from './profile.js';
import { solveBoxQp } from './qp.js';
import { settings as defaults } from './talos.js';
import { qexp, qlog, qmul, qconj, qnormalize, qrot, qrotInv, cross, sub, norm, angularDistance, clamp, dot } from './math.js';

export const Mode = { DISABLED: 0, FEEDFORWARD: 1, VELOCITY: 2, POSITION: 3 };

function cloneStage(s) {
  return {
    pose: s.pose.clone(),
    linear_velocity: s.linear_velocity.slice(),
    angular_velocity: s.angular_velocity.slice(),
    progress: { ...s.progress },
  };
}

export class MpcController {
  constructor(model = new FossenModel(), settings = {}) {
    this.model = model;
    this.s = { ...defaults, ...settings, motion: { ...defaults.motion, ...(settings.motion || {}) } };
    this.nu = model.n;
    this.nx = 12 + this.nu;
    this.actuator = new ThrusterReplica(model);
    this.motion = { ...this.s.motion };
    this.lastCommand = new Array(this.nu).fill(0);
    this.lastDeviation = new Array(this.nu).fill(0);
    this.issuedFeedforward = new Array(this.nu).fill(0);
    this.warm = null;
    this.live = { pose: new MotionProfile(), linear_velocity: [0, 0, 0], angular_velocity: [0, 0, 0], progress: { s: 0, v: 0, a: 0 } };
    this.target = { linear_mode: Mode.DISABLED, angular_mode: Mode.DISABLED, path: null };
    this.linearSeeded = this.angularSeeded = false;
    this.debug = false;
    this.applyBounds();
  }

  applyBounds() {
    const a = this.model.act, N = this.s.horizon;
    this.lb = new Float64Array(N * this.nu).fill(-a.reverse_max);
    this.ub = new Float64Array(N * this.nu).fill(a.forward_max);
  }
  setHorizon(N) {
    this.s.horizon = N;
    this.applyBounds();
    this.warm = null;
  }
  referenceLead() {
    return this.s.compensate_delay ? this.model.act.delay : 0;
  }
  advance(dt) {
    if (dt <= 0) return;
    this.actuator.advance(dt);
    this.stepReference(this.live, this.target, dt);
  }
  issue(cmd) {
    this.actuator.command(cmd);
    this.lastCommand = cmd.slice();
    this.lastDeviation = cmd.map((c, i) => c - this.issuedFeedforward[i]);
  }
  clearWarmStart() {
    this.warm = null;
    this.issuedFeedforward.fill(0);
    this.linearSeeded = this.angularSeeded = false;
  }

  stepReference(s, r, dt) {
    const m = this.motion;
    if (r.path && r.linear_mode === Mode.POSITION && r.angular_mode === Mode.POSITION) {
      r.path.step(s.progress, m.linear_speed / this.s.motion.linear_speed, m.angular_speed / this.s.motion.angular_speed, dt);
      r.path.pose(s.progress, s.pose);
      return;
    }
    if (r.linear_mode === Mode.POSITION) s.pose.stepLinear(r.position, m, dt);
    else if (r.linear_mode === Mode.VELOCITY) s.linear_velocity = MotionProfile.rateLimit(s.linear_velocity, r.linear_velocity, m.linear_accel, dt);
    if (r.angular_mode === Mode.POSITION) s.pose.stepAngular(r.orientation, m, dt);
    else if (r.angular_mode === Mode.VELOCITY) s.angular_velocity = MotionProfile.rateLimit(s.angular_velocity, r.angular_velocity, m.angular_accel, dt);
  }

  seedReference(x, r) {
    const q = qnormalize([x[3], x[4], x[5], x[6]]);
    const p = this.model.baseLinkPosition(x), vBody = this.model.baseLinkVelocity(x), wBody = [x[10], x[11], x[12]];
    const live = this.live;
    if (r.linear_mode !== this.target.linear_mode) {
      this.linearSeeded = false;
      live.linear_velocity = vBody;
    }
    if (r.angular_mode !== this.target.angular_mode) {
      this.angularSeeded = false;
      live.angular_velocity = wBody;
    }
    if (r.path && r.linear_mode === Mode.POSITION && r.angular_mode === Mode.POSITION) {
      const vWorld = qrot(q, vBody);
      if (r.path !== this.target.path) {
        live.progress = { s: 0, v: Math.max(0, dot(r.path.tangent(0), this.linearSeeded ? live.pose.velocity : vWorld)), a: 0 };
      } else if (!this.linearSeeded || norm(sub(live.pose.position, p)) > this.s.max_reference_lag) {
        live.progress.s = r.path.project(p, live.progress.s, 2.0);
        live.progress.v = Math.max(0, dot(r.path.tangent(live.progress.s), vWorld));
        live.progress.a = 0;
      }
      r.path.pose(live.progress, live.pose);
      this.linearSeeded = this.angularSeeded = true;
      this.target = r;
      return;
    }
    if (r.linear_mode === Mode.POSITION && (!this.linearSeeded || norm(sub(live.pose.position, p)) > this.s.max_reference_lag)) {
      live.pose.position = p;
      live.pose.velocity = qrot(q, vBody);
      live.pose.acceleration = [0, 0, 0];
      this.linearSeeded = true;
    }
    if (r.angular_mode === Mode.POSITION && (!this.angularSeeded || angularDistance(live.pose.orientation, q) > this.s.max_reference_lag_angle)) {
      live.pose.orientation = q;
      live.pose.angular_velocity = qrot(q, wBody);
      live.pose.angular_acceleration = [0, 0, 0];
      this.angularSeeded = true;
    }
    this.target = r;
  }

  boxplus(p, d) {
    const x = p.x.slice();
    x[0] += d[0]; x[1] += d[1]; x[2] += d[2];
    const q = qnormalize(qmul([x[3], x[4], x[5], x[6]], qexp([d[3], d[4], d[5]])));
    x[3] = q[0]; x[4] = q[1]; x[5] = q[2]; x[6] = q[3];
    for (let i = 0; i < 6; i++) x[7 + i] += d[6 + i];
    return { x, f: p.f.map((v, i) => v + d[12 + i]) };
  }
  boxminus(a, b, out) {
    out[0] = a.x[0] - b.x[0]; out[1] = a.x[1] - b.x[1]; out[2] = a.x[2] - b.x[2];
    const r = qlog(qmul(qconj([b.x[3], b.x[4], b.x[5], b.x[6]]), [a.x[3], a.x[4], a.x[5], a.x[6]]));
    out[3] = r[0]; out[4] = r[1]; out[5] = r[2];
    for (let i = 0; i < 6; i++) out[6 + i] = a.x[7 + i] - b.x[7 + i];
    for (let i = 0; i < this.nu; i++) out[12 + i] = a.f[i] - b.f[i];
    return out;
  }
  // One control interval with the command held; `fine` is the prediction, coarse is for Jacobians.
  stage(p, cmd, fine) {
    const hReq = fine ? this.s.model_step : this.s.linearization_step;
    const steps = Math.max(1, Math.round(this.s.dt / hReq));
    const h = this.s.dt / steps;
    const target = this.model.commandToTarget(cmd, !fine);
    let x = p.x;
    const f = p.f.slice();
    for (let i = 0; i < steps; i++) x = this.model.step(x, f, target, h);
    return { x, f };
  }

  output(x, r, s) {
    const q = qnormalize([x[3], x[4], x[5], x[6]]);
    const y = new Float64Array(12);
    const pb = this.model.baseLinkPosition(x);
    y[0] = pb[0] - s.pose.position[0]; y[1] = pb[1] - s.pose.position[1]; y[2] = pb[2] - s.pose.position[2];
    const e = qlog(qmul(qconj(s.pose.orientation), q));
    y[3] = e[0]; y[4] = e[1]; y[5] = e[2];
    const v = this.model.baseLinkVelocity(x);
    let ev;
    if (r.linear_mode === Mode.VELOCITY) ev = sub(v, s.linear_velocity);
    else ev = sub(v, qrotInv(q, s.pose.velocity));
    y[6] = ev[0]; y[7] = ev[1]; y[8] = ev[2];
    const w = [x[10], x[11], x[12]];
    const ew = r.angular_mode === Mode.VELOCITY ? sub(w, s.angular_velocity) : sub(w, qrotInv(q, s.pose.angular_velocity));
    y[9] = ew[0]; y[10] = ew[1]; y[11] = ew[2];
    return y;
  }

  weights(r) {
    const w = new Float64Array(12);
    const S = this.s;
    if (r.linear_mode === Mode.POSITION) { w.set(S.q_position, 0); w.set(S.q_linear_damping, 6); }
    else if (r.linear_mode === Mode.VELOCITY) w.set(S.q_linear_velocity, 6);
    if (r.angular_mode === Mode.POSITION) { w.set(S.q_attitude, 3); w.set(S.q_angular_damping, 9); }
    else if (r.angular_mode === Mode.VELOCITY) w.set(S.q_angular_velocity, 9);
    return w;
  }

  // Effective thruster matrix at x (6 x nu, row-major arrays) and min-norm allocation.
  effectiveT(x) {
    const T = [[], [], [], [], [], []];
    for (let i = 0; i < this.nu; i++) {
      const unit = new Array(this.nu).fill(0);
      unit[i] = 1;
      const col = this.model.propulsion(x, unit);
      for (let r = 0; r < 6; r++) T[r].push(col[r]);
    }
    return T;
  }
  allocate(x, wrench) {
    const T = this.effectiveT(x);
    const nu = this.nu;
    // u = T' (T T')^-1 w  (T has full row rank: the minimum-norm solution)
    const TT = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => T[i].reduce((s, t, k) => s + t * T[j][k], 0)));
    const A = new Float64Array(36);
    TT.forEach((row, i) => row.forEach((v, j) => (A[i * 6 + j] = v)));
    const y = Float64Array.from(wrench);
    choleskySolve6(A, y);
    let u = new Array(nu).fill(0).map((_, k) => T.reduce((s, row, i) => s + row[k] * y[i], 0));
    u = u.map((c, i) => Math.min(this.ub[i], Math.max(this.lb[i], c)));
    return this.model.limitTotalThrust(u);
  }

  feedforwardInput(s, r, x0) {
    const linPose = r.linear_mode === Mode.POSITION, angPose = r.angular_mode === Mode.POSITION;
    const q = angPose ? qnormalize(s.pose.orientation) : qnormalize([x0[3], x0[4], x0[5], x0[6]]);
    const p = linPose ? s.pose.position : this.model.baseLinkPosition(x0);
    let vBody = [0, 0, 0], aWorld = [0, 0, 0];
    if (linPose) { vBody = qrotInv(q, s.pose.velocity); aWorld = s.pose.acceleration; }
    else if (r.linear_mode === Mode.VELOCITY) vBody = s.linear_velocity;
    let wBody = [0, 0, 0], wDot = [0, 0, 0];
    if (angPose) { wBody = qrotInv(q, s.pose.angular_velocity); wDot = qrotInv(q, s.pose.angular_acceleration); }
    else if (r.angular_mode === Mode.VELOCITY) wBody = s.angular_velocity;
    const xRef = this.model.fromBaseLink(p, q, vBody, wBody);
    const vDotBase = sub(qrotInv(q, aWorld), cross(wBody, vBody));
    const lin = sub(vDotBase, cross(wDot, this.model.baseOffset));
    const nuDot = [...lin, ...wDot];
    const unforced = this.model.derivative(xRef, new Array(this.nu).fill(0)).slice(7);
    const M = this.model.M;
    const tau = M.map((row) => row.reduce((acc, m, j) => acc + m * (nuDot[j] - unforced[j]), 0));
    return this.allocate(xRef, tau);
  }

  compute(measured, reference) {
    const t0 = performance.now();
    const nu = this.nu, nx = this.nx, S = this.s;
    const out = { command: new Array(nu).fill(0), active: false, prediction: [], qp_iterations: 0, converged: true };
    const finish = () => {
      out.command = this.model.limitTotalThrust(out.command);
      out.wrench = this.model.T.map((row) => row.reduce((s, t, i) => s + t * out.command[i], 0));
      out.solve_ms = performance.now() - t0;
      return out;
    };
    if (reference.linear_mode === Mode.DISABLED || reference.angular_mode === Mode.DISABLED) {
      this.clearWarmStart();
      return finish();
    }
    out.active = true;

    // Where the vehicle will be when this command starts acting: roll the in-flight commands through the delay.
    let x0 = { x: measured.slice(), f: this.actuator.force.slice() };
    if (S.compensate_delay) {
      const act = this.actuator.clone();
      const delay = this.model.act.delay;
      const steps = Math.max(1, Math.round(delay / S.model_step));
      let x = x0.x;
      for (let i = 0; i < steps; i++) {
        act.advance(delay / steps / 2);
        const f = act.force.slice();
        x = this.model.rk4(x, f, delay / steps);
        act.advance(delay / steps / 2);
      }
      x0 = { x, f: act.force.slice() };
    }
    this.seedReference(x0.x, reference);

    // Reference governor: slow the profile's cruise when the vehicle lags it.
    {
      const scaleLag = (lag, start, stop) => (stop > start ? clamp(1 - (lag - start) / (stop - start), 0.1, 1) : 1);
      const q0 = [x0.x[3], x0.x[4], x0.x[5], x0.x[6]];
      this.motion = { ...S.motion };
      const live = this.live;
      const toGo = reference.path ? reference.path.length - live.progress.s : norm(sub(reference.position, live.pose.position));
      const heading = reference.path ? reference.path.tangent(live.progress.s) : sub(reference.position, live.pose.position);
      const braking = toGo <= MotionProfile.brakingDistance(norm(live.pose.velocity), this.motion, heading) + S.governor_stop_lag;
      this.governor = 1;
      if (reference.linear_mode === Mode.POSITION && !braking) {
        const sLin = scaleLag(norm(sub(this.model.baseLinkPosition(x0.x), live.pose.position)), S.governor_lag, S.governor_stop_lag);
        this.motion.linear_speed *= sLin;
        this.motion.linear_speed_vertical *= sLin;
        this.governor = sLin;
      }
      if (reference.angular_mode === Mode.POSITION) this.motion.angular_speed *= scaleLag(angularDistance(qnormalize(q0), live.pose.orientation), S.governor_angle, S.governor_stop_angle);
    }

    const N = S.horizon, nU = N * nu;
    const stageRef = new Array(N);
    {
      const s = cloneStage(this.live);
      for (let k = 0; k < N; k++) {
        this.stepReference(s, reference, S.dt);
        stageRef[k] = cloneStage(s);
      }
    }
    // Feedforward per stage, sampled a thruster rise time after the stage midpoint.
    const Uref = new Float64Array(nU);
    {
      const s = cloneStage(this.live);
      this.stepReference(s, reference, S.dt / 2 + this.model.act.rise);
      for (let k = 0; k < N; k++) {
        Uref.set(this.feedforwardInput(s, reference, x0.x), k * nu);
        this.stepReference(s, reference, S.dt);
      }
    }
    if (S.feedforward === false) Uref.fill(0); // demo switch only; riptide_mpc always uses it
    if (!this.warm || this.warm.length !== nU) this.warm = Float64Array.from(Uref);
    else this.warm.copyWithin(0, nu);
    const U = new Float64Array(nU);
    for (let i = 0; i < nU; i++) U[i] = Math.min(this.ub[i], Math.max(this.lb[i], this.warm[i]));
    const wStage = this.weights(reference);

    const eps = 1e-6;
    let H, g, lastQp = null;
    for (let iteration = 0; iteration < S.sqp_iterations; iteration++) {
      const nominal = [x0];
      for (let k = 0; k < N; k++) nominal.push(this.stage(nominal[k], Array.from(U.subarray(k * nu, (k + 1) * nu)), true));
      H = new Float64Array(nU * nU);
      g = new Float64Array(nU);
      const P = new Float64Array(nx * nU); // d(stage state)/dU, row-major nx x nU
      const A = new Float64Array(nx * nx), B = new Float64Array(nx * nu), C = new Float64Array(12 * nx);
      const d = new Float64Array(nx), diff = new Float64Array(nx);
      const tmp = new Float64Array(nx * nU);
      const Mt = new Float64Array(12 * nU), WMt = new Float64Array(12 * nU);
      const ys = [];
      for (let k = 0; k < N; k++) {
        const u = Array.from(U.subarray(k * nu, (k + 1) * nu));
        const base = this.stage(nominal[k], u, false);
        for (let i = 0; i < nx; i++) {
          d.fill(0); d[i] = eps;
          this.boxminus(this.stage(this.boxplus(nominal[k], d), u, false), base, diff);
          for (let r = 0; r < nx; r++) A[r * nx + i] = diff[r] / eps;
        }
        for (let i = 0; i < nu; i++) {
          const du = u.slice(); du[i] += eps;
          this.boxminus(this.stage(nominal[k], du, false), base, diff);
          for (let r = 0; r < nx; r++) B[r * nu + i] = diff[r] / eps;
        }
        const cols = (k + 1) * nu;
        // P[:, :cols] = A P[:, :cols];  P[:, k*nu : k*nu+nu] += B
        for (let r = 0; r < nx; r++) {
          const tr = r * nU;
          for (let c = 0; c < cols; c++) tmp[tr + c] = 0;
          for (let j = 0; j < nx; j++) {
            const a = A[r * nx + j];
            if (a === 0) continue;
            const pj = j * nU;
            for (let c = 0; c < cols; c++) tmp[tr + c] += a * P[pj + c];
          }
        }
        for (let r = 0; r < nx; r++) {
          for (let c = 0; c < cols; c++) P[r * nU + c] = tmp[r * nU + c];
          for (let i = 0; i < nu; i++) P[r * nU + k * nu + i] += B[r * nu + i];
        }
        const next = nominal[k + 1];
        const y = this.output(next.x, reference, stageRef[k]);
        ys.push(y);
        for (let i = 0; i < 12; i++) {
          d.fill(0); d[i] = eps;
          const yp = this.output(this.boxplus(next, d).x, reference, stageRef[k]);
          for (let r = 0; r < 12; r++) C[r * nx + i] = (yp[r] - y[r]) / eps;
        }
        for (let i = 12; i < nx; i++) for (let r = 0; r < 12; r++) C[r * nx + i] = 0;
        const wf = k === N - 1 ? S.terminal_factor : 1;
        // M = C P[:, :cols], stored transposed (cols x 12) so the products below run contiguously.
        for (let c = 0; c < cols; c++)
          for (let r = 0; r < 12; r++) {
            let s = 0;
            for (let j = 0; j < 12; j++) s += C[r * nx + j] * P[j * nU + c];
            Mt[c * 12 + r] = s;
            WMt[c * 12 + r] = s * wf * wStage[r];
          }
        // H += M' W M ;  g += M' W y
        for (let a = 0; a < cols; a++) {
          const ma = a * 12;
          let ga = 0;
          for (let r = 0; r < 12; r++) ga += WMt[ma + r] * y[r];
          g[a] += ga;
          for (let b = a; b < cols; b++) {
            const mb = b * 12;
            let s = 0;
            for (let r = 0; r < 12; r++) s += WMt[ma + r] * Mt[mb + r];
            H[a * nU + b] += s;
            if (b !== a) H[b * nU + a] += s;
          }
        }
        if (this.debug && k === 0) this.firstJacobian = { A: Float64Array.from(A), B: Float64Array.from(B) };
      }
      // Effort and smoothness act on the deviation from the feedforward.
      for (let i = 0; i < nU; i++) {
        H[i * nU + i] += S.r_thrust;
        g[i] += S.r_thrust * (U[i] - Uref[i]);
      }
      const D = new Float64Array(nU);
      for (let i = 0; i < nU; i++) D[i] = U[i] - Uref[i];
      const rr = S.r_thrust_rate;
      for (let k = 0; k < N; k++)
        for (let i = 0; i < nu; i++) {
          const prev = k === 0 ? this.lastDeviation[i] : D[(k - 1) * nu + i];
          const rate = D[k * nu + i] - prev;
          const a = k * nu + i;
          H[a * nU + a] += rr;
          g[a] += rr * rate;
          if (k > 0) {
            const b = (k - 1) * nu + i;
            H[b * nU + b] += rr;
            H[a * nU + b] -= rr;
            H[b * nU + a] -= rr;
            g[b] -= rr * rate;
          }
        }
      const delta = new Float64Array(nU);
      const lo = new Float64Array(nU), hi = new Float64Array(nU);
      for (let i = 0; i < nU; i++) { lo[i] = this.lb[i] - U[i]; hi[i] = this.ub[i] - U[i]; }
      const trace = this.debug ? [] : null;
      const qp = solveBoxQp(H, g, lo, hi, delta, nU, S.qp_max_iterations, 1e-8, trace);
      lastQp = { ...qp, trace };
      out.qp_iterations += qp.iterations;
      out.converged = out.converged && qp.converged;
      for (let i = 0; i < nU; i++) U[i] = Math.min(this.ub[i], Math.max(this.lb[i], U[i] + delta[i]));
      if (this.debug) this.lastCosts = ys;
    }
    this.warm = Float64Array.from(U);
    out.command = Array.from(U.subarray(0, nu));
    this.issuedFeedforward = Array.from(Uref.subarray(0, nu));
    let p = x0;
    out.prediction.push(p.x);
    for (let k = 0; k < N; k++) {
      p = this.stage(p, Array.from(U.subarray(k * nu, (k + 1) * nu)), true);
      out.prediction.push(p.x);
    }
    out.U = U;
    out.Uref = Uref;
    out.stageRef = stageRef.map((s) => ({ position: s.pose.position.slice(), orientation: s.pose.orientation.slice() }));
    if (this.debug) { out.H = H; out.g = g; out.qp = lastQp; }
    return finish();
  }
}

function choleskySolve6(A, y) {
  const n = 6;
  for (let j = 0; j < n; j++) {
    let s = A[j * n + j];
    for (let k = 0; k < j; k++) s -= A[j * n + k] * A[j * n + k];
    const d = Math.sqrt(Math.max(s, 1e-12));
    A[j * n + j] = d;
    for (let i = j + 1; i < n; i++) {
      let t = A[i * n + j];
      for (let k = 0; k < j; k++) t -= A[i * n + k] * A[j * n + k];
      A[i * n + j] = t / d;
    }
  }
  for (let i = 0; i < n; i++) {
    let t = y[i];
    for (let k = 0; k < i; k++) t -= A[i * n + k] * y[k];
    y[i] = t / A[i * n + i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let t = y[i];
    for (let k = i + 1; k < n; k++) t -= A[k * n + i] * y[k];
    y[i] = t / A[i * n + i];
  }
}

