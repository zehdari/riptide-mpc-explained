// Port of riptide_mpc's MarineDynamics + FossenModel + ThrusterDynamics.
// State x (length 13): [COM position (world), q wxyz (body->world), body linear
// velocity at COM (u v w), body angular velocity (p q r)].
import { vehicle, hydro } from './talos.js';
import { cross, qrot, qrotInv, qmul, rpyQuat, invert6, clamp, sub } from './math.js';

const G = 9.80665;

export class FossenModel {
  constructor(h = hydro, v = vehicle) {
    this.h = h;
    this.mass = h.mass;
    this.com = h.com;
    const I = h.inertia;
    const m = h.mass;
    this.MRB = [
      [m, 0, 0, 0, 0, 0],
      [0, m, 0, 0, 0, 0],
      [0, 0, m, 0, 0, 0],
      [0, 0, 0, I[0][0], I[0][1], I[0][2]],
      [0, 0, 0, I[1][0], I[1][1], I[1][2]],
      [0, 0, 0, I[2][0], I[2][1], I[2][2]],
    ];
    this.MA = this.MRB.map((row, i) => row.map((_, j) => (i === j ? h.added_mass[i] : 0)));
    this.M = this.MRB.map((row, i) => row.map((x, j) => x + this.MA[i][j]));
    this.Minv = invert6(this.M);
    this.Dl = h.linear_damping;
    this.Dq = h.quadratic_damping;
    this.baseOffset = sub(v.base_link, h.com); // base_link relative to COM, body axes
    this.positions = [];
    this.directions = [];
    this.T = [[], [], [], [], [], []]; // 6 x N, wrench at COM per newton
    for (const t of v.thrusters) {
      const p = sub(t.pose.slice(0, 3), h.com);
      const d = qrot(rpyQuat(t.pose[3], t.pose[4], t.pose[5]), [1, 0, 0]);
      this.positions.push(p);
      this.directions.push(d);
      const mo = cross(p, d);
      [...d, ...mo].forEach((val, r) => this.T[r].push(val));
    }
    this.n = v.thrusters.length;
    this.act = { ...h.thruster };
    this.disturbance = [0, 0, 0, 0, 0, 0]; // [force world, torque body]
  }

  // Coriolis-centripetal C(M, v) v in momentum form (Fossen / MSS m2c), applied to v.
  static coriolisTimes(M, v) {
    const h = M.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2] + row[3] * v[3] + row[4] * v[4] + row[5] * v[5]);
    const hl = [h[0], h[1], h[2]], ha = [h[3], h[4], h[5]];
    const vl = [v[0], v[1], v[2]], w = [v[3], v[4], v[5]];
    // C = [[0, -S(hl)], [-S(hl), -S(ha)]];  -S(a) b = b x a
    const lin = cross(w, hl);
    const a1 = cross(vl, hl), a2 = cross(w, ha);
    return [lin[0], lin[1], lin[2], a1[0] + a2[0], a1[1] + a2[1], a1[2] + a2[2]];
  }

  damping(v) {
    const out = new Array(6);
    for (let i = 0; i < 6; i++) out[i] = -(this.Dl[i] * v[i] + this.Dq[i] * Math.abs(v[i]) * v[i]);
    return out;
  }

  // Fraction of the buoyancy ellipsoid under water, and the wet centre (body).
  submerged(p, q) {
    const h = this.h;
    const vert = qrotInv(q, [0, 0, 1]);
    const r = h.buoyancy_radii;
    const height = Math.hypot(r[0] * vert[0], r[1] * vert[1], r[2] * vert[2]);
    const cobW = qrot(q, h.cob);
    const c = clamp((h.water_level - (p[2] + cobW[2])) / height, -1, 1);
    const fraction = (2 + 3 * c - c * c * c) / 4;
    let center = h.cob.slice();
    if (fraction > 1e-10 && fraction < 1) {
      const mean = (-3 * Math.pow(1 - c * c, 2)) / (16 * fraction);
      center = center.map((x, i) => x + r[i] * r[i] * vert[i] * (mean / height));
    }
    return { fraction, center };
  }

  restoring(p, q) {
    const { fraction, center } = this.submerged(p, q);
    const B = this.h.water_density * G * this.h.displaced_volume * fraction;
    const force = qrotInv(q, [0, 0, B]);
    const net = qrotInv(q, [0, 0, B - this.mass * G]);
    const tq = cross(center, force);
    return { wrench: [...net, ...tq], fraction };
  }

  // Thrust wrench at the COM, with the simulator's partial-submergence disk factor.
  propulsion(x, force) {
    const q = [x[3], x[4], x[5], x[6]];
    const out = [0, 0, 0, 0, 0, 0];
    // Every thruster is within ~0.6 m of the COM: deeper than that, all disks are fully wet.
    const deep = x[2] < this.h.water_level - 0.7;
    for (let i = 0; i < this.n; i++) {
      let f = force[i];
      if (f === 0) continue;
      if (deep) {
        for (let r = 0; r < 6; r++) out[r] += this.T[r][i] * f;
        continue;
      }
      const z = x[2] + qrot(q, this.positions[i])[2];
      const az = qrot(q, this.directions[i])[2];
      const extent = Math.max(0.001, this.act.propeller_radius * Math.sqrt(Math.max(0, 1 - az * az)));
      const c = clamp((this.h.water_level - z) / extent, -1, 1);
      if (c < 1) f *= (Math.acos(-c) + c * Math.sqrt(Math.max(0, 1 - c * c))) / Math.PI;
      for (let r = 0; r < 6; r++) out[r] += this.T[r][i] * f;
    }
    return out;
  }

  // dx/dt for thrust held at `force`; `parts` (optional) collects the terms for plots.
  derivative(x, force, parts) {
    const q = [x[3], x[4], x[5], x[6]];
    const v = [x[7], x[8], x[9], x[10], x[11], x[12]];
    const prop = this.propulsion(x, force);
    const d = this.disturbance;
    if (d[0] || d[1] || d[2] || d[3] || d[4] || d[5]) {
      const fb = qrotInv(q, [d[0], d[1], d[2]]);
      prop[0] += fb[0]; prop[1] += fb[1]; prop[2] += fb[2];
      prop[3] += d[3]; prop[4] += d[4]; prop[5] += d[5];
    }
    const { wrench: g, fraction } = this.restoring([x[0], x[1], x[2]], q);
    const D = this.damping(v);
    const cRB = FossenModel.coriolisTimes(this.MRB, v);
    const cA = FossenModel.coriolisTimes(this.MA, v);
    const rhs = new Array(6);
    for (let i = 0; i < 6; i++) rhs[i] = prop[i] + g[i] + fraction * D[i] - cRB[i] - cA[i];
    if (parts) Object.assign(parts, { prop, g, D, cRB, cA });
    const Mi = this.Minv;
    const dx = new Array(13);
    const pd = qrot(q, [v[0], v[1], v[2]]);
    dx[0] = pd[0]; dx[1] = pd[1]; dx[2] = pd[2];
    const dq = qmul(q, [0, v[3], v[4], v[5]]);
    dx[3] = dq[0] / 2; dx[4] = dq[1] / 2; dx[5] = dq[2] / 2; dx[6] = dq[3] / 2;
    for (let i = 0; i < 6; i++) {
      const r = Mi[i];
      dx[7 + i] = r[0] * rhs[0] + r[1] * rhs[1] + r[2] * rhs[2] + r[3] * rhs[3] + r[4] * rhs[4] + r[5] * rhs[5];
    }
    return dx;
  }

  rk4(x, force, h) {
    const k1 = this.derivative(x, force);
    const x2 = x.map((v, i) => v + (h / 2) * k1[i]);
    const k2 = this.derivative(x2, force);
    const x3 = x.map((v, i) => v + (h / 2) * k2[i]);
    const k3 = this.derivative(x3, force);
    const x4 = x.map((v, i) => v + h * k3[i]);
    const k4 = this.derivative(x4, force);
    const out = x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    const n = Math.hypot(out[3], out[4], out[5], out[6]);
    out[3] /= n; out[4] /= n; out[5] /= n; out[6] /= n;
    return out;
  }

  // ThrusterDynamics::command target. `smooth` drops the clamp (for Jacobians).
  commandToTarget(cmd, smooth = false) {
    const a = this.act;
    return cmd.map((c) => (smooth ? c : clamp(c, -a.reverse_max, a.forward_max)));
  }

  // First-order lag toward target with separate rise / fall constants and a slew cap.
  evolve(force, target, dt) {
    const a = this.act;
    for (let i = 0; i < force.length; i++) {
      const rising = target[i] * force[i] >= 0 && Math.abs(target[i]) > Math.abs(force[i]);
      const tau = rising ? a.rise : a.fall;
      let delta = (target[i] - force[i]) * -Math.expm1(-dt / tau);
      if (a.slew > 0) delta = clamp(delta, -a.slew * dt, a.slew * dt);
      force[i] += delta;
    }
  }

  // One physics_simulator step: actuators h/2, RK4 body with thrust held, actuators h/2.
  step(x, force, target, h) {
    this.evolve(force, target, h / 2);
    const nx = this.rk4(x, force, h);
    this.evolve(force, target, h / 2);
    return nx;
  }

  limitTotalThrust(cmd) {
    const total = cmd.reduce((s, c) => s + Math.abs(c), 0);
    const L = this.h.total_thrust_limit;
    return L > 0 && total > L ? cmd.map((c) => (c * L) / total) : cmd;
  }

  baseLinkPosition(x) {
    const p = qrot([x[3], x[4], x[5], x[6]], this.baseOffset);
    return [x[0] + p[0], x[1] + p[1], x[2] + p[2]];
  }
  baseLinkVelocity(x) {
    const w = [x[10], x[11], x[12]];
    const c = cross(w, this.baseOffset);
    return [x[7] + c[0], x[8] + c[1], x[9] + c[2]];
  }
  fromBaseLink(p, q, vBody, w) {
    const o = qrot(q, this.baseOffset);
    const c = cross(w, this.baseOffset);
    return [p[0] - o[0], p[1] - o[1], p[2] - o[2], q[0], q[1], q[2], q[3], vBody[0] - c[0], vBody[1] - c[1], vBody[2] - c[2], w[0], w[1], w[2]];
  }

  rpm(force) {
    if (force === 0) return 0;
    const [a, b] = force > 0 ? this.h.kt_forward : this.h.kt_reverse;
    const ks = (this.h.water_density * Math.pow(this.h.propeller_diameter, 4)) / 3600;
    const target = Math.abs(force) / ks;
    let r = Math.sqrt(target / a);
    for (let i = 0; i < 50; i++) {
      const step = ((a + b * r) * r * r - target) / ((2 * a + 3 * b * r) * r);
      r -= step;
      if (Math.abs(step) <= 1e-12 * r) break;
    }
    return Math.sign(force) * r;
  }
}

// The thrusters' delay queue + lag, kept in lockstep with the plant (MpcController::actuator_).
export class ThrusterReplica {
  constructor(model) {
    this.model = model;
    this.n = model.n;
    this.time = 0;
    this.queue = [];
    this.target = new Array(this.n).fill(0);
    this.force = new Array(this.n).fill(0);
  }
  clone() {
    const c = new ThrusterReplica(this.model);
    c.time = this.time;
    c.queue = this.queue.map((e) => ({ due: e.due, f: e.f.slice() }));
    c.target = this.target.slice();
    c.force = this.force.slice();
    return c;
  }
  command(cmd) {
    const f = this.model.commandToTarget(cmd);
    const due = this.time + this.model.act.delay;
    const last = this.queue[this.queue.length - 1];
    if (last && last.due === due) last.f = f;
    else this.queue.push({ due, f });
  }
  advance(dt) {
    const end = this.time + dt;
    let cursor = this.time;
    while (this.queue.length && this.queue[0].due <= end) {
      const e = this.queue.shift();
      const span = Math.max(0, e.due - cursor);
      if (span > 0) this.model.evolve(this.force, this.target, span);
      cursor = Math.max(cursor, e.due);
      this.target = e.f.slice();
    }
    if (end - cursor > 0) this.model.evolve(this.force, this.target, end - cursor);
    this.time = end;
  }
  stop() {
    this.queue = [];
    this.target.fill(0);
  }
}

// Plant: the same model integrated at 2 ms, with the thruster replica's realized forces.
export function plantStep(model, x, actuator, h) {
  actuator.advance(h / 2);
  const f = actuator.force.slice();
  const nx = model.rk4(x, f, h);
  actuator.advance(h / 2);
  return nx;
}
