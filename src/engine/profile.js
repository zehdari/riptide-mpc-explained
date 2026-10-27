// Port of riptide_mpc motion_profile.cpp: jerk-limited (S-curve) reference that
// travels to a setpoint and stops on it with zero velocity and acceleration.
import { add, sub, scale, dot, norm, cross, clamp, qexp, qlog, qmul, qconj, qnormalize } from './math.js';

export const kSubstep = 0.002;
export const kCapturePole = 10.0;

// Constant-jerk motion for time t. Mutates s = {v, a}; returns distance covered.
export function advance(s, j, t) {
  const d = s.v * t + (s.a * t * t) / 2 + (j * t * t * t) / 6;
  s.v += s.a * t + (j * t * t) / 2;
  s.a += j * t;
  return d;
}

// Distance to come to rest from (v >= 0, a), braking as hard as allowed.
export function stoppingDistance(v, a, l) {
  const s = { v, a };
  let d = 0;
  if (s.a > 0) {
    d += advance(s, -l.jerk, s.a / l.jerk);
    s.a = 0;
  }
  if (s.v <= 0) return d;
  const a0 = -s.a;
  let peak = Math.sqrt((2 * l.jerk * s.v + a0 * a0) / 2);
  if (peak < a0) {
    const t = (a0 - Math.sqrt(Math.max(0, a0 * a0 - 2 * l.jerk * s.v))) / l.jerk;
    return d + advance(s, l.jerk, t);
  }
  peak = Math.min(peak, l.accel);
  d += advance(s, -l.jerk, (peak - a0) / l.jerk);
  const hold = (s.v - (peak * peak) / (2 * l.jerk)) / peak;
  if (hold > 0) d += advance(s, 0, hold);
  return d + advance(s, l.jerk, peak / l.jerk);
}

const clampJerk = (j, a, l, h) => clamp(j, (-l.accel - a) / h, (l.accel - a) / h);

export function velocityJerk(v, a, target, l, h) {
  const vEnd = v + (a * Math.abs(a)) / (2 * l.jerk);
  let j;
  if (Math.abs(vEnd - target) < l.jerk * h * h) j = clamp(-a / h, -l.jerk, l.jerk);
  else j = vEnd < target ? l.jerk : -l.jerk;
  return clampJerk(j, a, l, h);
}

export function brakingJerk(v, a, l, h) {
  if (a > 0) return clampJerk(-l.jerk, a, l, h);
  const a0 = -a;
  let peak = Math.sqrt((2 * l.jerk * v + a0 * a0) / 2);
  if (peak < a0) return l.jerk;
  peak = Math.min(peak, l.accel);
  if (v <= (peak * peak) / (2 * l.jerk) + 1e-12) return l.jerk;
  if (a0 < peak) return clampJerk(-Math.min(l.jerk, (peak - a0) / h), a, l, h);
  return 0;
}

export function linearLimitsAlong(m, dir) {
  const n = norm(dir);
  const z = n > 1e-12 ? Math.min(1, Math.abs(dir[2]) / n) : 0;
  const h = Math.sqrt(1 - z * z);
  const blend = (hor, ver) => (ver > 0 ? 1 / Math.hypot(h / hor, z / ver) : hor);
  return {
    speed: blend(m.linear_speed, m.linear_speed_vertical),
    accel: blend(m.linear_accel, m.linear_accel_vertical),
    jerk: blend(m.linear_jerk, m.linear_jerk_vertical),
  };
}

function unitOrthogonal(e) {
  const t = Math.abs(e[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const c = cross(e, t);
  return scale(c, 1 / norm(c));
}

// One substep toward a point `offset` away. Mutates st = {v, a} (vectors); returns displacement.
function substep(offset, st, limitsAlong, c, h, extra = 0) {
  const distance = norm(offset);
  const vn = norm(st.v);
  const stray = distance < (c.stray_distance || 0) && vn < (c.stray_speed || 0) && dot(offset, st.v) < 0.5 * distance * vn;
  if ((distance < c.distance && vn < c.speed && norm(st.a) < c.accel) || stray) {
    const p = kCapturePole;
    let j = sub(sub(scale(offset, p * p * p), scale(st.v, 3 * p * p)), scale(st.a, 3 * p));
    const jl = limitsAlong(j).jerk;
    if (norm(j) > jl) j = scale(j, jl / norm(j));
    const d = add(add(scale(st.v, h), scale(st.a, (h * h) / 2)), scale(j, (h * h * h) / 6));
    st.v = add(st.v, add(scale(st.a, h), scale(j, (h * h) / 2)));
    st.a = add(st.a, scale(j, h));
    return d;
  }
  const travelling = distance > 1e-9;
  const e0 = travelling ? scale(offset, 1 / distance) : vn > 1e-12 ? scale(st.v, 1 / vn) : norm(st.a) > 1e-12 ? scale(st.a, 1 / norm(st.a)) : [1, 0, 0];
  let side = sub(st.v, scale(e0, dot(e0, st.v)));
  if (norm(side) < 1e-12) side = sub(st.a, scale(e0, dot(e0, st.a)));
  const e1 = norm(side) > 1e-12 ? scale(side, 1 / norm(side)) : unitOrthogonal(e0);
  const e2 = cross(e0, e1);
  const E = [e0, e1, e2];
  let disp = [0, 0, 0], vNew = [0, 0, 0], aNew = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    const l = limitsAlong(E[i]);
    const s = { v: dot(E[i], st.v), a: dot(E[i], st.a) };
    let j, braking = false;
    if (i === 0 && travelling) {
      j = velocityJerk(s.v, s.a, l.speed, l, h);
      const sc = { ...s };
      const dc = advance(sc, j, h);
      if (sc.v > 0 && dc + stoppingDistance(sc.v, clamp(sc.a, -l.accel, l.accel), l) > distance + extra) {
        j = brakingJerk(s.v, s.a, l, h);
        braking = true;
      }
    } else {
      j = velocityJerk(s.v, s.a, 0, l, h);
    }
    let d = advance(s, j, h);
    if (braking && s.v < 0) {
      s.v = s.a = 0;
      d = Math.max(d, 0);
    }
    disp = add(disp, scale(E[i], d));
    vNew = add(vNew, scale(E[i], s.v));
    aNew = add(aNew, scale(E[i], s.a));
  }
  st.v = vNew;
  st.a = aNew;
  return disp;
}

const atRest = (distance, v, a) => distance < 2e-4 && norm(v) < 2e-3 && norm(a) < 2e-2;

export class MotionProfile {
  constructor() {
    this.position = [0, 0, 0];
    this.velocity = [0, 0, 0];
    this.acceleration = [0, 0, 0];
    this.orientation = [1, 0, 0, 0];
    this.angular_velocity = [0, 0, 0];
    this.angular_acceleration = [0, 0, 0];
  }
  clone() {
    const c = new MotionProfile();
    c.position = this.position.slice();
    c.velocity = this.velocity.slice();
    c.acceleration = this.acceleration.slice();
    c.orientation = this.orientation.slice();
    c.angular_velocity = this.angular_velocity.slice();
    c.angular_acceleration = this.angular_acceleration.slice();
    return c;
  }
  stepLinear(target, m, dt, extra = 0) {
    const l = (dir) => linearLimitsAlong(m, dir);
    const steps = Math.ceil(dt / kSubstep - 1e-9);
    for (let i = 0; i < steps; i++) {
      if (extra === 0 && atRest(norm(sub(target, this.position)), this.velocity, this.acceleration)) {
        this.position = target.slice();
        this.velocity = [0, 0, 0];
        this.acceleration = [0, 0, 0];
        return;
      }
      const st = { v: this.velocity, a: this.acceleration };
      const d = substep(sub(target, this.position), st, l, { distance: 2e-3, speed: 2e-2, accel: 1e-1, stray_distance: 2e-2, stray_speed: 1e-1 }, dt / steps, extra);
      this.position = add(this.position, d);
      this.velocity = st.v;
      this.acceleration = st.a;
    }
  }
  stepAngular(targetIn, m, dt) {
    const limits = { speed: m.angular_speed, accel: m.angular_accel, jerk: m.angular_jerk };
    const l = () => limits;
    const target = qnormalize(targetIn);
    const steps = Math.ceil(dt / kSubstep - 1e-9);
    for (let i = 0; i < steps; i++) {
      const offset = qlog(qmul(target, qconj(this.orientation)));
      if (atRest(norm(offset), this.angular_velocity, this.angular_acceleration)) {
        this.orientation = target.slice();
        this.angular_velocity = [0, 0, 0];
        this.angular_acceleration = [0, 0, 0];
        return;
      }
      const st = { v: this.angular_velocity, a: this.angular_acceleration };
      const d = substep(offset, st, l, { distance: 5e-3, speed: 3e-2, accel: 1.5e-1 }, dt / steps);
      this.orientation = qnormalize(qmul(qexp(d), this.orientation));
      this.angular_velocity = st.v;
      this.angular_acceleration = st.a;
    }
  }
  static brakingDistance(speed, m, dir = [1, 0, 0]) {
    return stoppingDistance(speed, 0, linearLimitsAlong(m, dir));
  }
  static rateLimit(current, target, accel, dt) {
    const change = sub(target, current);
    const limit = accel * dt;
    return norm(change) > limit ? add(current, scale(change, limit / norm(change))) : target.slice();
  }
}

// 1-D S-curve for the profile chart: rest-to-rest move of `distance` under limits l.
export function profile1d(distance, l, dt = 0.01, tMax = 60) {
  const out = [];
  const st = { v: 0, a: 0 };
  let x = 0, t = 0;
  const h = kSubstep;
  const every = Math.max(1, Math.round(dt / h));
  for (let k = 0; t < tMax; k++) {
    const remaining = distance - x;
    if (Math.abs(remaining) < 2e-4 && Math.abs(st.v) < 2e-3 && Math.abs(st.a) < 2e-2) {
      out.push({ t, x: distance, v: 0, a: 0, j: 0 });
      break;
    }
    let j = velocityJerk(st.v, st.a, l.speed, l, h);
    const sc = { ...st };
    const dc = advance(sc, j, h);
    if (sc.v > 0 && dc + stoppingDistance(sc.v, clamp(sc.a, -l.accel, l.accel), l) > remaining) j = brakingJerk(st.v, st.a, l, h);
    if (remaining < 2e-3 && Math.abs(st.v) < 2e-2) {
      const c = kCapturePole;
      j = clamp(c * c * c * remaining - 3 * c * c * st.v - 3 * c * st.a, -l.jerk, l.jerk);
    }
    if (k % every === 0) out.push({ t, x, v: st.v, a: st.a, j });
    x += advance(st, j, h);
    if (st.v < 0 && remaining > 0) st.v = Math.max(st.v, 0);
    t += h;
  }
  return out;
}
