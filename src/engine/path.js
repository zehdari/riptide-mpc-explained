// Port of riptide_mpc path_plan.cpp (yaw-only attitude: roll/pitch blending,
// spins, moving look targets and frame shifts are left out of this port).
// A path is one curve parameterized by distance s: lines, arcs, quadratic
// corner blends and turns in place, with a jerk-limited speed profile along s.
import { add, sub, scale, dot, norm, clamp, wrap, yawQuat, twist } from './math.js';
import { advance, stoppingDistance, velocityJerk, brakingJerk, linearLimitsAlong, kSubstep, kCapturePole } from './profile.js';

const kInf = Infinity;
const kTable = 256;
const kSampleStep = 0.02;
const kYawStep = 0.01;
const kMinBlendAngle = 0.035, kMaxBlendAngle = 2.6;

function catmullRom(y, u) {
  const n = y.length - 1;
  if (n < 1) return y.length ? y[0] : 0;
  const x = clamp(u, 0, 1) * n;
  const i = Math.min(Math.floor(x), n - 1);
  const f = x - i;
  const p0 = y[Math.max(i - 1, 0)], p1 = y[i], p2 = y[i + 1], p3 = y[Math.min(i + 2, n)];
  return p1 + 0.5 * f * (p2 - p0 + f * (2 * p0 - 5 * p1 + 4 * p2 - p3 + f * (3 * (p1 - p2) + p3 - p0)));
}

class Piece {
  constructor(kind) {
    this.kind = kind; // LINE ARC BEZIER TURN
    this.a = [0, 0, 0]; this.b = [0, 0, 0]; this.c = [0, 0, 0];
    this.cx = 0; this.cy = 0; this.phi0 = 0; this.sweep = 0; this.r0 = 0; this.r1 = 0; this.z0 = 0; this.z1 = 0;
    this.t0 = 0; this.t1 = 1; this.s0 = 0; this.length = 0; this.segment = 0; this.table = null;
  }
  derivatives(t) {
    switch (this.kind) {
      case 'LINE':
        return { p: add(this.a, scale(sub(this.b, this.a), t)), d1: sub(this.b, this.a), d2: [0, 0, 0] };
      case 'ARC': {
        const phi = this.phi0 + this.sweep * t, r = this.r0 + (this.r1 - this.r0) * t, dr = this.r1 - this.r0;
        const cs = Math.cos(phi), sn = Math.sin(phi), w = this.sweep;
        return {
          p: [this.cx + r * cs, this.cy + r * sn, this.z0 + (this.z1 - this.z0) * t],
          d1: [dr * cs - r * w * sn, dr * sn + r * w * cs, this.z1 - this.z0],
          d2: [-2 * dr * w * sn - r * w * w * cs, 2 * dr * w * cs - r * w * w * sn, 0],
        };
      }
      case 'BEZIER':
        return {
          p: add(add(scale(this.a, (1 - t) * (1 - t)), scale(this.b, 2 * t * (1 - t))), scale(this.c, t * t)),
          d1: add(scale(sub(this.b, this.a), 2 * (1 - t)), scale(sub(this.c, this.b), 2 * t)),
          d2: scale(add(sub(this.a, scale(this.b, 2)), this.c), 2),
        };
      default:
        return { p: add(this.a, scale(sub(this.b, this.a), t)), d1: [0, 0, 0], d2: [0, 0, 0] };
    }
  }
  at(t) { return this.derivatives(t).p; }
  finalize() {
    this.table = null;
    if (this.kind === 'TURN') return;
    const uniform = this.kind === 'LINE' || (this.kind === 'ARC' && this.r0 === this.r1);
    if (uniform) {
      this.length = norm(this.derivatives(this.t0).d1) * (this.t1 - this.t0);
      return;
    }
    const table = new Float64Array(kTable + 1);
    const dt = (this.t1 - this.t0) / kTable;
    let prev = norm(this.derivatives(this.t0).d1);
    for (let k = 1; k <= kTable; k++) {
      const mid = norm(this.derivatives(this.t0 + (k - 0.5) * dt).d1);
      const next = norm(this.derivatives(this.t0 + k * dt).d1);
      table[k] = table[k - 1] + (dt * (prev + 4 * mid + next)) / 6;
      prev = next;
    }
    this.table = table;
    this.length = table[kTable];
  }
  tAt(local) {
    if (this.length <= 0) return this.t0;
    local = clamp(local, 0, this.length);
    if (!this.table) return this.t0 + ((this.t1 - this.t0) * local) / this.length;
    const tb = this.table;
    let lo = 0, hi = kTable;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (tb[mid] <= local) lo = mid; else hi = mid;
    }
    const span = tb[lo + 1] - tb[lo];
    const f = span > 0 ? (local - tb[lo]) / span : 0;
    return this.t0 + ((this.t1 - this.t0) * (lo + f)) / kTable;
  }
}

export class PathPlan {
  // start {position, yaw}; points [{position, yaw, shape: 'LINE'|'ARC', center, sweep, heading: 'WAYPOINT'|'PATH'|'LOOK_AT', look_at, yaw_offset}]
  static build(start, points, m, o) {
    if (!points.length) throw new Error('path has no points');
    const plan = new PathPlan();
    const rho = m.linear_speed / m.angular_speed;
    plan.rho = rho;
    const raw = [];
    let from = start.position.slice();
    points.forEach((q, i) => {
      let p;
      const c = q.center || [0, 0, 0];
      const r0 = Math.hypot(from[0] - c[0], from[1] - c[1]), r1 = Math.hypot(q.position[0] - c[0], q.position[1] - c[1]);
      if (q.shape === 'ARC' && Math.abs(q.sweep) > 1e-9 && Math.max(r0, r1) > 1e-6) {
        p = new Piece('ARC');
        Object.assign(p, { cx: c[0], cy: c[1], sweep: q.sweep, r0, r1, z0: from[2], z1: q.position[2] });
        p.phi0 = r0 > 1e-6 ? Math.atan2(from[1] - c[1], from[0] - c[0]) : Math.atan2(q.position[1] - c[1], q.position[0] - c[0]) - q.sweep;
      } else if (norm(sub(q.position, from)) > Math.max(o.turn_drift, 1e-6)) {
        p = new Piece('LINE');
        p.a = from; p.b = q.position.slice();
      } else {
        p = new Piece('TURN');
        p.a = from; p.b = q.position.slice();
      }
      p.segment = i;
      p.finalize();
      raw.push(p);
      from = p.at(1);
    });

    // Round off sharp joins with a quadratic blend from d before the corner to d after it.
    const original = raw.map((p) => p.length);
    const blends = raw.map(() => null);
    for (let i = 0; i + 1 < raw.length; i++) {
      const A = raw[i], B = raw[i + 1];
      if (A.kind === 'TURN' || B.kind === 'TURN') continue;
      const ta = A.derivatives(A.t1).d1, tb = B.derivatives(B.t0).d1;
      const angle = Math.acos(clamp(dot(ta, tb) / (norm(ta) * norm(tb)), -1, 1));
      const d = Math.min(o.corner_radius, 0.45 * original[i], 0.45 * original[i + 1]);
      if (angle < kMinBlendAngle || angle > kMaxBlendAngle || d < 0.01) continue;
      const taTrim = A.tAt(A.length - d), tbTrim = B.tAt(d);
      const P0 = A.at(taTrim), J = A.at(A.t1), P2 = B.at(tbTrim);
      const mid = scale(add(add(P0, scale(J, 2)), P2), 0.25);
      A.t1 = taTrim; A.finalize();
      B.t0 = tbTrim; B.finalize();
      const h1 = new Piece('BEZIER'), h2 = new Piece('BEZIER');
      h1.a = P0; h1.b = scale(add(P0, J), 0.5); h1.c = mid; h1.segment = i;
      h2.a = mid; h2.b = scale(add(J, P2), 0.5); h2.c = P2; h2.segment = i + 1;
      h1.finalize(); h2.finalize();
      blends[i] = [h1, h2];
      plan.corners = plan.corners || [];
      plan.corners.push({ corner: J, from: P0, to: P2 });
    }
    plan.pieces = [];
    raw.forEach((p, i) => {
      if (i > 0 && blends[i - 1]) plan.pieces.push(blends[i - 1][1]);
      plan.pieces.push(p);
      if (blends[i]) plan.pieces.push(blends[i][0]);
    });

    // Attitude per segment.
    let s = 0, yawBefore = start.yaw;
    let k = 0;
    plan.segments = [];
    points.forEach((q, i) => {
      const g = { first: k, linear: true, start_yaw: yawBefore, turn: 0, mode_yaw: null, delta: 0, blend: 1 };
      while (k < plan.pieces.length && plan.pieces[k].segment === i) k++;
      g.last = k;
      const inPlace = g.last - g.first === 1 && plan.pieces[g.first].kind === 'TURN';
      const here = plan.pieces[g.first].kind === 'TURN' ? plan.pieces[g.first].b : [0, 0, 0];
      const heading = q.heading || 'WAYPOINT';
      const yawOffset = q.yaw_offset || 0;
      if (heading === 'WAYPOINT') g.turn = wrap((q.yaw ?? 0) - yawBefore);
      else if (inPlace) {
        const to = [q.look_at[0] - here[0], q.look_at[1] - here[1]];
        g.turn = heading === 'LOOK_AT' && Math.hypot(...to) > 0.05 ? wrap(Math.atan2(to[1], to[0]) + yawOffset - yawBefore) : 0;
      } else g.linear = false;
      g.s0 = s;
      if (inPlace) {
        const p = plan.pieces[g.first];
        p.length = Math.max(rho * Math.abs(g.turn), norm(sub(p.b, p.a)));
        p.s0 = s;
        s += p.length;
      } else {
        for (let j = g.first; j < g.last; j++) {
          plan.pieces[j].s0 = s;
          s += plan.pieces[j].length;
        }
      }
      g.s1 = s;
      if (!g.linear) {
        const len = g.s1 - g.s0;
        const n = Math.max(8, Math.ceil(len / kYawStep));
        g.mode_yaw = new Array(n + 1);
        let held = yawBefore - yawOffset;
        for (let j = 0; j <= n; j++) {
          const geo = plan.geometry(g.s0 + (len * j) / n, g.first, g.last);
          const to = heading === 'PATH' ? [geo.t[0], geo.t[1]] : [q.look_at[0] - geo.p[0], q.look_at[1] - geo.p[1]];
          const valid = heading === 'PATH' ? Math.hypot(...to) > 0.3 : Math.hypot(...to) > 0.05;
          const rawYaw = valid ? Math.atan2(to[1], to[0]) : held;
          held = rawYaw;
          const y = rawYaw + yawOffset;
          g.mode_yaw[j] = j === 0 ? y : g.mode_yaw[j - 1] + wrap(y - g.mode_yaw[j - 1]);
        }
        const shift = yawBefore + wrap(g.mode_yaw[0] - yawBefore) - g.mode_yaw[0];
        for (let j = 0; j <= n; j++) g.mode_yaw[j] += shift;
        g.delta = yawBefore - g.mode_yaw[0];
        g.blend = clamp(1.5 * Math.abs(g.delta) * rho, 0.02, Math.max(len, 0.02));
      }
      plan.segments.push(g);
      yawBefore = plan.yawOf(g, g.s1);
    });
    plan.length = s;
    if (s <= 0) throw new Error('path does not move or turn');

    // Yaw grid averaged over heading_blend: the turn rate per metre never steps.
    {
      const n = Math.max(1, Math.ceil(s / kYawStep));
      const h = s / n;
      const half = Math.min(n, Math.round((0.5 * Math.max(o.heading_blend, 0)) / h));
      const rawYaw = new Float64Array(n + 1);
      let i = 0;
      for (let kk = 0; kk <= n; kk++) {
        const at = Math.min(kk * h, s);
        while (i + 1 < plan.segments.length && at >= plan.segments[i].s1) i++;
        rawYaw[kk] = plan.yawOf(plan.segments[i], at);
      }
      plan.yawStep = h;
      plan.yawHalf = half;
      const size = n + 2 * half + 1;
      plan.yaw = new Float64Array(size);
      plan.yawArea = new Float64Array(size);
      for (let j = 0; j < size; j++) {
        const kk = j - half;
        plan.yaw[j] = kk < 0 ? 2 * rawYaw[0] - rawYaw[-kk] : kk > n ? 2 * rawYaw[n] - rawYaw[2 * n - kk] : rawYaw[kk];
        if (j > 0) plan.yawArea[j] = plan.yawArea[j - 1] + (h * (plan.yaw[j - 1] + plan.yaw[j])) / 2;
      }
    }
    plan.endScale = Math.max(1, rho * Math.abs(plan.headingRate(s)));

    // Speed limits sampled along s.
    plan.samples = [];
    const firstSample = [];
    plan.pieces.forEach((p, j) => {
      firstSample.push(plan.samples.length);
      const n = Math.max(1, Math.ceil(p.length / kSampleStep));
      for (let i = 0; i <= n; i++) {
        const x = { s: p.s0 + (p.length * i) / n };
        const geo = plan.geometry(x.s, j, j + 1);
        x.position = geo.p;
        const moved = plan.travel(p, x.s);
        const perMetre = norm(moved);
        let linear = { speed: kInf, accel: kInf, jerk: kInf };
        if (perMetre > 1e-12) {
          const l = linearLimitsAlong(m, moved);
          linear = { speed: l.speed / perMetre, accel: l.accel / perMetre, jerk: l.jerk / perMetre };
        }
        const rate = Math.abs(plan.headingRate(x.s)), bend = Math.abs(plan.headingBend(x.s));
        x.linear_cap = linear.speed;
        x.angular_cap = rate > 1e-9 ? m.angular_speed / rate : kInf;
        const kappa = norm(geo.k);
        x.curvature = kappa;
        x.envelope = Math.min(x.linear_cap, x.angular_cap, kappa > 1e-9 ? Math.sqrt(o.lateral_accel / kappa) : kInf, bend > 1e-9 ? Math.sqrt(m.angular_accel / (2 * bend)) : kInf);
        x.local = x.envelope;
        const spare = m.angular_accel - bend * x.envelope * x.envelope;
        x.accel = Math.min(linear.accel, rate > 1e-9 ? spare / rate : kInf);
        x.jerk = Math.min(linear.jerk, rate > 1e-9 ? m.angular_jerk / rate : kInf);
        if (!isFinite(x.accel)) x.accel = m.linear_accel;
        if (!isFinite(x.jerk)) x.jerk = m.linear_jerk;
        plan.samples.push(x);
      }
    });
    for (let j = 0; j + 1 < plan.pieces.length; j++) {
      const A = plan.pieces[j], B = plan.pieces[j + 1];
      let cap = kInf;
      const jump = norm(sub(plan.travel(A, B.s0), plan.travel(B, B.s0)));
      if (jump > 1e-9) cap = o.kink_speed / jump;
      for (const i of [firstSample[j + 1] - 1, firstSample[j + 1]]) {
        plan.samples[i].envelope = Math.min(plan.samples[i].envelope, cap);
        plan.samples[i].local = Math.min(plan.samples[i].local, cap);
      }
    }
    // Brake for what is ahead, at half the acceleration limit.
    const S = plan.samples;
    for (let j = S.length - 1; j-- > 0;) S[j].envelope = Math.min(S[j].envelope, Math.sqrt(S[j + 1].envelope * S[j + 1].envelope + S[j].accel * (S[j + 1].s - S[j].s)));
    return plan;
  }

  pieceAt(s, begin, end) {
    let lo = begin, hi = end;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (this.pieces[mid].s0 <= s) lo = mid; else hi = mid;
    }
    return lo;
  }
  geometry(s, begin, end) {
    const piece = this.pieces[this.pieceAt(s, begin, end)];
    const { p, d1, d2 } = piece.derivatives(piece.tAt(s - piece.s0));
    const speed = norm(d1);
    if (piece.kind === 'TURN' || speed < 1e-12) return { p, t: [0, 0, 0], k: [0, 0, 0] };
    const t = scale(d1, 1 / speed);
    const k = scale(sub(d2, scale(t, dot(d2, t))), 1 / (speed * speed));
    return { p, t, k };
  }
  yawOf(g, s) {
    const len = g.s1 - g.s0, u = len > 1e-12 ? clamp((s - g.s0) / len, 0, 1) : 1;
    if (g.linear) return g.start_yaw + g.turn * u;
    const x = clamp((s - g.s0) / g.blend, 0, 1);
    return catmullRom(g.mode_yaw, u) + g.delta * (1 - x * x * (3 - 2 * x));
  }
  segmentAt(s) {
    for (let i = 0; i < this.segments.length; i++) if (s < this.segments[i].s1) return i;
    return this.segments.length - 1;
  }
  gridIndex(at) {
    const u = at / this.yawStep + this.yawHalf;
    const i = clamp(Math.floor(u), 0, this.yaw.length - 2);
    return { i, f: u - i };
  }
  heading(s) {
    s = clamp(s, 0, this.length);
    const w = this.yawHalf * this.yawStep;
    if (this.yawHalf === 0) {
      const { i, f } = this.gridIndex(s);
      return this.yaw[i] + f * (this.yaw[i + 1] - this.yaw[i]);
    }
    const area = (at) => {
      const { i, f } = this.gridIndex(at);
      return this.yawArea[i] + this.yawStep * f * (this.yaw[i] + 0.5 * f * (this.yaw[i + 1] - this.yaw[i]));
    };
    return (area(s + w) - area(s - w)) / (2 * w);
  }
  headingRate(s) {
    s = clamp(s, 0, this.length);
    const w = this.yawHalf * this.yawStep;
    const value = (at) => {
      const { i, f } = this.gridIndex(at);
      return this.yaw[i] + f * (this.yaw[i + 1] - this.yaw[i]);
    };
    if (this.yawHalf === 0) {
      const { i } = this.gridIndex(s);
      return (this.yaw[i + 1] - this.yaw[i]) / this.yawStep;
    }
    return (value(s + w) - value(s - w)) / (2 * w);
  }
  headingBend(s) {
    if (this.yawHalf === 0) return 0;
    s = clamp(s, 0, this.length);
    const w = this.yawHalf * this.yawStep;
    const slope = (at) => {
      const { i } = this.gridIndex(at);
      return (this.yaw[i + 1] - this.yaw[i]) / this.yawStep;
    };
    return (slope(s + w) - slope(s - w)) / (2 * w);
  }
  travel(piece, s) {
    if (piece.kind === 'TURN') return piece.length > 0 ? scale(sub(piece.b, piece.a), 1 / piece.length) : [0, 0, 0];
    const { d1 } = piece.derivatives(piece.tAt(s - piece.s0));
    const sp = norm(d1);
    return sp > 1e-12 ? scale(d1, 1 / sp) : [0, 0, 0];
  }
  position(s) { return this.geometry(clamp(s, 0, this.length), 0, this.pieces.length).p; }
  tangent(s) { return this.geometry(clamp(s, 0, this.length), 0, this.pieces.length).t; }
  orientation(s) { return yawQuat(this.heading(s)); }
  sampleAt(s) {
    const S = this.samples;
    let lo = 0, hi = S.length;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (S[mid].s <= s) lo = mid; else hi = mid;
    }
    return lo;
  }
  atEnd(p) { return p.s >= this.length && p.v === 0 && p.a === 0; }

  step(p, linearScale, angularScale, dt) {
    const steps = Math.ceil(dt / kSubstep - 1e-9);
    const h = dt / Math.max(steps, 1);
    for (let i = 0; i < steps; i++) {
      const remaining = this.length - p.s, e = this.endScale;
      if (remaining < 2e-4 / e && Math.abs(p.v) < 2e-3 / e && Math.abs(p.a) < 2e-2 / e) {
        p.s = this.length; p.v = 0; p.a = 0;
        return;
      }
      const here = this.sampleAt(p.s);
      const l = { speed: this.samples[here].linear_cap, accel: this.samples[here].accel, jerk: this.samples[here].jerk };
      let j, braking = false;
      if (remaining < 2e-3 / e && Math.abs(p.v) < 2e-2 / e && Math.abs(p.a) < 1e-1 / e) {
        const c = kCapturePole;
        j = clamp(c * c * c * remaining - 3 * c * c * p.v - 3 * c * p.a, -l.jerk, l.jerk);
      } else {
        const look = p.s + Math.max(p.v, 0) * (l.accel / l.jerk + h);
        let target = 1e3;
        for (let k = here; k < this.samples.length; k++) {
          const x = this.samples[k];
          target = Math.min(target, x.envelope, linearScale * x.linear_cap, angularScale * x.angular_cap);
          l.accel = Math.min(l.accel, x.accel);
          l.jerk = Math.min(l.jerk, x.jerk);
          if (x.s > look) break;
        }
        j = velocityJerk(p.v, p.a, target, l, h);
        const sc = { v: p.v, a: p.a };
        const dc = advance(sc, j, h);
        if (sc.v > 0 && dc + stoppingDistance(sc.v, clamp(sc.a, -l.accel, l.accel), l) > remaining) {
          j = brakingJerk(p.v, p.a, l, h);
          braking = true;
        }
      }
      const st = { v: p.v, a: p.a };
      let d = advance(st, j, h);
      p.v = st.v; p.a = st.a;
      if (p.v < 0 && (braking || remaining > 2e-3 / e)) {
        p.v = p.a = 0;
        d = Math.max(d, 0);
      }
      p.s = clamp(p.s + d, 0, this.length);
    }
  }

  // Reference pose / rates at progress p, written into a MotionProfile.
  pose(p, out) {
    const s = clamp(p.s, 0, this.length);
    const geo = this.geometry(s, 0, this.pieces.length);
    const moved = this.travel(this.pieces[this.pieceAt(s, 0, this.pieces.length)], s);
    out.position = geo.p;
    out.velocity = scale(moved, p.v);
    out.acceleration = add(scale(moved, p.a), scale(geo.k, p.v * p.v));
    out.orientation = this.orientation(s);
    const w = this.headingRate(s);
    const h = 0.02, sp = Math.min(this.length, s + h), sm = Math.max(0, s - h);
    const wDot = sp > sm ? (this.headingRate(sp) - this.headingRate(sm)) / (sp - sm) : 0;
    out.angular_velocity = [0, 0, w * p.v];
    out.angular_acceleration = [0, 0, wDot * p.v * p.v + w * p.a];
  }

  project(pt, near, window) {
    let best = near, bestD = kInf;
    for (let k = this.sampleAt(near - window); k < this.samples.length && this.samples[k].s <= near + window; k++) {
      const d = norm(sub(this.samples[k].position, pt));
      if (d < bestD) { bestD = d; best = this.samples[k].s; }
    }
    return best;
  }

  // Sampled polyline for drawing.
  polyline(step = 0.02) {
    const out = [];
    const n = Math.max(2, Math.ceil(this.length / step));
    for (let i = 0; i <= n; i++) out.push(this.position((this.length * i) / n));
    return out;
  }
}

export { twist };
