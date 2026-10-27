// Runs the ported controller through the demo scenarios and writes public/data/demo_*.json.
import { writeFileSync } from 'node:fs';
import { Simulation, makePlantHydro, Mode } from '../src/engine/sim.js';
import { PathPlan } from '../src/engine/path.js';
import { pathOptions } from '../src/engine/talos.js';
import { quatToRpy, twist, yawQuat } from '../src/engine/math.js';

const r4 = (v) => Math.round(v * 1e4) / 1e4;
const only = process.argv[2];

function run({ name, seconds, settings = {}, plant = {}, learn = true, refAt, predEvery = 0, extra }) {
  if (only && only !== name) return;
  const sim = new Simulation({ settings: { model_step: 0.01, ...settings }, plantHydro: makePlantHydro(plant), learn });
  const hold = sim.holdHere();
  const rows = [], preds = [];
  let ms = 0;
  for (let k = 0; sim.t < seconds - 1e-9; k++) {
    const ref = refAt(sim.t, hold, sim);
    const out = sim.tick(ref);
    ms += out.solve_ms;
    const p = sim.basePose();
    const live = sim.controller.live;
    const rpy = quatToRpy(p.orientation);
    rows.push([sim.t, ...p.position, ...rpy, ...sim.model.baseLinkVelocity(sim.x), ...live.pose.position, twist(live.pose.orientation), ...live.pose.velocity, ...out.command, ...sim.controller.model.disturbance, out.qp_iterations, out.solve_ms, sim.controller.governor ?? 1, ref.path ? live.progress.s : 0, ref.path ? live.progress.v : 0].map(r4));
    if (predEvery && k % predEvery === 0) preds.push({ t: r4(sim.t), p: out.prediction.map((x) => sim.model.baseLinkPosition(x).map(r4)) });
  }
  const cols = ['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'u', 'v', 'w', 'rx', 'ry', 'rz', 'ryaw', 'rvx', 'rvy', 'rvz', 'f0', 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'dfx', 'dfy', 'dfz', 'dtx', 'dty', 'dtz', 'qp', 'ms', 'gov', 's', 'sv'];
  const data = { name, cols, rows, preds, ...(extra ? extra(sim) : {}) };
  writeFileSync(`public/data/demo_${name}.json`, JSON.stringify(data));
  console.log(name, rows.length, 'ticks, avg solve', (ms / rows.length).toFixed(1), 'ms');
}

const step = (dx, dz = 0, at = 1) => (t, h) => (t < at ? h : { ...h, position: [h.position[0] + dx, h.position[1], h.position[2] + dz] });

run({ name: 'horizon', seconds: 12, plant: { dragScale: 2 }, learn: false, refAt: step(1.5), predEvery: 3 });
run({ name: 'ff_on', seconds: 16, settings: { r_thrust: 0.03 }, refAt: step(1.5) });
run({ name: 'ff_off', seconds: 16, settings: { feedforward: false, r_thrust: 0.03 }, refAt: step(1.5) });
const plant = { buoyancy: 6, cobShift: [0.012, 0, 0] };
const dist = (t, h) => (t < 20 ? h : { ...h, position: [h.position[0] + 1, h.position[1], h.position[2]] });
run({ name: 'learn_on', seconds: 40, plant, learn: true, refAt: dist });
run({ name: 'learn_off', seconds: 40, plant, learn: false, refAt: dist });
run({ name: 'gov_on', seconds: 30, settings: { motion: { linear_speed: 0.6, linear_accel: 0.3, linear_jerk: 0.6 } }, refAt: step(8) });
run({ name: 'gov_off', seconds: 30, settings: { governor_lag: 50, governor_stop_lag: 60, motion: { linear_speed: 0.6, linear_accel: 0.3, linear_jerk: 0.6 } }, refAt: step(8) });
run({ name: 'yaw', seconds: 8, refAt: (t, h) => (t < 0.5 ? h : { ...h, orientation: yawQuat(Math.PI / 2) }) });
{
  let plan = null;
  run({
    name: 'path',
    seconds: 40,
    predEvery: 10,
    refAt: (t, h, sim) => {
      if (t < 0.5) return h;
      if (!plan) {
        const p = h.position;
        plan = PathPlan.build({ position: p, yaw: 0 }, [
          { position: [p[0] + 2, p[1], p[2]], heading: 'PATH' },
          { position: [p[0] + 2, p[1] + 1.6, p[2]], heading: 'PATH' },
          { position: [p[0] + 2, p[1] + 1.6, p[2]], shape: 'ARC', center: [p[0] + 2, p[1] + 2.4, p[2]], sweep: Math.PI, heading: 'PATH' },
          { position: [p[0], p[1] + 3.2, p[2]], heading: 'WAYPOINT', yaw: Math.PI },
        ], sim.controller.s.motion, pathOptions);
      }
      return { linear_mode: Mode.POSITION, angular_mode: Mode.POSITION, position: plan.position(plan.length), orientation: plan.orientation(plan.length), path: plan };
    },
    extra: () => ({
      planned: plan.polyline(0.02).map((p) => p.map(r4)),
      envelope: plan.samples.filter((_, i) => i % 2 === 0).map((x) => [r4(x.s), r4(Math.min(x.envelope, 1)), r4(Math.min(x.local, 1)), r4(x.curvature)]),
      length: r4(plan.length),
      pieces: plan.pieces.map((p) => ({ kind: p.kind, s0: r4(p.s0), length: r4(p.length) })),
    }),
  });
}
