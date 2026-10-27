// Model-error lab: the plant differs from the controller's model by the errors the page sets.
// Streams one frame per control tick, paced to `speed` × real time.
import { Simulation, makePlantHydro } from '../engine/sim.js';
import { hydro } from '../engine/talos.js';
import { quatToRpy } from '../engine/math.js';

let sim = null, hold = null, ref = null, speed = 1, wall0 = 0, sim0 = 0, running = true;

// e: { mass, volume, comX, cobX, cobZ, thrust, vus, inertia, lag, drag, surgeAdded, learn } as fractions / metres / factors.
function build(e) {
  const plant = makePlantHydro({
    massScale: 1 + e.mass,
    buoyancy: e.volume * 338.07,
    comShift: [e.comX, 0, 0],
    cobShift: [e.cobX, 0, e.cobZ],
    dragScale: 1 + e.drag,
    inertiaScale: 1 + e.inertia,
  });
  // The controller's surge added mass is what the page sets; the plant keeps the identified value.
  const model = { ...hydro, added_mass: [e.surgeAdded, ...hydro.added_mass.slice(1)] };
  const tweak = (m) => {
    m.T = m.T.map((r) => r.map((v, i) => v * (1 + e.thrust) * (i === 0 ? 1 + e.vus : 1)));
    m.act = { ...m.act, delay: m.act.delay * e.lag, rise: m.act.rise * e.lag, fall: m.act.fall * e.lag };
  };
  const s = new Simulation({ settings: { model_step: 0.01 }, plantHydro: plant, modelHydro: model, learn: e.learn, plantTweak: tweak });
  return s;
}

function start(e) {
  sim = build(e);
  hold = sim.holdHere();
  ref = hold;
  wall0 = performance.now();
  sim0 = 0;
}

function loop() {
  if (sim && running) {
    const t0 = performance.now();
    const target = ((t0 - wall0) / 1000) * speed;
    const frames = [];
    while (sim.t - sim0 < target && performance.now() - t0 < 30) {
      const out = sim.tick(ref);
      const p = sim.basePose();
      const rpy = quatToRpy(p.orientation);
      const live = sim.controller.live.pose;
      frames.push({
        t: sim.t,
        x: p.position[0] - hold.position[0],
        rx: live.position[0] - hold.position[0],
        dz: p.position[2] - live.position[2],
        roll: rpy[0],
        pitch: rpy[1],
        err: Math.hypot(p.position[0] - live.position[0], p.position[1] - live.position[1], p.position[2] - live.position[2]),
        thrust: out.command.reduce((s, v) => s + Math.abs(v), 0),
        d: sim.controller.model.disturbance.slice(),
        ms: out.solve_ms,
      });
    }
    if (sim.t - sim0 < target - 0.5) wall0 = performance.now() - ((sim.t - sim0) / speed) * 1000;
    if (frames.length) postMessage(frames);
  }
  setTimeout(loop, 10);
}

onmessage = ({ data }) => {
  if (data.type === 'start') start(data.errors);
  else if (data.type === 'move') ref = { ...hold, position: [hold.position[0] + data.dx, hold.position[1], hold.position[2]] };
  else if (data.type === 'speed') { speed = data.speed; wall0 = performance.now(); sim0 = sim ? sim.t : 0; }
  else if (data.type === 'running') { running = data.on; wall0 = performance.now(); sim0 = sim ? sim.t : 0; }
};

loop();
