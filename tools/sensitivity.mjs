// How much each model parameter matters: the plant differs from the controller's model in one
// parameter, by an error typical of how that parameter is obtained. Two runs per case:
//   hold, learning off, 20 s          -> steady position / attitude error (the statics)
//   hold 20 s then a 1.5 m surge, learning on -> worst tracking error and pitch while moving
// Writes public/data/sensitivity.json.
import { writeFileSync } from 'node:fs';
import { Simulation, makePlantHydro } from '../src/engine/sim.js';
import { hydro } from '../src/engine/talos.js';
import { angularDistance, quatToRpy } from '../src/engine/math.js';

const cases = [
  { id: 'baseline', label: 'Exact model', source: '—' },
  { id: 'mass', label: 'Mass +1% (0.34 kg)', source: 'lab: scale', plant: { massScale: 1.01 } },
  { id: 'volume', label: 'Displaced volume −1%', source: 'CAD, then pool statics', plant: { buoyancy: -0.01 * 338.07 } },
  { id: 'com', label: 'COM 5 mm forward', source: 'lab: balance', plant: { comShift: [0.005, 0, 0] } },
  { id: 'cob', label: 'COB 5 mm forward', source: 'CAD, then pool statics', plant: { cobShift: [0.005, 0, 0] } },
  { id: 'cobz', label: 'COB 10 mm lower', source: 'CAD, then pool statics', plant: { cobShift: [0, 0, -0.01] } },
  { id: 'thrust', label: 'Thrust curve −10%', source: 'Blue Robotics T200 data', tweak: (m) => { m.T = m.T.map((r) => r.map((v) => v * 0.9)); } },
  { id: 'onethr', label: 'One thruster −20% (VUS)', source: 'per-thruster gains', tweak: (m) => { m.T = m.T.map((r) => r.map((v, i) => (i === 0 ? v * 0.8 : v))); } },
  { id: 'inertia', label: 'Rotational inertia +20%', source: 'CAD', plant: { inertiaScale: 1.2 } },
  { id: 'lag', label: 'Thruster lag ×2', source: 'bench', tweak: (m) => { m.act = { ...m.act, delay: 0.06, rise: 0.16, fall: 0.12 }; } },
  { id: 'drag', label: 'Drag +30%', source: 'pool: drag runs', plant: { dragScale: 1.3 } },
  { id: 'added', label: 'Surge added mass 32 kg (was in talos.yaml)', source: 'pool: inertia fits', model: { ...hydro, added_mass: [31.9014, ...hydro.added_mass.slice(1)] } },
];

function run(c, learn, seconds, moveAt) {
  const sim = new Simulation({ settings: { model_step: 0.01 }, plantHydro: makePlantHydro(c.plant || {}), modelHydro: c.model || hydro, learn, plantTweak: c.tweak });
  const hold = sim.holdHere();
  const goal = { ...hold, position: [hold.position[0] + 1.5, hold.position[1], hold.position[2]] };
  let moveErr = 0, movePitch = 0, thrust = 0, out;
  for (let k = 0; sim.t < seconds - 1e-9; k++) {
    const moving = moveAt !== undefined && sim.t >= moveAt;
    out = sim.tick(moving ? goal : hold);
    const p = sim.basePose();
    if (moving) {
      const ref = sim.controller.live.pose.position;
      moveErr = Math.max(moveErr, Math.hypot(p.position[0] - ref[0], p.position[1] - ref[1], p.position[2] - ref[2]));
      movePitch = Math.max(movePitch, Math.abs(quatToRpy(p.orientation)[1]));
      thrust = Math.max(thrust, out.command.reduce((s, v) => s + Math.abs(v), 0));
    }
  }
  const p = sim.basePose(), ref = (moveAt !== undefined ? goal : hold);
  return {
    pos: Math.hypot(...p.position.map((v, i) => v - ref.position[i])),
    att: angularDistance(p.orientation, ref.orientation),
    moveErr, movePitch, thrust,
    d: sim.controller.model.disturbance.slice(),
  };
}

const rows = [];
for (const c of cases) {
  const h = run(c, false, 20);
  const m = run(c, true, 36, 20);
  const r = { id: c.id, label: c.label, source: c.source, hold_cm: +(100 * h.pos).toFixed(2), hold_deg: +((h.att * 180) / Math.PI).toFixed(2), move_cm: +(100 * m.moveErr).toFixed(2), move_pitch_deg: +((m.movePitch * 180) / Math.PI).toFixed(2), end_cm: +(100 * m.pos).toFixed(2), peak_thrust: +m.thrust.toFixed(1), learned: m.d.map((v) => +v.toFixed(2)) };
  rows.push(r);
  console.log(JSON.stringify(r));
}
writeFileSync('public/data/sensitivity.json', JSON.stringify({ rows }, null, 1));
