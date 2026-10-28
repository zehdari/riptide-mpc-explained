import { setup } from '../layout.js';
import { Graph } from '../viz/graph.js';
import { LineChart } from '../viz/chart.js';
import { loadDemo, loadTrace, series } from './demo.js';

setup();

// ---------- estimator graph ----------
{
  const host = document.getElementById('est-graph');
  const note = document.createElement('div');
  note.style.cssText = 'font-family:var(--sans);font-size:.86rem;color:var(--ink-2);min-height:2.6em';
  const txt = {
    model: 'Between measurements, the state is propagated with the Fossen model and a replica of the commanded thrust, in 2 ms steps.',
    gyro: 'IMU gyro: body rates, applied outright (gain 1). Its innovations also teach the disturbance torque.',
    fog: 'Fibre-optic gyro: yaw rate, owns the yaw axis while fresh (gain 1).',
    tilt: 'IMU orientation, used only for the gravity direction: roll and pitch pulled in with a 1 s time constant.',
    dvl: 'DVL: velocity of its own point (v + ω × r), gain 0.7, compared at its timestamp. Its innovations teach the disturbance force.',
    depth: 'Depth sensor: base_link z, 0.3 s time constant.',
    ekf: 'EKF: x, y and heading anchors (2 s), and a 0.3 s stand-in for any stale sensor.',
    pose: 'Position p (world): z from depth, x and y only from the EKF anchor.',
    att: 'Attitude q: roll and pitch from gravity, yaw from the EKF anchor, everything fast from integrating the rates.',
    vel: 'Body velocity v at the COM.',
    rate: 'Body rates ω.',
    dist: 'Learned disturbance wrench d̂: force (world frame) from DVL innovations, torque (body) from gyro innovations, both with 3 s time constants.',
  };
  const g = new Graph(host, {
    width: 900, height: 360, arrows: true,
    nodes: [
      { id: 'gyro', x: 100, y: 40, label: 'IMU gyro', w: 120 },
      { id: 'fog', x: 100, y: 95, label: 'FOG', w: 120 },
      { id: 'tilt', x: 100, y: 150, label: 'IMU gravity', w: 120 },
      { id: 'dvl', x: 100, y: 205, label: 'DVL', w: 120 },
      { id: 'depth', x: 100, y: 260, label: 'depth', w: 120 },
      { id: 'ekf', x: 100, y: 315, label: 'EKF', w: 120 },
      { id: 'rate', x: 520, y: 55, label: 'rates ω', w: 120 },
      { id: 'att', x: 520, y: 135, label: 'attitude q', w: 120 },
      { id: 'vel', x: 520, y: 215, label: 'velocity v', w: 120 },
      { id: 'pose', x: 520, y: 295, label: 'position p', w: 120 },
      { id: 'model', x: 800, y: 135, label: 'Fossen model', sub: '+ thruster replica', w: 150 },
      { id: 'dist', x: 800, y: 295, label: 'disturbance d̂', w: 140, stroke: 'var(--accent)' },
    ],
    edges: [
      { from: 'gyro', to: 'rate', label: 'gain 1' },
      { from: 'fog', to: 'rate', label: 'yaw axis, gain 1' },
      { from: 'tilt', to: 'att', label: 'roll, pitch: τ 1 s' },
      { from: 'dvl', to: 'vel', label: 'gain 0.7' },
      { from: 'depth', to: 'pose', label: 'z: τ 0.3 s' },
      { from: 'ekf', to: 'pose', label: 'x, y and yaw: τ 2 s', dash: true },
      { from: 'ekf', to: 'att', dash: true, bend: -90 },
      { from: 'model', to: 'rate', label: 'propagate', bend: 10 },
      { from: 'model', to: 'att', bend: 0 },
      { from: 'model', to: 'vel', bend: -10 },
      { from: 'model', to: 'pose', bend: -20 },
      { from: 'rate', to: 'dist', label: 'innovations', dash: true, bend: -70 },
      { from: 'vel', to: 'dist', dash: true },
      { from: 'dist', to: 'model', label: 'τ_d', dash: true },
    ],
    info: (n) => (note.textContent = n ? txt[n.id] : 'Tap or hover a sensor or a part of the state.'),
  });
  g.highlight(null);
  host.appendChild(note);
}

// ---------- disturbance learning ----------
{
  const host = document.getElementById('dist-charts');
  const mk = (yLabel, unit) => {
    const d = document.createElement('div');
    host.appendChild(d);
    return new LineChart(d, { height: 210, xLabel: 'time (s)', yLabel, unit, digits: 2 });
  };
  const cz = mk('depth error (cm)', 'cm'), cp = mk('pitch (°)', '°'), cf = mk('learned heave force (N)', 'N'), ct = mk('learned pitch torque (N·m)', 'N·m');
  Promise.all([loadDemo('learn_on'), loadDemo('learn_off')]).then(([on, off]) => {
    const pair = (f) => [
      { name: 'learning on', color: 'var(--state)', values: series(on, f) },
      { name: 'learning off', color: 'var(--reference)', values: series(off, f) },
    ];
    const mv = { markers: [{ x: 20, label: 'move 1 m' }] };
    cz.set(pair((r) => 100 * (r.z - r.rz)), { yZero: true, ...mv });
    cp.set(pair((r) => (r.pitch * 180) / Math.PI), { yZero: true, ...mv });
    cf.set([{ name: 'learned', color: 'var(--state)', values: series(on, (r) => r.dfz) }, { name: 'true error', color: 'var(--ink-2)', dash: true, width: 1.5, noTip: true, values: [{ x: 0, y: 6 }, { x: 40, y: 6 }] }], { yZero: true, ...mv });
    ct.set([{ name: 'learned', color: 'var(--state)', values: series(on, (r) => r.dty) }], { yZero: true, ...mv });
  });
}

// ---------- trust ----------
{
  const host = document.getElementById('trust-chart');
  host.classList.add('cols');
  const a = document.createElement('div'), b = document.createElement('div');
  host.append(a, b);
  const cp = new LineChart(a, { height: 220, xLabel: 'time (s)', yLabel: 'progress along the step (m)', unit: 'm', digits: 3 });
  const ct = new LineChart(b, { height: 220, xLabel: 'time (s)', yLabel: 'settle trust', digits: 3, legend: false });
  loadTrace('surge_step').then((d) => {
    const i0 = d.pos.findIndex((p) => p && p[0] !== null);
    const start = d.pos[i0], goal = d.commands[0].vect;
    const dir = [goal[0] - start[0], goal[1] - start[1]], L = Math.hypot(...dir);
    const prog = (p) => (p && p[0] !== null ? ((p[0] - start[0]) * dir[0] + (p[1] - start[1]) * dir[1]) / L : null);
    const pts = (arr) => d.t.map((t, i) => ({ x: t, y: prog(arr[i]) })).filter((p) => p.y !== null);
    cp.set([
      { name: 'reference', color: 'var(--reference)', dash: true, values: pts(d.ref_pos) },
      { name: 'Talos', color: 'var(--state)', values: pts(d.pos) },
    ], { yDomain: [-0.05, 2.1] });
    ct.set([{ name: 'trust', color: 'var(--prediction)', values: d.t.map((t, i) => ({ x: t, y: d.trust[i] })).filter((p) => p.y !== null) }], { yDomain: [0, 1.05], markers: [{ x: d.commands[0].t, label: 'new setpoint' }] });
  });
}
