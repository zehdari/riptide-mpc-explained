import { setup } from '../layout.js';
import { Graph } from '../viz/graph.js';
import { LineChart, barChart } from '../viz/chart.js';
import { FossenModel } from '../engine/model.js';
import { hydro, vehicle } from '../engine/talos.js';
import { rpyQuat, quatToRpy } from '../engine/math.js';

setup();
const model = new FossenModel();
const phone = matchMedia('(max-width: 640px)');
const mm = (v) => v.map((x) => (x * 1000).toFixed(0)).join(', ');

// ---------- frame tree ----------
{
  const host = document.getElementById('frame-graph');
  const note = document.createElement('div');
  note.style.cssText = 'font-family:var(--sans);font-size:.86rem;color:var(--ink-2);min-height:2.6em;margin-top:.3rem';
  const rel = (p) => mm(p.map((x, i) => x - hydro.com[i]));
  const txt = {
    world: 'World (odometry) frame: x, y on the surface plane, z up. Depth is negative.',
    base_link: `base_link: what odometry reports and every setpoint uses. ${rel(vehicle.base_link)} mm from the COM.`,
    origin: 'CAD origin: the farthest-port screw on the camera cage. Every pose in the vehicle YAML is measured from here.',
    com: `Centre of mass, measured wet: (${mm(hydro.com)}) mm from the origin. The physics is written here.`,
    cob: `Centre of buoyancy: (${mm(hydro.cob)}) mm from the COM. 22.5 mm above it, so Talos rights itself.`,
    thrusters: 'Eight thrusters. Each has a position and an axis; relative to the COM they form the 6×8 thruster matrix (Chapter 2).',
    dvl: `DVL: measures velocity of its own point, v + ω × r with r = (${rel(vehicle.dvl)}) mm from the COM.`,
    imu: 'IMU: body rates and the gravity direction (roll/pitch). Mounted rotated; the estimator rotates its readings into the body frame.',
    depth: 'Depth sensor: pressure → z of base_link.',
  };
  const g = new Graph(host, {
    width: 900,
    height: 300,
    nodes: [
      { id: 'world', x: 80, y: 150, label: 'world', sub: 'odometry frame' },
      { id: 'base_link', x: 270, y: 150, label: 'base_link', sub: 'setpoints, odometry', w: 130 },
      { id: 'origin', x: 470, y: 150, label: 'CAD origin', sub: 'vehicle YAML', w: 118 },
      { id: 'com', x: 670, y: 60, label: 'COM', sub: 'the model', bold: true },
      { id: 'cob', x: 830, y: 60, label: 'COB', sub: 'buoyancy', w: 92 },
      { id: 'thrusters', x: 690, y: 150, label: '8 thrusters', sub: 'pose + axis', w: 112 },
      { id: 'dvl', x: 650, y: 245, label: 'DVL', w: 70 },
      { id: 'imu', x: 740, y: 245, label: 'IMU', w: 70 },
      { id: 'depth', x: 830, y: 245, label: 'depth', w: 70 },
    ],
    edges: [
      { from: 'world', to: 'base_link', label: 'odometry (moving)' },
      { from: 'origin', to: 'base_link', label: 'fixed' },
      { from: 'origin', to: 'com', label: 'measured' },
      { from: 'com', to: 'cob', label: 'identified' },
      { from: 'origin', to: 'thrusters' },
      { from: 'origin', to: 'dvl' },
      { from: 'origin', to: 'imu' },
      { from: 'origin', to: 'depth' },
    ],
    info: (n) => (note.textContent = n ? txt[n.id] : 'Hover or tap a frame.'),
  });
  host.appendChild(note);
  g.highlight(null);
}

// ---------- every term, live ----------
{
  const sliders = [
    ['u', 'surge u', -0.6, 0.6, 0.4, 'm/s'],
    ['v', 'sway v', -0.6, 0.6, 0.1, 'm/s'],
    ['w', 'heave w', -0.6, 0.6, 0, 'm/s'],
    ['r', 'yaw rate r', -1, 1, 0.3, 'rad/s'],
    ['roll', 'roll', -30, 30, 0, '°'],
    ['pitch', 'pitch', -30, 30, 5, '°'],
  ];
  const ctl = document.getElementById('term-controls');
  const state = {};
  ctl.innerHTML = sliders.map(([k, label, lo, hi, v, unit]) => `<label>${label} <input type="range" data-k="${k}" min="${lo}" max="${hi}" step="${(hi - lo) / 120}" value="${v}"><output>${v} ${unit}</output></label>`).join('');
  const table = document.getElementById('term-table');
  const names = ['surge X', 'sway Y', 'heave Z', 'roll K', 'pitch M', 'yaw N'];
  const update = () => {
    ctl.querySelectorAll('input').forEach((i) => {
      state[i.dataset.k] = +i.value;
      const s = sliders.find((s) => s[0] === i.dataset.k);
      i.nextElementSibling.textContent = `${(+i.value).toFixed(s[5] === '°' ? 0 : 2)} ${s[5]}`;
    });
    const q = rpyQuat((state.roll * Math.PI) / 180, (state.pitch * Math.PI) / 180, 0);
    const x = [0, 0, -1, ...q, state.u, state.v, state.w, 0, 0, state.r];
    const parts = {};
    const dx = model.derivative(x, new Array(8).fill(0), parts);
    const cols = [
      ['Damping', 'D(ν)ν', parts.D.map((v) => v)],
      ['Coriolis, body', '−C_RB(ν)ν', parts.cRB.map((v) => -v)],
      ['Coriolis, added mass', '−C_A(ν)ν', parts.cA.map((v) => -v)],
      ['Gravity + buoyancy', '−g(η)', parts.g],
    ];
    const total = names.map((_, i) => cols.reduce((s, c) => s + c[2][i], 0));
    const narrow = phone.matches;
    const cell = (v, max) => {
      const w = Math.min(100, (Math.abs(v) / max) * 100);
      return `<td style="text-align:right;position:relative"><span style="position:absolute;right:0;top:20%;bottom:20%;width:${w}%;background:${v >= 0 ? 'rgba(42,120,214,.16)' : 'rgba(235,104,52,.18)'};border-radius:2px"></span><span style="position:relative">${v.toFixed(narrow ? 1 : 2)}</span></td>`;
    };
    const max = (i) => Math.max(1e-6, ...cols.map((c) => Math.abs(c[2][i])), Math.abs(total[i]));
    // Phones get short headers and one decimal so all seven columns fit.
    const heads = narrow ? ['Drag', 'C<sub>RB</sub>', 'C<sub>A</sub>', 'g', 'Sum', 'ν̇'] : [...cols.map((c) => c[0]), 'Sum', 'Acceleration ν̇'];
    const rowName = (n) => (narrow ? n.split(' ')[1] : n);
    const accel = (i) => (narrow ? dx[7 + i].toFixed(2) : `${dx[7 + i].toFixed(3)} ${i < 3 ? 'm/s²' : 'rad/s²'}`);
    table.innerHTML = `<table class="params" style="margin:0"><thead><tr><th></th>${heads.map((h) => `<th style="text-align:right">${h}</th>`).join('')}</tr></thead><tbody>${names
      .map((n, i) => `<tr><td>${rowName(n)}</td>${cols.map((c) => cell(c[2][i], max(i))).join('')}${cell(total[i], max(i))}<td style="text-align:right">${accel(i)}</td></tr>`)
      .join('')}</tbody></table>${narrow ? '<p class="eq-note" style="margin:.6rem 0 0">Drag D(ν)ν; C<sub>RB</sub>, C<sub>A</sub>: Coriolis of the body and of the added mass; g: gravity and buoyancy. N (X–Z) and N·m (K–N); ν̇ in m/s² and rad/s².</p>' : ''}`;
  };
  phone.addEventListener('change', () => update());
  ctl.addEventListener('input', update);
  update();
}

// ---------- effective mass ----------
{
  const rigid = [hydro.mass, hydro.mass, hydro.mass, ...hydro.inertia.map((r, i) => r[i])];
  const labels = ['surge', 'sway', 'heave', 'roll ×20', 'pitch ×20', 'yaw ×20'];
  const host = document.getElementById('mass-chart');
  const leg = document.createElement('div');
  leg.className = 'legend';
  leg.innerHTML = '<span><i style="background:var(--state);height:8px"></i>rigid body</span><span><i style="background:var(--prediction);height:8px"></i>added mass</span>';
  host.appendChild(leg);
  const box = document.createElement('div');
  host.appendChild(box);
  barChart(box, labels.map((l, i) => ({ label: l, segments: [{ value: rigid[i] * (i > 2 ? 20 : 1), color: 'var(--state)', name: 'rigid' }, { value: hydro.added_mass[i] * (i > 2 ? 20 : 1), color: 'var(--prediction)', name: 'added' }], total: rigid[i] + hydro.added_mass[i], unit: i > 2 ? ' kg·m²' : ' kg' })), { digits: 1, valueW: 92 });
}

// ---------- drag ----------
{
  const c = new LineChart(document.getElementById('drag-chart'), { height: 260, xLabel: 'speed (m/s)', yLabel: 'drag (N)', digits: 1, unit: 'N', xDigits: 2 });
  const axis = (i) => Array.from({ length: 61 }, (_, k) => { const s = k / 60; return { x: s, y: hydro.linear_damping[i] * s + hydro.quadratic_damping[i] * s * s }; });
  c.set([
    { name: 'surge', color: 'var(--state)', values: axis(0) },
    { name: 'sway', color: 'var(--reference)', values: axis(1) },
    { name: 'heave', color: 'var(--prediction)', values: axis(2) },
    { name: '73 N budget', color: 'var(--ink-2)', dash: true, width: 1.5, noTip: true, values: [{ x: 0, y: 73 }, { x: 1, y: 73 }] },
  ], { yDomain: [0, 300], xDomain: [0, 1] });
}

// ---------- free response ----------
{
  const c = new LineChart(document.getElementById('tilt-chart'), { height: 240, xLabel: 'time (s)', yLabel: 'angle (°)', digits: 1, unit: '°' });
  const run = () => {
    const r0 = +document.getElementById('tilt-roll').value, p0 = +document.getElementById('tilt-pitch').value;
    document.getElementById('tilt-roll-o').textContent = `${r0}°`;
    document.getElementById('tilt-pitch-o').textContent = `${p0}°`;
    let x = [0, 0, -1.5, ...rpyQuat((r0 * Math.PI) / 180, (p0 * Math.PI) / 180, 0), 0, 0, 0, 0, 0, 0];
    const roll = [], pitch = [], climb = [];
    const zero = new Array(8).fill(0);
    for (let k = 0; k <= 400; k++) {
      if (k % 4 === 0) {
        const [r, p] = quatToRpy([x[3], x[4], x[5], x[6]]);
        roll.push({ x: k * 0.02, y: (r * 180) / Math.PI });
        pitch.push({ x: k * 0.02, y: (p * 180) / Math.PI });
        climb.push({ x: k * 0.02, y: (x[2] + 1.5) * 100 });
      }
      x = model.rk4(x, zero, 0.02);
    }
    c.set([
      { name: 'roll', color: 'var(--state)', values: roll },
      { name: 'pitch', color: 'var(--reference)', values: pitch },
    ], { yDomain: [-12, 46] });
  };
  document.getElementById('tilt-roll').addEventListener('input', run);
  document.getElementById('tilt-pitch').addEventListener('input', run);
  run();
}
