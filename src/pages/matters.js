import { setup } from '../layout.js';
import { LineChart } from '../viz/chart.js';
import { hydro } from '../engine/talos.js';

setup();

// ---------- the precomputed study ----------
const phone = matchMedia('(max-width: 640px)');
fetch(`${import.meta.env.BASE_URL}data/sensitivity.json`).then((r) => r.json()).then(({ rows }) => {
  const cols = [
    ['hold_cm', 'Hold: position', 'cm', 'var(--state)'],
    ['hold_deg', 'Hold: attitude', '°', 'var(--state)'],
    ['move_cm', 'Move: tracking', 'cm', 'var(--reference)'],
  ];
  const max = Object.fromEntries(cols.map(([k]) => [k, Math.max(...rows.map((r) => r[k]))]));
  const host = document.getElementById('sens-table');
  const table = () => {
    const cell = (v, k, color) => `<td style="text-align:right;position:relative;min-width:6.5rem"><span style="position:absolute;right:3.4rem;top:30%;bottom:30%;width:calc((100% - 3.8rem) * ${(v / max[k]).toFixed(3)});background:${color};border-radius:2px;opacity:.8"></span><span style="position:relative">${v.toFixed(2)}</span></td>`;
    return `<table class="params" style="margin:0"><thead><tr><th>Wrong by</th><th>Comes from</th>${cols.map((c) => `<th style="text-align:right">${c[1]} (${c[2]})</th>`).join('')}</tr></thead><tbody>${rows
      .map((r) => `<tr><td>${r.label}</td><td>${r.source}</td>${cols.map(([k, , , c]) => cell(r[k], k, c)).join('')}</tr>`)
      .join('')}</tbody></table>`;
  };
  // Phones: one card per case, its three results as labelled bars.
  const cards = () => rows
    .map((r) => `<div style="padding:.7rem 0;border-bottom:1px solid var(--rule);font-family:var(--sans)">
      <div style="font-weight:600;font-size:.92rem">${r.label}</div>
      <div style="font-size:.78rem;color:var(--ink-2);margin-bottom:.35rem">${r.source}</div>
      ${cols.map(([k, label, unit, color]) => `<div style="display:grid;grid-template-columns:7.2rem 1fr 3.6rem;align-items:center;gap:.5rem;font-size:.8rem;color:var(--ink-2)"><span>${label}</span><span style="height:8px"><span style="display:block;height:100%;width:${((100 * r[k]) / max[k]).toFixed(1)}%;background:${color};border-radius:2px;opacity:.8"></span></span><span style="text-align:right;color:var(--ink);font-variant-numeric:tabular-nums">${r[k].toFixed(2)} ${unit}</span></div>`).join('')}
    </div>`)
    .join('');
  const draw = () => (host.innerHTML = phone.matches ? cards() : table());
  phone.addEventListener('change', draw);
  draw();
});

// ---------- the live lab ----------
const sliders = [
  ['mass', 'mass', -3, 3, 0.25, 0, '%', 0.01],
  ['volume', 'displaced volume', -3, 3, 0.25, 0, '%', 0.01],
  ['comX', 'COM forward', -15, 15, 1, 0, 'mm', 0.001],
  ['cobX', 'COB forward', -15, 15, 1, 0, 'mm', 0.001],
  ['cobZ', 'COB up', -20, 20, 1, 0, 'mm', 0.001],
  ['thrust', 'thrust curve', -30, 30, 5, 0, '%', 0.01],
  ['vus', 'one thruster (VUS)', -50, 20, 5, 0, '%', 0.01],
  ['drag', 'drag', -50, 100, 10, 0, '%', 0.01],
  ['inertia', 'rotational inertia', -40, 60, 10, 0, '%', 0.01],
  ['lag', 'thruster lag', 0.5, 4, 0.25, 1, '×', 1],
  ['surgeAdded', "model's surge added mass", 0, 200, 4, Math.round(hydro.added_mass[0]), 'kg', 1],
];
const presets = {
  exact: {},
  mass: { mass: 1 },
  com: { comX: 5 },
  added: { surgeAdded: 32 },
  all: { mass: 0.5, volume: -0.5, comX: 3, cobX: -2, thrust: -10, drag: 20, inertia: 15, lag: 1.5 },
};
const host = document.getElementById('lab-sliders');
host.innerHTML = sliders.map(([k, label, lo, hi, step, v, unit]) => `<label style="display:flex;align-items:center;justify-content:space-between;gap:.6rem">${label} <span style="display:inline-flex;align-items:center;gap:.5rem"><input type="range" data-k="${k}" min="${lo}" max="${hi}" step="${step}" value="${v}"><output style="min-width:4.2em;color:var(--ink);font-variant-numeric:tabular-nums"></output></span></label>`).join('');
const inputs = [...host.querySelectorAll('input')];
const fmt = (s, v) => `${s[6] === '×' || s[6] === 'kg' ? '' : v > 0 ? '+' : ''}${v}${s[6] === '×' ? '×' : ' ' + s[6]}`;
const showValues = () => inputs.forEach((i) => (i.nextElementSibling.textContent = fmt(sliders.find((s) => s[0] === i.dataset.k), +i.value)));
const errors = () => {
  const e = { learn: document.getElementById('lab-learn').checked };
  inputs.forEach((i) => { const s = sliders.find((x) => x[0] === i.dataset.k); e[s[0]] = +i.value * s[7]; });
  return e;
};

const worker = new Worker(new URL('../worker/lab.worker.js', import.meta.url), { type: 'module' });
const charts = document.getElementById('lab-charts');
const mk = (yLabel, unit, digits = 2) => {
  const d = document.createElement('div');
  charts.appendChild(d);
  return new LineChart(d, { height: 200, xLabel: 'time (s)', yLabel, unit, digits });
};
const cPos = mk('surge position (m)', 'm', 3), cErr = mk('error from the reference (cm)', 'cm'), cPitch = mk('attitude (°)', '°'), cForce = mk('learned disturbance force (N)', 'N'), cTorque = mk('learned disturbance torque (N·m)', 'N·m');
const stats = document.getElementById('lab-stats');
let buf = [], dirty = false, started = false;
const WINDOW = 40;
worker.onmessage = ({ data }) => {
  buf.push(...data);
  const t = buf[buf.length - 1].t;
  while (buf.length && buf[0].t < t - WINDOW) buf.shift();
  dirty = true;
};
const every = (f, k) => buf.filter((_, i) => i % k === 0).map(f);
setInterval(() => {
  if (!dirty || !buf.length) return;
  dirty = false;
  const k = buf.length > 400 ? 2 : 1;
  const x0 = Math.max(0, buf[buf.length - 1].t - WINDOW), x1 = Math.max(WINDOW, buf[buf.length - 1].t);
  const xd = { xDomain: [x0, x1] };
  cPos.set([
    { name: 'reference', color: 'var(--reference)', dash: true, values: every((f) => ({ x: f.t, y: f.rx }), k) },
    { name: 'Talos', color: 'var(--state)', values: every((f) => ({ x: f.t, y: f.x }), k) },
  ], { ...xd, yDomain: [-0.1, 1.7] });
  cErr.set([
    { name: 'distance to reference', color: 'var(--state)', values: every((f) => ({ x: f.t, y: 100 * f.err }), k) },
    { name: 'depth error', color: 'var(--prediction)', values: every((f) => ({ x: f.t, y: 100 * f.dz }), k) },
  ], { ...xd, yZero: true });
  cPitch.set([
    { name: 'pitch', color: 'var(--state)', values: every((f) => ({ x: f.t, y: (f.pitch * 180) / Math.PI }), k) },
    { name: 'roll', color: 'var(--prediction)', values: every((f) => ({ x: f.t, y: (f.roll * 180) / Math.PI }), k) },
  ], { ...xd, yZero: true });
  cForce.set([
    { name: 'heave', color: 'var(--state)', values: every((f) => ({ x: f.t, y: f.d[2] }), k) },
    { name: 'surge', color: 'var(--prediction)', values: every((f) => ({ x: f.t, y: f.d[0] }), k) },
  ], { ...xd, yZero: true });
  cTorque.set([
    { name: 'pitch', color: 'var(--state)', values: every((f) => ({ x: f.t, y: f.d[4] }), k) },
    { name: 'roll', color: 'var(--prediction)', values: every((f) => ({ x: f.t, y: f.d[3] }), k) },
  ], { ...xd, yZero: true });
  const f = buf[buf.length - 1];
  const tiles = [
    ['Error now', `${(100 * f.err).toFixed(1)} cm`],
    ['Pitch now', `${((f.pitch * 180) / Math.PI).toFixed(2)}°`],
    ['Total thrust', `${f.thrust.toFixed(1)} N`],
    ['Solve', `${f.ms.toFixed(0)} ms`],
  ];
  stats.innerHTML = tiles.map(([l, v]) => `<div style="background:var(--paper);border-radius:8px;padding:.5rem .7rem"><div style="font-size:.78rem;color:var(--ink-2)">${l}</div><div style="font-size:1.15rem;font-weight:600;font-variant-numeric:tabular-nums">${v}</div></div>`).join('');
}, 200);

const restart = () => {
  buf = [];
  showValues();
  worker.postMessage({ type: 'start', errors: errors() });
};
let pending = null;
const later = () => { clearTimeout(pending); pending = setTimeout(restart, 350); };
host.addEventListener('input', () => { showValues(); later(); });
document.getElementById('lab-learn').addEventListener('change', restart);
document.querySelectorAll('[data-preset]').forEach((b) => b.addEventListener('click', () => {
  const p = presets[b.dataset.preset];
  inputs.forEach((i) => { const s = sliders.find((x) => x[0] === i.dataset.k); i.value = p[s[0]] ?? s[5]; });
  restart();
}));
document.querySelectorAll('[data-move]').forEach((b) => b.addEventListener('click', () => worker.postMessage({ type: 'move', dx: +b.dataset.move })));
document.querySelectorAll('[data-speed]').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('[data-speed]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
  worker.postMessage({ type: 'speed', speed: +b.dataset.speed });
}));
// Start when the lab scrolls into view, pause when it leaves.
new IntersectionObserver(([e]) => {
  if (e.isIntersecting && !started) { started = true; restart(); }
  worker.postMessage({ type: 'running', on: e.isIntersecting });
}).observe(charts);
showValues();
