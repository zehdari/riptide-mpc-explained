import { setup } from '../layout.js';
import { Graph } from '../viz/graph.js';
import { LineChart, barChart } from '../viz/chart.js';
import { FossenModel, ThrusterReplica } from '../engine/model.js';
import { thrusterNames } from '../engine/talos.js';
import { solveBoxQp } from '../engine/qp.js';
import { createScene } from '../viz/scene.js';

setup();
const model = new FossenModel();
const T = model.T;
const dofs = ['surge X', 'sway Y', 'heave Z', 'roll K', 'pitch M', 'yaw N'];

// ---------- bipartite graph ----------
{
  const host = document.getElementById('bipartite');
  const W = 900;
  const nodes = [
    ...thrusterNames.map((n, i) => ({ id: 't' + i, x: 70 + i * 108, y: 40, label: n, w: 64 })),
    ...dofs.map((d, j) => ({ id: 'd' + j, x: 95 + j * 142, y: 250, label: d, w: 92 })),
  ];
  const edges = [];
  T.forEach((row, j) => row.forEach((v, i) => {
    const s = Math.abs(v) * (j > 2 ? 2.5 : 1);
    if (s > 0.05) edges.push({ from: 't' + i, to: 'd' + j, width: 0.6 + s * 3.2, color: v > 0 ? 'var(--reference)' : 'var(--state)', opacity: 0.85, arrow: false });
  }));
  const note = document.createElement('div');
  note.style.cssText = 'font-family:var(--sans);font-size:.86rem;color:var(--ink-2);min-height:2.6em';
  new Graph(host, {
    width: W, height: 290, nodes, edges, arrows: false,
    info: (n) => {
      if (!n) return (note.textContent = 'Hover a thruster or an axis.');
      if (n.id[0] === 't') {
        const i = +n.id.slice(1);
        note.textContent = `${thrusterNames[i]}: ` + dofs.map((d, j) => `${d.split(' ')[0]} ${T[j][i].toFixed(2)}`).join(', ') + ' (N or N·m per N of thrust)';
      } else {
        const j = +n.id.slice(1);
        const who = thrusterNames.filter((_, i) => Math.abs(T[j][i]) > 0.05);
        note.textContent = `${dofs[j]}: moved by ${who.join(', ')}.`;
      }
    },
  }).highlight(null);
  host.appendChild(note);

  const tm = document.getElementById('tmatrix');
  tm.innerHTML = `<table class="params" style="margin:0"><thead><tr><th>T</th>${thrusterNames.map((n) => `<th style="text-align:right">${n}</th>`).join('')}</tr></thead><tbody>${T.map(
    (row, j) => `<tr><td>${dofs[j]}</td>${row.map((v) => `<td style="text-align:right;color:${Math.abs(v) < 0.05 ? 'var(--muted)' : 'var(--ink)'}">${v.toFixed(3)}</td>`).join('')}</tr>`
  ).join('')}</tbody></table>`;
}

// ---------- null space ----------
function nullSpace() {
  // Projector onto the null space: I - T'(TT')^-1 T, then Gram-Schmidt on its columns.
  const n = 8;
  const TT = T.map((a) => T.map((b) => a.reduce((s, x, k) => s + x * b[k], 0)));
  const inv = invert(TT);
  const P = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0) - T.reduce((s, row, a) => s + row[i] * T.reduce((t, row2, b) => t + inv[a][b] * row2[j], 0), 0)));
  const basis = [];
  for (let c = 0; c < n && basis.length < 2; c++) {
    let v = P.map((r) => r[c]);
    for (const b of basis) { const d = v.reduce((s, x, k) => s + x * b[k], 0); v = v.map((x, k) => x - d * b[k]); }
    const nv = Math.hypot(...v);
    if (nv > 1e-6) basis.push(v.map((x) => x / nv));
  }
  return basis;
}
function invert(M) {
  const n = M.length, a = M.map((r, i) => [...r, ...M.map((_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    const d = a[c][c];
    a[c] = a[c].map((x) => x / d);
    for (let r = 0; r < n; r++) if (r !== c) { const f = a[r][c]; a[r] = a[r].map((x, k) => x - f * a[c][k]); }
  }
  return a.map((r) => r.slice(n));
}
{
  const [A, B] = nullSpace();
  // Rotate the basis so each pattern is simple: maximise weight on forward thrusters for A.
  const fwd = [0, 1, 6, 7];
  const wa = fwd.reduce((s, i) => s + A[i] * A[i], 0), wb = fwd.reduce((s, i) => s + B[i] * B[i], 0), wab = fwd.reduce((s, i) => s + A[i] * B[i], 0);
  const th = 0.5 * Math.atan2(2 * wab, wa - wb);
  const a = A.map((x, i) => Math.cos(th) * x + Math.sin(th) * B[i]);
  const b = A.map((x, i) => -Math.sin(th) * x + Math.cos(th) * B[i]);
  const host = document.getElementById('null-chart');
  host.innerHTML = '<div class="cols"><div><div class="legend"><span><i style="background:var(--state);height:8px"></i>pattern A</span></div><div id="null-a"></div></div><div><div class="legend"><span><i style="background:var(--prediction);height:8px"></i>pattern B</span></div><div id="null-b"></div></div></div>';
  const opts = { digits: 2, max: 0.7, rowH: 22, width: 360, labelW: 48, valueW: 48 };
  barChart(host.querySelector('#null-a'), thrusterNames.map((nm, i) => ({ label: nm, segments: [{ value: a[i], color: 'var(--state)', name: 'A' }], total: a[i] })), opts);
  barChart(host.querySelector('#null-b'), thrusterNames.map((nm, i) => ({ label: nm, segments: [{ value: b[i], color: 'var(--prediction)', name: 'B' }], total: b[i] })), opts);
  const check = T.map((row) => Math.abs(row.reduce((s, t, i) => s + t * a[i], 0)) + Math.abs(row.reduce((s, t, i) => s + t * b[i], 0)));
  console.debug('null check', Math.max(...check));
}

// ---------- allocation ----------
{
  const canvas = document.getElementById('alloc-canvas');
  const scene = createScene(canvas, { floorDepth: -2.2, cameraPos: [-1.6, -1.7, -0.45], target: [-0.05, 0.05, -1.05], interactive: false, autoRotate: true });
  scene.setRobot([0, 0, -1], [1, 0, 0, 0]);
  scene.setReference([0, 0, -50], [1, 0, 0, 0]);
  const ctl = document.getElementById('alloc-controls');
  const axes = [['X', 'surge force', 60, 'N'], ['Y', 'sway force', 40, 'N'], ['Z', 'heave force', 40, 'N'], ['K', 'roll moment', 8, 'N·m'], ['M', 'pitch moment', 8, 'N·m'], ['N', 'yaw moment', 15, 'N·m']];
  const init = [70, 0, 18, 0, 0, 8];
  ctl.innerHTML = axes.map(([k, l, max, u], i) => `<label style="justify-content:space-between">${l} <input type="range" data-i="${i}" min="${-max * 1.6}" max="${max * 1.6}" step="${max / 40}" value="${init[i] ?? 0}"><output>${init[i]} ${u}</output></label>`).join('');
  let method = 'pinv';
  document.querySelectorAll('#alloc-grid [data-m]').forEach((b) => b.addEventListener('click', () => {
    method = b.dataset.m;
    document.querySelectorAll('#alloc-grid [data-m]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
    update();
  }));
  const TT = T.map((a) => T.map((b) => a.reduce((s, x, k) => s + x * b[k], 0)));
  const inv = invert(TT);
  const update = () => {
    const tau = [...ctl.querySelectorAll('input')].map((i) => +i.value);
    ctl.querySelectorAll('input').forEach((i, k) => (i.nextElementSibling.textContent = `${(+i.value).toFixed(1)} ${axes[k][3]}`));
    let f;
    const y = inv.map((r) => r.reduce((s, x, j) => s + x * tau[j], 0));
    const pinv = Array.from({ length: 8 }, (_, i) => T.reduce((s, row, j) => s + row[i] * y[j], 0));
    if (method === 'pinv') f = pinv.map((v) => Math.max(-24, Math.min(24, v)));
    else {
      // min ||W(Tf - tau)||^2 + eps||f||^2  s.t. |f| <= 24  (moments weighted like forces at 0.4 m)
      const w = [1, 1, 1, 6.25, 6.25, 6.25];
      const H = new Float64Array(64), g = new Float64Array(8);
      for (let a = 0; a < 8; a++) {
        for (let b = 0; b < 8; b++) H[a * 8 + b] = T.reduce((s, row, j) => s + w[j] * row[a] * row[b], 0) + (a === b ? 1e-4 : 0);
        g[a] = -T.reduce((s, row, j) => s + w[j] * row[a] * tau[j], 0);
      }
      const x = Float64Array.from(pinv.map((v) => Math.max(-24, Math.min(24, v))));
      solveBoxQp(H, g, new Float64Array(8).fill(-24), new Float64Array(8).fill(24), x, 8, 50);
      f = Array.from(x);
    }
    const got = T.map((row) => row.reduce((s, t, i) => s + t * f[i], 0));
    scene.setThrust(f);
    barChart(document.getElementById('alloc-bars'), thrusterNames.map((n, i) => ({ label: n, segments: [{ value: f[i], color: Math.abs(f[i]) >= 23.99 ? 'var(--s8)' : 'var(--ink-2)' }], total: f[i] })), { digits: 1, max: 24, unit: 'N', rowH: 21, width: 420, labelW: 52, valueW: 60 });
    const err = got.map((g2, j) => g2 - tau[j]);
    const cos = tau.reduce((s, t, j) => s + t * got[j] * (j > 2 ? 6.25 : 1), 0) / Math.sqrt(tau.reduce((s, t, j) => s + t * t * (j > 2 ? 6.25 : 1), 0) * got.reduce((s, t, j) => s + t * t * (j > 2 ? 6.25 : 1), 0) || 1);
    const sat = pinv.filter((v) => Math.abs(v) > 24).length;
    document.getElementById('alloc-result').innerHTML = `Delivered wrench: ${got.map((v, j) => `${dofs[j].split(' ')[1]} ${v.toFixed(1)}`).join(', ')}<br>Largest error ${Math.max(...err.map(Math.abs)).toFixed(1)}; direction off by ${((Math.acos(Math.min(1, cos)) * 180) / Math.PI).toFixed(1)}°. ${sat ? `${sat} thruster${sat > 1 ? 's' : ''} past ±24 N before clipping.` : 'No thruster saturates.'}`;
  };
  ctl.addEventListener('input', update);
  scene.ready.then(update);
  update();
}

// ---------- thruster dynamics ----------
{
  const c = new LineChart(document.getElementById('td-chart'), { height: 250, xLabel: 'time (s)', yLabel: 'force (N)', digits: 2, unit: 'N', xDigits: 3 });
  const run = () => {
    const d = +document.getElementById('td-delay').value, rise = +document.getElementById('td-rise').value, fall = +document.getElementById('td-fall').value;
    document.getElementById('td-delay-o').textContent = `${(d * 1000).toFixed(0)} ms`;
    document.getElementById('td-rise-o').textContent = `${(rise * 1000).toFixed(0)} ms`;
    document.getElementById('td-fall-o').textContent = `${(fall * 1000).toFixed(0)} ms`;
    const m = new FossenModel();
    m.act = { ...m.act, delay: d, rise, fall };
    const act = new ThrusterReplica(m);
    const cmd = [], tgt = [], real = [];
    const h = 0.002;
    for (let k = 0; k <= 700; k++) {
      const t = k * h;
      const u = t < 0.1 ? 0 : t < 0.8 ? 15 : -10;
      if (k % 25 === 0) act.command(new Array(8).fill(u));
      cmd.push({ x: t, y: u });
      tgt.push({ x: t, y: act.target[0] });
      real.push({ x: t, y: act.force[0] });
      act.advance(h);
    }
    c.set([
      { name: 'command', color: 'var(--reference)', dash: true, values: cmd },
      { name: 'after the delay', color: 'var(--muted)', values: tgt, width: 1.5 },
      { name: 'realized force', color: 'var(--state)', values: real },
    ], { yDomain: [-12, 17], markers: [{ x: 0.1, label: 'step' }, { x: 0.15, label: '+50 ms' }] });
  };
  ['td-delay', 'td-rise', 'td-fall'].forEach((id) => document.getElementById(id).addEventListener('input', run));
  run();
}

// ---------- rpm ----------
{
  const c = new LineChart(document.getElementById('rpm-chart'), { height: 230, xLabel: 'thrust (N)', yLabel: 'rpm', digits: 0, unit: 'rpm' });
  const fw = [], rv = [];
  for (let k = 1; k <= 48; k++) {
    const f = k * 0.5;
    fw.push({ x: f, y: model.rpm(f) });
    rv.push({ x: f, y: -model.rpm(-f) });
  }
  c.set([{ name: 'forward', color: 'var(--state)', values: fw }, { name: 'reverse', color: 'var(--reference)', values: rv }], { yDomain: [0, 3000] });
}
