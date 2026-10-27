import { setup } from '../layout.js';
import { LineChart, el } from '../viz/chart.js';
import { Graph } from '../viz/graph.js';
import { profile1d } from '../engine/profile.js';
import { PathPlan } from '../engine/path.js';
import { settings, pathOptions } from '../engine/talos.js';
import { loadDemo, series } from './demo.js';

setup();

// ---------- S-curve ----------
{
  const host = document.getElementById('sc-charts');
  const mk = (yLabel, unit, digits) => {
    const d = document.createElement('div');
    host.appendChild(d);
    return new LineChart(d, { height: 170, xLabel: 'time (s)', yLabel, unit, digits, legend: false });
  };
  const cp = mk('position (m)', 'm', 3), cv = mk('speed (m/s)', 'm/s', 3), ca = mk('accel (m/s²)', 'm/s²', 3), cj = mk('jerk (m/s³)', 'm/s³', 2);
  const ids = ['d', 'v', 'a', 'j'];
  const units = { d: 'm', v: 'm/s', a: 'm/s²', j: 'm/s³' };
  const run = () => {
    const [d, v, a, j] = ids.map((k) => +document.getElementById(`sc-${k}`).value);
    ids.forEach((k) => (document.getElementById(`sc-${k}-o`).textContent = `${(+document.getElementById(`sc-${k}`).value).toFixed(2)} ${units[k]}`));
    const pts = profile1d(d, { speed: v, accel: a, jerk: j }, 0.02, 80);
    const T = pts.at(-1).t;
    const opt = { xDomain: [0, Math.max(T * 1.05, 1)], markers: [{ x: T, label: `arrives ${T.toFixed(1)} s` }] };
    cp.set([{ name: 'position', color: 'var(--reference)', values: pts.map((p) => ({ x: p.t, y: p.x })) }], { ...opt, yDomain: [0, d * 1.08] });
    cv.set([{ name: 'speed', color: 'var(--reference)', values: pts.map((p) => ({ x: p.t, y: p.v })) }], { ...opt, yDomain: [0, v * 1.1] });
    ca.set([{ name: 'accel', color: 'var(--reference)', values: pts.map((p) => ({ x: p.t, y: p.a })) }], { ...opt, yDomain: [-a * 1.15, a * 1.15], yZero: true });
    cj.set([{ name: 'jerk', color: 'var(--reference)', values: pts.map((p) => ({ x: p.t, y: p.j })) }], { ...opt, yDomain: [-j * 1.15, j * 1.15], yZero: true });
  };
  ids.forEach((k) => document.getElementById(`sc-${k}`).addEventListener('input', run));
  run();
}

// ---------- path editor ----------
{
  const wrap = document.getElementById('pp-wrap');
  const left = document.createElement('div'), right = document.createElement('div');
  wrap.append(left, right);
  const W = 520, H = 420, scale = 95, ox = 150, oy = 340; // world (x right, y up) -> svg
  const toS = (p) => [ox + p[0] * scale, oy - p[1] * scale];
  const toW = (x, y) => [(x - ox) / scale, (oy - y) / scale];
  const svg = el('svg', { class: 'chart', viewBox: `0 0 ${W} ${H}`, style: 'touch-action:none;background:var(--paper);border-radius:8px' }, left);
  const info = document.createElement('div');
  info.style.cssText = 'font-family:var(--sans);font-size:.84rem;color:var(--ink-2);margin-top:.4rem';
  left.appendChild(info);
  const chart = new LineChart(right, { height: 300, xLabel: 's (m)', yLabel: 'speed (m/s)', unit: 'm/s', digits: 3, xDigits: 2 });
  const pts = [[0, 0], [2, 0], [2, 1.6], [0.5, 2.6], [-0.6, 1.0]];
  let heading = 'PATH', plan = null, sim = null, playing = false;
  const motion = settings.motion;
  const colors = { LINE: 'var(--state)', ARC: 'var(--prediction)', BEZIER: 'var(--reference)', TURN: 'var(--s7)' };
  const gGrid = el('g', {}, svg), gPath = el('g', {}, svg), gPts = el('g', {}, svg), gRef = el('g', {}, svg);
  for (let x = -1; x <= 3.5; x += 0.5) { const [sx] = toS([x, 0]); el('line', { x1: sx, x2: sx, y1: 0, y2: H, stroke: 'var(--grid)' }, gGrid); }
  for (let y = -0.5; y <= 4; y += 0.5) { const [, sy] = toS([0, y]); el('line', { x1: 0, x2: W, y1: sy, y2: sy, stroke: 'var(--grid)' }, gGrid); }
  const scaleText = el('text', { class: 'label', x: W - 10, y: H - 10, 'text-anchor': 'end' }, gGrid);
  scaleText.textContent = 'grid 0.5 m';
  const refDot = el('circle', { r: 7, fill: 'var(--reference)', stroke: 'var(--paper)', 'stroke-width': 2, visibility: 'hidden' }, gRef);
  const refNose = el('line', { stroke: 'var(--reference)', 'stroke-width': 2.5, 'stroke-linecap': 'round', visibility: 'hidden' }, gRef);
  const chainHost = document.getElementById('pp-chain');

  function rebuild() {
    const r = +document.getElementById('pp-r').value, lat = +document.getElementById('pp-lat').value;
    document.getElementById('pp-r-o').textContent = `${r.toFixed(2)} m`;
    document.getElementById('pp-lat-o').textContent = `${lat.toFixed(2)} m/s²`;
    try {
      plan = PathPlan.build({ position: [...pts[0], -1], yaw: 0 }, pts.slice(1).map((p, i) => ({ position: [...p, -1], heading, yaw: i === pts.length - 2 ? Math.PI / 2 : 0 })), motion, { ...pathOptions, corner_radius: r, lateral_accel: lat });
    } catch (e) {
      info.textContent = e.message;
      return;
    }
    gPath.innerHTML = '';
    for (const p of plan.pieces) {
      const n = Math.max(2, Math.ceil(p.length / 0.02));
      const d = Array.from({ length: n + 1 }, (_, i) => toS(plan.position(p.s0 + (p.length * i) / n))).map((q, i) => `${i ? 'L' : 'M'}${q[0].toFixed(1)},${q[1].toFixed(1)}`).join('');
      el('path', { d, fill: 'none', stroke: colors[p.kind], 'stroke-width': 3, 'stroke-linecap': 'round' }, gPath);
    }
    // straight-line skeleton between the points
    el('path', { d: pts.map((p, i) => `${i ? 'L' : 'M'}${toS(p).join(',')}`).join(''), fill: 'none', stroke: 'var(--muted)', 'stroke-dasharray': '3 4', 'stroke-width': 1 }, gPath);
    // simulate the reference along s
    const prog = { s: 0, v: 0, a: 0 };
    sim = [];
    for (let t = 0; t < 120; t += 0.05) {
      sim.push({ t, s: prog.s, v: prog.v });
      if (plan.atEnd(prog)) break;
      plan.step(prog, 1, 1, 0.05);
    }
    const env = plan.samples.map((x) => ({ x: x.s, y: Math.min(x.envelope, 0.8) }));
    const loc = plan.samples.map((x) => ({ x: x.s, y: Math.min(x.local, 0.8) }));
    chart.set([
      { name: 'local limit', color: 'var(--muted)', dash: true, width: 1.5, values: loc },
      { name: 'envelope', color: 'var(--ink-2)', values: env },
      { name: 'reference speed', color: 'var(--reference)', values: sim.map((p) => ({ x: p.s, y: p.v })) },
    ], { yDomain: [0, 0.7], xDomain: [0, plan.length] });
    info.textContent = `Length ${plan.length.toFixed(2)} m of s · ${plan.pieces.length} pieces · the reference takes ${sim.at(-1).t.toFixed(1)} s, top speed ${Math.max(...sim.map((p) => p.v)).toFixed(2)} m/s.`;
    // chain graph
    chainHost.innerHTML = '';
    const k = plan.pieces.length, gw = 900, step = Math.min(118, (gw - 60) / k);
    new Graph(chainHost, {
      width: gw, height: 70,
      nodes: plan.pieces.map((p, i) => ({ id: 'p' + i, x: 40 + step * (i + 0.5), y: 35, label: p.kind.toLowerCase(), sub: `${p.length.toFixed(2)} m`, w: step - 22, stroke: colors[p.kind] })),
      edges: plan.pieces.slice(1).map((_, i) => ({ from: 'p' + i, to: 'p' + (i + 1) })),
    });
    drawPoints();
  }
  function drawPoints() {
    gPts.innerHTML = '';
    pts.forEach((p, i) => {
      const [x, y] = toS(p);
      const c = el('circle', { cx: x, cy: y, r: i ? 9 : 6, fill: i ? 'var(--paper)' : 'var(--ink)', stroke: 'var(--ink)', 'stroke-width': 2, style: i ? 'cursor:grab' : '' }, gPts);
      if (!i) return;
      const t = el('text', { class: 'label', x: x + 12, y: y - 10 }, gPts);
      t.textContent = i;
      c.addEventListener('pointerdown', (e) => {
        c.setPointerCapture(e.pointerId);
        const move = (ev) => {
          const r = svg.getBoundingClientRect();
          const sx = ((ev.clientX - r.left) / r.width) * W, sy = ((ev.clientY - r.top) / r.height) * H;
          pts[i] = toW(Math.max(10, Math.min(W - 10, sx)), Math.max(10, Math.min(H - 10, sy))).map((v) => Math.round(v * 20) / 20);
          rebuild();
        };
        const up = () => { c.removeEventListener('pointermove', move); c.removeEventListener('pointerup', up); };
        c.addEventListener('pointermove', move);
        c.addEventListener('pointerup', up);
      });
    });
  }
  document.getElementById('pp-r').addEventListener('input', rebuild);
  document.getElementById('pp-lat').addEventListener('input', rebuild);
  document.querySelectorAll('[data-h]').forEach((b) => b.addEventListener('click', () => {
    heading = b.dataset.h;
    document.querySelectorAll('[data-h]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
    rebuild();
  }));
  document.getElementById('pp-play').addEventListener('click', () => {
    if (!plan || !sim || playing) return;
    playing = true;
    const t0 = performance.now(), speedUp = 2;
    refDot.setAttribute('visibility', 'visible');
    refNose.setAttribute('visibility', 'visible');
    const frame = (now) => {
      const t = ((now - t0) / 1000) * speedUp;
      const k = Math.min(sim.length - 1, Math.floor(t / 0.05));
      const s = sim[k].s;
      const [x, y] = toS(plan.position(s));
      const yaw = plan.heading(s);
      refDot.setAttribute('cx', x); refDot.setAttribute('cy', y);
      refNose.setAttribute('x1', x); refNose.setAttribute('y1', y);
      refNose.setAttribute('x2', x + 22 * Math.cos(yaw)); refNose.setAttribute('y2', y - 22 * Math.sin(yaw));
      if (k < sim.length - 1) requestAnimationFrame(frame);
      else playing = false;
    };
    requestAnimationFrame(frame);
  });
  rebuild();
  const mq = matchMedia('(max-width: 760px)');
  const layout = () => (wrap.style.gridTemplateColumns = mq.matches ? '1fr' : 'minmax(0,1.15fr) minmax(0,1fr)');
  mq.addEventListener('change', layout);
  layout();
}

// ---------- governor ----------
{
  const host = document.getElementById('gov-charts');
  const a = document.createElement('div'), b = document.createElement('div');
  host.append(a, b);
  const lag = new LineChart(a, { height: 230, xLabel: 'time (s)', yLabel: 'reference ahead of vehicle (m)', unit: 'm', digits: 3 });
  const spd = new LineChart(b, { height: 230, xLabel: 'time (s)', yLabel: 'surge speed (m/s)', unit: 'm/s', digits: 3 });
  Promise.all([loadDemo('gov_on'), loadDemo('gov_off')]).then(([on, off]) => {
    lag.set([
      { name: 'governor on', color: 'var(--state)', values: series(on, (r) => r.rx - r.x) },
      { name: 'governor off', color: 'var(--reference)', values: series(off, (r) => r.rx - r.x) },
    ]);
    spd.set([
      { name: 'vehicle, on', color: 'var(--state)', values: series(on, (r) => r.u) },
      { name: 'vehicle, off', color: 'var(--reference)', values: series(off, (r) => r.u) },
      { name: 'reference, off', color: 'var(--reference)', dash: true, width: 1.5, values: series(off, (r) => r.rvx) },
    ]);
  });
}
