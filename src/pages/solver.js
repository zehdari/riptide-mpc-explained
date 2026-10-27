import { setup } from '../layout.js';
import { LineChart, el } from '../viz/chart.js';
import { Simulation } from '../engine/sim.js';
import { solveBoxQp } from '../engine/qp.js';
import { loadDemo, loadTrace, series } from './demo.js';

setup();

// ---------- manifold picture ----------
{
  const host = document.getElementById('manifold-fig');
  const W = 560, H = 330, cx = 200, cy = 175, R = 125;
  const svg = el('svg', { class: 'chart', viewBox: `0 0 ${W} ${H}`, style: 'touch-action:none' }, host);
  el('circle', { cx, cy, r: R, fill: 'none', stroke: 'var(--ink-2)', 'stroke-width': 1.5 }, svg);
  el('text', { class: 'label', x: cx, y: cy + 4, 'text-anchor': 'middle' }, svg).textContent = 'unit attitudes';
  const g = el('g', {}, svg);
  let th = 2.2;
  const delta = 0.75;
  const draw = () => {
    g.innerHTML = '';
    const p = [cx + R * Math.cos(th), cy - R * Math.sin(th)];
    const t = [-Math.sin(th), -Math.cos(th)]; // tangent (screen coords, counterclockwise)
    const add = [p[0] + R * delta * t[0], p[1] + R * delta * t[1]];
    const bp = [cx + R * Math.cos(th + delta), cy - R * Math.sin(th + delta)];
    el('line', { x1: p[0], y1: p[1], x2: add[0], y2: add[1], stroke: 'var(--muted)', 'stroke-width': 2, 'stroke-dasharray': '5 4' }, g);
    el('circle', { cx: add[0], cy: add[1], r: 5, fill: 'var(--muted)' }, g);
    el('text', { class: 'label', x: add[0] + 8, y: add[1] - 6 }, g).textContent = 'x + δ: off the surface';
    const a0 = -th, a1 = -(th + delta);
    el('path', { d: `M${p[0]},${p[1]} A${R},${R} 0 0 0 ${bp[0]},${bp[1]}`, fill: 'none', stroke: 'var(--prediction)', 'stroke-width': 3.5, 'stroke-linecap': 'round' }, g);
    el('circle', { cx: bp[0], cy: bp[1], r: 6, fill: 'var(--prediction)', stroke: 'var(--paper-2)', 'stroke-width': 2 }, g);
    el('text', { class: 'label', x: bp[0] + (Math.cos(th + delta) > 0 ? 10 : -10), y: bp[1] + 16, 'text-anchor': Math.cos(th + delta) > 0 ? 'start' : 'end' }, g).textContent = 'x ⊞ δ: along it';
    const dot = el('circle', { cx: p[0], cy: p[1], r: 9, fill: 'var(--accent)', stroke: 'var(--paper-2)', 'stroke-width': 2, style: 'cursor:grab' }, g);
    el('text', { class: 'label', x: p[0] + (Math.cos(th) > 0 ? 14 : -14), y: p[1] + 4, 'text-anchor': Math.cos(th) > 0 ? 'start' : 'end' }, g).textContent = 'x';
    void a0; void a1;
    dot.addEventListener('pointerdown', (e) => {
      dot.setPointerCapture(e.pointerId);
      const move = (ev) => {
        const r = svg.getBoundingClientRect();
        const x = ((ev.clientX - r.left) / r.width) * W - cx, y = cy - ((ev.clientY - r.top) / r.height) * H;
        th = Math.atan2(y, x);
        draw();
      };
      dot.addEventListener('pointermove', move);
      dot.addEventListener('pointerup', () => dot.removeEventListener('pointermove', move), { once: true });
    });
  };
  draw();
}

// ---------- the condensed Hessian ----------
{
  const grid = document.getElementById('h-grid');
  const left = document.createElement('div'), right = document.createElement('div');
  grid.append(left, right);
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 480;
  canvas.style.cssText = 'width:100%;max-width:480px;aspect-ratio:1;display:block;image-rendering:pixelated;border-radius:4px';
  left.appendChild(canvas);
  const row = new LineChart(right, { height: 300, xLabel: 'command index (stage × 8 + thruster)', yLabel: 'H[VUS₀, ·]', digits: 4, xDigits: 0, legend: false });
  const info = document.getElementById('h-info');
  let sim = null, k = 0;
  const ramp = ['#e8f1fc', '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'];
  const run = () => {
    if (!sim) {
      sim = new Simulation({ settings: { model_step: 0.01 } });
    }
    const goals = [[1.2, 0.6, -1.3], [0.2, -0.5, -0.8], [1.5, -0.3, -1.1]];
    const ref = sim.holdHere();
    ref.position = goals[k++ % goals.length];
    for (let i = 0; i < 14; i++) sim.tick(ref);
    sim.controller.debug = true;
    const out = sim.tick(ref);
    sim.controller.debug = false;
    const H = out.H, n = Math.round(Math.sqrt(H.length));
    const ctx = canvas.getContext('2d');
    const img = ctx.createImageData(n, n);
    let max = 0;
    for (let i = 0; i < H.length; i++) max = Math.max(max, Math.abs(H[i]));
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) {
        const v = Math.abs(H[i * n + j]) / max;
        const l = v > 0 ? Math.max(0, Math.min(1, 1 + Math.log10(v) / 6)) : 0;
        const c = ramp[Math.min(ramp.length - 1, Math.floor(l * ramp.length))];
        const o = (i * n + j) * 4;
        img.data[o] = parseInt(c.slice(1, 3), 16);
        img.data[o + 1] = parseInt(c.slice(3, 5), 16);
        img.data[o + 2] = parseInt(c.slice(5, 7), 16);
        img.data[o + 3] = l > 0 ? 255 : 0;
      }
    const tmp = document.createElement('canvas');
    tmp.width = tmp.height = n;
    tmp.getContext('2d').putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, 480, 480);
    ctx.drawImage(tmp, 0, 0, 480, 480);
    row.set([{ name: 'H row', color: 'var(--state)', values: Array.from({ length: n }, (_, j) => ({ x: j, y: H[j] })) }], { yZero: true });
    info.textContent = `${n} × ${n}, solved in ${out.solve_ms.toFixed(0)} ms with ${out.qp_iterations} QP iteration${out.qp_iterations === 1 ? '' : 's'} (browser).`;
  };
  document.getElementById('h-run').addEventListener('click', run);
  new IntersectionObserver(([e], obs) => { if (e.isIntersecting) { obs.disconnect(); setTimeout(run, 50); } }).observe(grid);
}

// ---------- projected Newton in 2-D ----------
{
  const host = document.getElementById('qp-fig');
  const W = 900, H = 440, S = 7, ox = 250, oy = 220; // 1 N = S px
  const svg = el('svg', { class: 'chart', viewBox: `0 0 ${W} ${H}`, style: 'touch-action:none' }, host);
  const toS = (p) => [ox + p[0] * S, oy - p[1] * S];
  const toW = (x, y) => [(x - ox) / S, (oy - y) / S];
  let c = [34, 12], x0 = [-10, -16];
  const gC = el('g', {}, svg), gB = el('g', {}, svg), gP = el('g', {}, svg), gH = el('g', {}, svg);
  const [bx, by] = toS([-24, 24]);
  el('rect', { x: bx, y: by, width: 48 * S, height: 48 * S, fill: 'rgba(15,38,51,0.04)', stroke: 'var(--ink)', 'stroke-width': 1.5 }, gB);
  el('text', { class: 'label', x: bx + 4, y: by - 6 }, gB).textContent = 'box: −24 ≤ uᵢ ≤ 24';
  const side = el('g', {}, svg);
  const draw = () => {
    const rot = (+document.getElementById('qp-rot').value * Math.PI) / 180, cond = +document.getElementById('qp-cond').value;
    document.getElementById('qp-rot-o').textContent = `${(+document.getElementById('qp-rot').value).toFixed(0)}°`;
    document.getElementById('qp-cond-o').textContent = `${cond.toFixed(1)}`;
    const cr = Math.cos(rot), sr = Math.sin(rot), l1 = 1, l2 = cond;
    const Hm = [cr * cr * l1 + sr * sr * l2, cr * sr * (l1 - l2), cr * sr * (l1 - l2), sr * sr * l1 + cr * cr * l2];
    const Hf = Float64Array.from(Hm);
    const g = Float64Array.from([-(Hm[0] * c[0] + Hm[1] * c[1]), -(Hm[2] * c[0] + Hm[3] * c[1])]);
    const x = Float64Array.from(x0);
    const trace = [];
    const res = solveBoxQp(Hf, g, Float64Array.from([-24, -24]), Float64Array.from([24, 24]), x, 2, 50, 1e-8, trace);
    gC.innerHTML = '';
    const [ccx, ccy] = toS(c);
    for (const lev of [4, 16, 36, 64, 100, 160, 240, 340]) {
      el('ellipse', { cx: ccx, cy: ccy, rx: Math.sqrt((2 * lev) / l1) * S, ry: Math.sqrt((2 * lev) / l2) * S, transform: `rotate(${(-rot * 180) / Math.PI} ${ccx} ${ccy})`, fill: 'none', stroke: 'var(--rule)', 'stroke-width': 1.2 }, gC);
    }
    gP.innerHTML = '';
    const pts = trace.map(toS);
    el('path', { d: pts.map((p, i) => `${i ? 'L' : 'M'}${p[0]},${p[1]}`).join(''), fill: 'none', stroke: 'var(--accent)', 'stroke-width': 2 }, gP);
    pts.forEach((p, i) => { if (i) el('circle', { cx: p[0], cy: p[1], r: 4.5, fill: 'var(--accent)', stroke: 'var(--paper-2)', 'stroke-width': 2 }, gP); });
    gH.innerHTML = '';
    const cross = el('g', { style: 'cursor:grab' }, gH);
    el('circle', { cx: ccx, cy: ccy, r: 14, fill: 'transparent' }, cross);
    el('path', { d: `M${ccx - 8},${ccy - 8}L${ccx + 8},${ccy + 8}M${ccx + 8},${ccy - 8}L${ccx - 8},${ccy + 8}`, stroke: 'var(--ink)', 'stroke-width': 2.5 }, cross);
    const [sx0, sy0] = toS(x0);
    const start = el('circle', { cx: sx0, cy: sy0, r: 8, fill: 'var(--paper)', stroke: 'var(--ink)', 'stroke-width': 2, style: 'cursor:grab' }, gH);
    const drag = (node, set) => node.addEventListener('pointerdown', (e) => {
      node.setPointerCapture(e.pointerId);
      const move = (ev) => {
        const r = svg.getBoundingClientRect();
        set(toW(((ev.clientX - r.left) / r.width) * W, ((ev.clientY - r.top) / r.height) * H));
        draw();
      };
      node.addEventListener('pointermove', move);
      node.addEventListener('pointerup', () => node.removeEventListener('pointermove', move), { once: true });
    });
    drag(cross, (p) => (c = p.map((v) => Math.max(-50, Math.min(50, v)))));
    drag(start, (p) => (x0 = p.map((v) => Math.max(-24, Math.min(24, v)))));
    side.innerHTML = '';
    const lines = [
      `iterations: ${res.iterations}${res.converged ? ', converged' : ''}`,
      `solution: (${x[0].toFixed(2)}, ${x[1].toFixed(2)})`,
      `active bounds: ${[0, 1].filter((i) => Math.abs(Math.abs(x[i]) - 24) < 1e-6).map((i) => `u${'₁₂'[i]}`).join(', ') || 'none'}`,
    ];
    lines.forEach((t, i) => (el('text', { class: 'label', x: 560, y: 120 + i * 24, style: 'font-size:14px' }, side).textContent = t));
    el('text', { class: 'label', x: 560, y: 210 }, side).textContent = '× unconstrained minimum   ○ warm start   ● iterates';
  };
  ['qp-rot', 'qp-cond'].forEach((id) => document.getElementById(id).addEventListener('input', draw));
  draw();
}

// ---------- RTI stats ----------
{
  const host = document.getElementById('rti-charts');
  const a = document.createElement('div'), b = document.createElement('div');
  host.append(a, b);
  const ca = new LineChart(a, { height: 220, xLabel: 'time (s)', yLabel: 'C++ solve time (ms)', unit: 'ms', digits: 1, legend: false });
  const cb = new LineChart(b, { height: 220, xLabel: 'time (s)', yLabel: 'QP iterations (browser)', unit: '', digits: 0, legend: false });
  loadTrace('path').then((d) => {
    ca.set([{ name: 'solve time', color: 'var(--state)', width: 1.5, values: d.t.map((t, i) => ({ x: t, y: d.solve_ms[i] })) },
      { name: '50 ms period', color: 'var(--ink-2)', dash: true, width: 1.2, noTip: true, values: [{ x: d.t[0], y: 50 }, { x: d.t.at(-1), y: 50 }] }], { yDomain: [0, 55] });
  });
  loadDemo('gov_on').then((d) => cb.set([{ name: 'QP iterations', color: 'var(--prediction)', width: 1.5, values: series(d, (r) => r.qp) }], { yDomain: [0, 6] }));
}
