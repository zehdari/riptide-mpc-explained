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
  const info = document.getElementById('h-info');
  const names = ['VUS', 'VUP', 'HUS', 'HUP', 'HLS', 'HLP', 'VLS', 'VLP'];
    // Continuous blue ramp on a square-root scale; the smallest coupling is still visibly blue, never background.
  const seqColor = (u) => {
    const t = Math.sqrt(Math.max(0, Math.min(1, u)));
    const lo = [205, 226, 251], hi = [13, 54, 107];
    return `rgb(${lo.map((l, i) => Math.round(l + (hi[i] - l) * t)).join(',')})`;
  };
  // diverging: blue (negative) - neutral - orange (positive)
  const div = (u) => {
    const t = Math.min(1, Math.abs(u));
    const end = u < 0 ? [42, 120, 214] : [235, 104, 52], mid = [231, 239, 240];
    const c = mid.map((m, i) => Math.round(m + (end[i] - m) * t));
    return `rgb(${c.join(',')})`;
  };
  let sim = null, k = 0;
  const drawStages = (H, n) => {
    const N = n / 8, S = 440, pad = 52, cs = (S - pad) / N;
    const B = Array.from({ length: N }, (_, j) => Array.from({ length: N }, (_, kk) => {
      let v = 0;
      for (let a = 0; a < 8; a++) for (let c = 0; c < 8; c++) v += H[(8 * j + a) * n + 8 * kk + c] ** 2;
      return Math.sqrt(v);
    }));
    const mx = Math.max(...B.flat());
    let cells = '';
    B.forEach((row, j) => row.forEach((v, kk) => {
      cells += `<rect x="${pad + kk * cs}" y="${pad + j * cs}" width="${cs + 0.3}" height="${cs + 0.3}" fill="${seqColor(v / mx)}"><title>stages ${j} and ${kk}: ${(v / mx).toFixed(2)}</title></rect>`;
    }));
    let ticks = '';
    for (let t = 0; t < N; t += 5) {
      ticks += `<text class="label" x="${pad + (t + 0.5) * cs}" y="${pad - 8}" text-anchor="middle">${t}</text><text class="label" x="${pad - 8}" y="${pad + (t + 0.5) * cs + 4}" text-anchor="end">${t}</text>`;
    }
    left.innerHTML = `<div style="font-family:var(--sans);font-size:.86rem;color:var(--ink-2);margin-bottom:.3rem">Coupling between stages (each cell: one 8 × 8 block of H)</div><svg class="chart" viewBox="0 0 ${S} ${S}">${cells}${ticks}<text class="label" x="${pad}" y="14">stage of one command →</text><text class="label" transform="translate(14 ${pad}) rotate(90)">stage of the other →</text></svg>`;
  };
  const drawBlock = (H, n) => {
    const S = 420, pad = 44, cs = (S - pad) / 8;
    const blk = names.map((_, a) => names.map((_, b) => H[a * n + b]));
    const mx = Math.max(...blk.flat().map(Math.abs));
    let cells = '';
    blk.forEach((row, a) => row.forEach((v, b) => {
      const u = v / mx;
      cells += `<rect x="${pad + b * cs}" y="${pad + a * cs}" width="${cs - 2}" height="${cs - 2}" rx="2" fill="${div(u)}"/><text x="${pad + b * cs + cs / 2 - 1}" y="${pad + a * cs + cs / 2 + 3}" text-anchor="middle" style="font-size:11.5px;fill:${Math.abs(u) > 0.6 ? '#fff' : 'var(--ink)'}">${u.toFixed(2)}</text>`;
    }));
    const labels = names.map((nm, i) => `<text class="label" x="${pad + i * cs + cs / 2}" y="${pad - 8}" text-anchor="middle">${nm}</text><text class="label" x="${pad - 6}" y="${pad + i * cs + cs / 2 + 4}" text-anchor="end">${nm}</text>`).join('');
    right.innerHTML = `<div style="font-family:var(--sans);font-size:.86rem;color:var(--ink-2);margin-bottom:.3rem">Stage 0's block: how the eight first commands interact</div><svg class="chart" viewBox="0 0 ${S} ${S}">${cells}${labels}</svg><div class="legend" style="margin-top:.3rem"><span><i style="background:rgb(235,104,52);height:8px"></i>same effect (positive)</span><span><i style="background:rgb(42,120,214);height:8px"></i>opposite effects (negative)</span></div>`;
  };
  const run = () => {
    if (!sim) sim = new Simulation({ settings: { model_step: 0.01 } });
    const goals = [[1.2, 0.6, -1.3], [0.2, -0.5, -0.8], [1.5, -0.3, -1.1]];
    const ref = sim.holdHere();
    ref.position = goals[k++ % goals.length];
    for (let i = 0; i < 14; i++) sim.tick(ref);
    sim.controller.debug = true;
    const out = sim.tick(ref);
    sim.controller.debug = false;
    const n = Math.round(Math.sqrt(out.H.length));
    drawStages(out.H, n);
    drawBlock(out.H, n);
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
