import { setup } from '../layout.js';
import { Graph } from '../viz/graph.js';
import { LineChart, el } from '../viz/chart.js';
import { loadDemo, series } from './demo.js';

setup();

// ---------- factor graph ----------
{
  const host = document.getElementById('fg');
  const note = document.getElementById('fg-note');
  let view = 'full';
  const sub = (k) => String(k).split('').map((d) => '₀₁₂₃₄₅₆₇₈₉'[d]).join('');
  function draw() {
    const N = +document.getElementById('fg-n').value;
    document.getElementById('fg-n-o').textContent = `${N}`;
    host.innerHTML = '';
    const W = 900, sp = (W - 120) / N;
    const X = (k) => 60 + k * sp;
    const nodes = [], edges = [];
    if (view === 'full') {
      for (let k = 0; k <= N; k++) {
        nodes.push({ id: 'x' + k, x: X(k), y: 165, shape: 'circle', r: 17, label: 'x' + sub(k), fill: k === 0 ? 'var(--ink)' : undefined });
        if (k > 0) {
          nodes.push({ id: 'c' + k, x: X(k), y: 250, shape: 'square', r: 9, fill: 'var(--state)', stroke: 'var(--state)' });
          edges.push({ from: 'x' + k, to: 'c' + k });
        }
      }
      for (let k = 0; k < N; k++) {
        nodes.push({ id: 'u' + k, x: X(k) + sp / 2, y: 80, shape: 'circle', r: 17, label: 'u' + sub(k) });
        nodes.push({ id: 'f' + k, x: X(k) + sp / 2, y: 165, shape: 'square', r: 7, fill: 'var(--ink-2)', stroke: 'var(--ink-2)' });
        nodes.push({ id: 'e' + k, x: X(k) + sp / 2, y: 22, shape: 'square', r: 7, fill: 'var(--reference)', stroke: 'var(--reference)' });
        edges.push({ from: 'x' + k, to: 'f' + k }, { from: 'f' + k, to: 'x' + (k + 1) }, { from: 'u' + k, to: 'f' + k }, { from: 'u' + k, to: 'e' + k });
        if (k > 0) {
          nodes.push({ id: 's' + k, x: X(k), y: 80, shape: 'square', r: 6, fill: 'var(--s7)', stroke: 'var(--s7)' });
          edges.push({ from: 'u' + (k - 1), to: 's' + k }, { from: 's' + k, to: 'u' + k });
        }
      }
    } else {
      for (let k = 0; k < N; k++) {
        nodes.push({ id: 'u' + k, x: X(k) + sp / 2, y: 80, shape: 'circle', r: 17, label: 'u' + sub(k) });
        nodes.push({ id: 'e' + k, x: X(k) + sp / 2, y: 22, shape: 'square', r: 7, fill: 'var(--reference)', stroke: 'var(--reference)' });
        edges.push({ from: 'u' + k, to: 'e' + k });
        if (k > 0) {
          nodes.push({ id: 's' + k, x: X(k), y: 80, shape: 'square', r: 6, fill: 'var(--s7)', stroke: 'var(--s7)' });
          edges.push({ from: 'u' + (k - 1), to: 's' + k }, { from: 's' + k, to: 'u' + k });
        }
      }
      for (let k = 1; k <= N; k++) {
        nodes.push({ id: 'c' + k, x: X(k), y: 250, shape: 'square', r: 9, fill: 'var(--state)', stroke: 'var(--state)' });
        for (let j = 0; j < k; j++) edges.push({ from: 'u' + j, to: 'c' + k, opacity: 0.5 });
      }
    }
    const g = new Graph(host, {
      width: W, height: 275, nodes, edges, arrows: false,
      info: (n) => {
        if (!n) return (note.innerHTML = '<span style="color:var(--state)">■</span> tracking cost ‖y<sub>k</sub>‖² · <span style="color:var(--reference)">■</span> effort · <span style="color:var(--s7)">■</span> smoothness · <span style="color:var(--ink-2)">■</span> dynamics x<sub>k+1</sub> = F(x<sub>k</sub>, u<sub>k</sub>) · filled x₀: fixed by the measurement and the delay roll-out');
        const id = n.id, k = +id.slice(1);
        if (id[0] === 'u') {
          // everything downstream of u_k
          const on = new Set(['u' + k, 'e' + k, 's' + k, 's' + (k + 1)]);
          for (let j = k; j < N; j++) on.add('f' + j);
          for (let j = k + 1; j <= N; j++) { on.add('x' + j); on.add('c' + j); }
          g.nodes.forEach((m, key) => m.g.classList.toggle('dim', !on.has(key)));
          g.edges.forEach((e) => {
            const hit = on.has(e.from) && on.has(e.to);
            e.path.classList.toggle('dim', !hit);
            e.path.classList.toggle('hot', hit);
          });
          note.innerHTML = `u${sub(k)} is the thrust over [${(k * 50 + 30)}, ${(k * 50 + 80)}] ms from now. It changes x${sub(k + 1)} and, through the chain, every later state: it appears in ${N - k} tracking cost${N - k > 1 ? 's' : ''}, its own effort term and two smoothness terms.`;
        } else if (id[0] === 'c') note.innerHTML = `Tracking cost at the end of stage ${k}: ‖y${sub(k)}‖² weighted by Q${k === N ? ', times 5 (terminal)' : ''}. Depends on u₀ … u${sub(k - 1)}.`;
        else if (id[0] === 'x') note.innerHTML = k === 0 ? 'x₀: the measured state rolled 30 ms through the commands already in flight. Not a decision.' : `x${sub(k)}: predicted state, a function of x₀ and u₀ … u${sub(k - 1)}.`;
        else if (id[0] === 'f') note.innerHTML = `Dynamics: 25 steps of the Fossen model with the thruster lag, u${sub(k)} held for 50 ms.`;
        else if (id[0] === 'e') note.innerHTML = `Effort: r‖u${sub(k)} − u${sub(k)}ᶠᶠ‖², deviation from the inverse-dynamics feedforward.`;
        else if (id[0] === 's') note.innerHTML = `Smoothness: r<sub>Δ</sub>‖Δ(u − uᶠᶠ)‖² between stages ${k - 1} and ${k}.`;
      },
    });
    g.highlight(null);
  }
  document.getElementById('fg-n').addEventListener('input', draw);
  document.querySelectorAll('[data-v]').forEach((b) => b.addEventListener('click', () => {
    view = b.dataset.v;
    document.querySelectorAll('[data-v]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
    draw();
  }));
  draw();
}

// ---------- feedforward on / off ----------
{
  const host = document.getElementById('ff-charts');
  const a = document.createElement('div'), b = document.createElement('div');
  host.append(a, b);
  const lag = new LineChart(a, { height: 230, xLabel: 'time (s)', yLabel: 'reference ahead of vehicle (cm)', unit: 'cm', digits: 2 });
  const dz = new LineChart(b, { height: 230, xLabel: 'time (s)', yLabel: 'depth error (cm)', unit: 'cm', digits: 2 });
  Promise.all([loadDemo('ff_on'), loadDemo('ff_off')]).then(([on, off]) => {
    lag.set([
      { name: 'with feedforward', color: 'var(--state)', values: series(on, (r) => 100 * (r.rx - r.x)) },
      { name: 'effort about zero', color: 'var(--reference)', values: series(off, (r) => 100 * (r.rx - r.x)) },
    ]);
    dz.set([
      { name: 'with feedforward', color: 'var(--state)', values: series(on, (r) => 100 * (r.z - r.rz)) },
      { name: 'effort about zero', color: 'var(--reference)', values: series(off, (r) => 100 * (r.z - r.rz)) },
    ], { yZero: true });
  });
}

// ---------- timeline ----------
{
  const host = document.getElementById('timeline');
  const W = 900, H = 170, t0 = -0.22, t1 = 1.62;
  const sx = (t) => 30 + ((t - t0) / (t1 - t0)) * (W - 50);
  const svg = el('svg', { class: 'chart', viewBox: `0 0 ${W} ${H}` }, host);
  // in-flight
  el('rect', { x: sx(0), y: 50, width: sx(0.03) - sx(0), height: 36, fill: 'var(--s4)', opacity: 0.35, rx: 2 }, svg);
  for (let k = 0; k < 30; k++) {
    const a = sx(0.03 + k * 0.05), b = sx(0.08 + k * 0.05);
    el('rect', { x: a + 1, y: 50, width: b - a - 2, height: 36, fill: k === 0 ? 'var(--accent)' : 'var(--prediction)', opacity: k === 0 ? 0.95 : 0.18 + 0.5 * (1 - k / 30), rx: 2 }, svg);
  }
  const tick = (t, label, y = 30) => {
    el('line', { x1: sx(t), x2: sx(t), y1: y + 6, y2: 104, stroke: 'var(--ink)', 'stroke-width': 1.2 }, svg);
    el('text', { class: 'label', x: sx(t), y, 'text-anchor': 'middle' }, svg).textContent = label;
  };
  el('line', { x1: sx(0), x2: sx(0), y1: 30, y2: 104, stroke: 'var(--ink)', 'stroke-width': 1.2 }, svg);
  el('text', { class: 'label', x: sx(0) - 6, y: 26, 'text-anchor': 'end' }, svg).textContent = 'tick k: measure x̂';
  el('line', { x1: sx(0.05), x2: sx(0.05), y1: 30, y2: 104, stroke: 'var(--ink)', 'stroke-width': 1.2 }, svg);
  el('text', { class: 'label', x: sx(0.05) + 6, y: 26 }, svg).textContent = 'tick k+1: plan again';
  el('line', { x1: sx(t0), x2: sx(t1), y1: 104, y2: 104, stroke: 'var(--rule)' }, svg);
  for (const t of [0, 0.5, 1, 1.5]) el('text', { class: 'label', x: sx(t), y: 120, 'text-anchor': 'middle' }, svg).textContent = `${t} s`;
  const lab = (x, y, t, anchor = 'start') => (el('text', { class: 'label', x, y, 'text-anchor': anchor }, svg).textContent = t);
  el('line', { x1: sx(0.015), x2: sx(0.015), y1: 88, y2: 142, stroke: 'var(--muted)', 'stroke-dasharray': '2 3' }, svg);
  lab(sx(0.015) + 6, 142, 'in flight: 30 ms of commands already sent');
  el('line', { x1: sx(0.055), x2: sx(0.055), y1: 88, y2: 162, stroke: 'var(--muted)', 'stroke-dasharray': '2 3' }, svg);
  lab(sx(0.055) + 6, 162, 'u₀: the only command published');
  lab(sx(0.9), 72, 'u₁ … u₂₉: the rest of the plan (30 stages, 1.5 s)', 'middle');
}

// ---------- receding horizon fan ----------
{
  const c = new LineChart(document.getElementById('fan'), { height: 320, xLabel: 'time (s)', yLabel: 'surge position x (m)', unit: 'm', digits: 3 });
  loadDemo('horizon').then((d) => {
    const preds = d.preds.filter((_, i) => i % 1 === 0).map((p, i) => ({ name: 'prediction', color: 'var(--prediction)', width: 1.2, opacity: 0.55, noTip: true, noLegend: i > 0, values: p.p.map((q, k) => ({ x: p.t + 0.03 + k * 0.05, y: q[0] })) }));
    c.set([
      ...preds,
      { name: 'reference', color: 'var(--reference)', dash: true, values: series(d, (r) => r.rx) },
      { name: 'Talos', color: 'var(--state)', values: series(d, (r) => r.x) },
    ], { xDomain: [0, 12], yDomain: [-0.05, 1.65], forceLegend: true });
  });
}
