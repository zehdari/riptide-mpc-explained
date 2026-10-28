import { setup } from '../layout.js';
import { LineChart, barChart } from '../viz/chart.js';
import { thrusterNames } from '../engine/talos.js';

setup();

fetch(`${import.meta.env.BASE_URL}data/ident_session.json`).then((r) => r.json()).then((d) => {
  // ---------- session timeline ----------
  const blockOf = (s) => (s.kind === 'hold' ? 'statics' : s.kind === 'ramp' ? 'ramps' : s.kind === 'null' ? 'null' : s.kind === 'release' ? 'releases' : { 0: 'surge', 1: 'sway', 2: 'heave', 5: 'yaw' }[s.axis] || s.kind);
  const bands = [], markers = [];
  let cur = null;
  for (const [t, id] of d.timeline) {
    const b = blockOf(d.segments[id]);
    if (!cur || cur.name !== b) {
      if (cur) cur.x1 = t;
      cur = { name: b, x0: t, x1: t };
      bands.push(cur);
    }
    cur.x1 = t;
  }
  bands.forEach((b, i) => { b.color = i % 2 ? 'rgba(15,38,51,0.035)' : 'rgba(15,38,51,0.09)'; markers.push({ x: b.x0, label: b.name }); });
  const tv = new LineChart(document.getElementById('tl-v'), { height: 240, xLabel: 'time (s)', yLabel: 'body velocity (m/s)', unit: 'm/s', digits: 3 });
  tv.set([
    { name: 'surge u', color: 'var(--state)', width: 1.4, values: d.timeline.map((r) => ({ x: r[0], y: r[2] })) },
    { name: 'sway v', color: 'var(--reference)', width: 1.4, values: d.timeline.map((r) => ({ x: r[0], y: r[3] })) },
    { name: 'heave w', color: 'var(--prediction)', width: 1.4, values: d.timeline.map((r) => ({ x: r[0], y: r[4] })) },
  ], { bands, markers, yDomain: [-0.65, 0.65] });
  const tw = new LineChart(document.getElementById('tl-w'), { height: 150, xLabel: 'time (s)', yLabel: 'yaw rate (rad/s)', unit: 'rad/s', digits: 3, legend: false });
  tw.set([{ name: 'yaw rate', color: 'var(--s7)', width: 1.4, values: d.timeline.map((r) => ({ x: r[0], y: r[5] })) }], { bands, yDomain: [-1.1, 1.1] });

  // ---------- statics ----------
  const s = d.statics, mm = (v) => v.map((x) => (x * 1000).toFixed(1)).join(', ');
  document.getElementById('statics-table').innerHTML = `<table class="params" style="margin:0"><thead><tr><th></th><th style="text-align:right">Fit</th><th style="text-align:right">Prior</th></tr></thead><tbody>
    <tr><td>Buoyant force B</td><td style="text-align:right">${s.net_buoyancy_N.toFixed(2)} N</td><td></td></tr>
    <tr><td>Weight W = mg</td><td style="text-align:right">${d.W.toFixed(2)} N</td><td style="text-align:right">trusted</td></tr>
    <tr><td>Net buoyancy B − W</td><td style="text-align:right">+${(s.net_buoyancy_N - d.W).toFixed(2)} N</td><td></td></tr>
    <tr><td>Displaced volume</td><td style="text-align:right">${(s.displaced_volume * 1000).toFixed(2)} L</td><td style="text-align:right">${(s.displaced_volume_prior * 1000).toFixed(2)} L</td></tr>
    <tr><td>COB relative to COM</td><td style="text-align:right">(${mm(s.cob_relative)}) mm</td><td style="text-align:right">(${mm(s.cob_relative_prior)}) mm</td></tr>
    <tr><td>Residual over holds</td><td style="text-align:right">${s.residual_force_rms_N.toFixed(2)} N, ${s.residual_torque_rms_Nm.toFixed(2)} N·m</td><td></td></tr>
  </tbody></table>`;

  // ---------- drag ----------
  const dragHost = document.getElementById('drag-charts');
  for (const [name, unit] of [['surge', 'm/s'], ['sway', 'm/s']]) {
    const a = d.axes[name];
    const div = document.createElement('div');
    dragHost.appendChild(div);
    const c = new LineChart(div, { height: 250, xLabel: `${name} speed (${unit})`, yLabel: `${name} drag (N)`, unit: 'N', digits: 1, xDigits: 3 });
    const vmax = Math.max(...a.samples.map((p) => Math.abs(p[0]))) * 1.1;
    const curve = (D1, D2) => Array.from({ length: 61 }, (_, k) => { const v = -vmax + (2 * vmax * k) / 60; return { x: v, y: D1 * v + D2 * Math.abs(v) * v }; });
    c.set([
      { name: 'cruise samples', color: 'var(--ink-2)', points: true, noTip: true, values: a.samples.map((p) => ({ x: p[0], y: p[1] })) },
      { name: `fit (R² ${a.drag_r2.toFixed(3)})`, color: 'var(--state)', values: curve(a.D1, a.D2) },
      { name: 'prior', color: 'var(--reference)', dash: true, width: 1.5, values: curve(a.D1_prior, a.D2_prior) },
    ], { yZero: true });
  }

  // ---------- lag check ----------
  const lagHost = document.getElementById('lag-charts');
  const mDiv = document.createElement('div'), rDiv = document.createElement('div');
  lagHost.append(mDiv, rDiv);
  const cm = new LineChart(mDiv, { height: 240, xLabel: 'extra thrust delay (s)', yLabel: 'fitted total mass (kg)', unit: 'kg', digits: 1, xDigits: 2 });
  const cr = new LineChart(rDiv, { height: 240, xLabel: 'extra thrust delay (s)', yLabel: 'fit quality R²', digits: 3, xDigits: 2 });
  const lagSeries = (key) => [
    { name: 'surge', color: 'var(--state)', values: d.axes.surge.lag.map((p) => ({ x: p.lag, y: p[key] })) },
    { name: 'sway', color: 'var(--reference)', values: d.axes.sway.lag.map((p) => ({ x: p.lag, y: p[key] })) },
  ];
  cm.set([...lagSeries('M'), { name: 'old model, surge (66 kg)', color: 'var(--ink-2)', dash: true, width: 1.5, noTip: true, values: [{ x: 0, y: 66.2 }, { x: 0.6, y: 66.2 }] }], { yDomain: [40, 170] });
  cr.set(lagSeries('r2'), { yDomain: [0.84, 1] });

  // ---------- gains ----------
  const gHost = document.getElementById('gain-charts');
  const g = d.thrusters;
  for (const [key, label] of [['forward_gain', 'forward'], ['reverse_gain', 'reverse']]) {
    const div = document.createElement('div');
    div.innerHTML = `<div style="font-family:var(--sans);font-size:.86rem;color:var(--ink-2);margin-bottom:.3rem">${label} gain − 1</div><div></div>`;
    gHost.appendChild(div);
    barChart(div.lastChild, thrusterNames.map((n, i) => ({ label: n, segments: [{ value: g[key][i] - 1, color: g.group[i] === 0 ? 'var(--state)' : 'var(--prediction)' }], total: g[key][i], unit: '' })), { digits: 3, max: 0.2, rowH: 22, width: 380, labelW: 48, valueW: 52 });
  }
  const leg = document.createElement('div');
  leg.className = 'legend';
  leg.innerHTML = '<span><i style="background:var(--state);height:8px"></i>forward thrusters (VUS, VUP, VLS, VLP)</span><span><i style="background:var(--prediction);height:8px"></i>angled thrusters</span>';
  gHost.parentElement.appendChild(leg);

  // ---------- results ----------
  const rows = [
    ['surge', 'N·s/m', 'kg'], ['sway', 'N·s/m', 'kg'], ['yaw', 'N·m·s/rad', 'kg·m²'],
  ];
  const fmt = (v, n = 2) => (v === undefined || v === null ? '—' : (+v).toFixed(n));
  const body = rows.map(([n]) => {
    const a = d.axes[n];
    return `<tr><td>${n}</td><td style="text-align:right">${fmt(a.D1)} / ${fmt(a.D2, 1)}</td><td style="text-align:right">${fmt(a.drag_r2, 3)}</td><td style="text-align:right">${fmt(a.added, 1)} (total ${fmt(a.M, 1)})</td><td style="text-align:right">${fmt(a.inertia_r2, 3)}</td><td>accepted</td></tr>`;
  }).join('') + ['heave', 'roll', 'pitch'].map((n) => `<tr><td>${n}</td><td colspan="4" style="color:var(--ink-2)">${d.axes[n].note || ''}</td><td>rejected: prior kept</td></tr>`).join('');
  document.getElementById('results-table').innerHTML = `<table class="params" style="margin:0"><thead><tr><th>Axis</th><th style="text-align:right">D₁ / D₂</th><th style="text-align:right">drag R²</th><th style="text-align:right">added mass</th><th style="text-align:right">inertia R²</th><th>result</th></tr></thead><tbody>${body}</tbody></table>`;
});
