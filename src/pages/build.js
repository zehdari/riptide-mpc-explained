import { setup } from '../layout.js';
import { LineChart } from '../viz/chart.js';
import { loadTrace } from './demo.js';

setup();

{
  const host = document.getElementById('nereus-charts');
  const mk = (yLabel, unit) => {
    const d = document.createElement('div');
    host.appendChild(d);
    return new LineChart(d, { height: 210, xLabel: 'time (s)', yLabel, unit, digits: 3 });
  };
  const cs = mk('surge progress (m)', 'm'), cy = mk('yaw change (°)', '°'), cd = mk('depth (m)', 'm');
  const pts = (d, f, arr) => d.t.map((t, i) => ({ x: t, y: arr[i] && arr[i][0] !== null ? f(arr[i]) : null })).filter((p) => p.y !== null && t0(d) <= p.x);
  const t0 = (d) => Math.max(0, d.commands[0].t - 1);
  loadTrace('surge_step').then((d) => {
    const i0 = d.pos.findIndex((p) => p && p[0] !== null);
    const s = d.pos[i0], g = d.commands[0].vect, dir = [g[0] - s[0], g[1] - s[1]], L = Math.hypot(...dir);
    const f = (p) => ((p[0] - s[0]) * dir[0] + (p[1] - s[1]) * dir[1]) / L;
    cs.set([{ name: 'reference', color: 'var(--reference)', dash: true, values: pts(d, f, d.ref_pos) }, { name: 'Talos', color: 'var(--state)', values: pts(d, f, d.pos) }]);
  });
  loadTrace('yaw_step').then((d) => {
    const i0 = d.rpy.findIndex((p) => p && p[0] !== null);
    const y0 = d.rpy[i0][2];
    const f = (r) => ((r[2] - y0) * 180) / Math.PI;
    cy.set([{ name: 'reference', color: 'var(--reference)', dash: true, values: pts(d, f, d.ref_rpy) }, { name: 'Talos', color: 'var(--state)', values: pts(d, f, d.rpy) }]);
  });
  loadTrace('depth_step').then((d) => {
    const f = (p) => p[2];
    cd.set([{ name: 'reference', color: 'var(--reference)', dash: true, values: pts(d, f, d.ref_pos) }, { name: 'Talos', color: 'var(--state)', values: pts(d, f, d.pos) }]);
  });
}

{
  const g = document.getElementById('gallery');
  const base = import.meta.env.BASE_URL + 'img/';
  const shots = [
    ['nereus_path_overview', 'The whole path from above'],
    ['nereus_follow_line', 'Following the first line'],
    ['nereus_path_corner', 'Through the rounded corner'],
    ['nereus_follow_arc', 'On the 180° arc'],
  ];
  g.innerHTML = shots.map(([f, alt]) => `<a href="${base}${f}.png"><img src="${base}${f}.webp" alt="Nereus viewer: ${alt}" loading="lazy" style="width:100%;display:block;aspect-ratio:1400/780;object-fit:cover"></a>`).join('');
}
