import { setup, SRC } from '../layout.js';
import { createScene } from '../viz/scene.js';
import { Graph } from '../viz/graph.js';
import { thrusterNames } from '../engine/talos.js';

setup();

// ---------- hero: live closed loop ----------
const canvas = document.getElementById('hero-canvas');
const scene = createScene(canvas, {
  shiftRight: 0.17,
  onPick: (p) => {
    const depth = last ? (last.goal || last.refPosition)[2] : -1;
    worker.postMessage({ type: 'goal', position: [p[0], p[1], depth], rpy: att.map(deg) });
  },
});
const worker = new Worker(new URL('../worker/sim.worker.js', import.meta.url), { type: 'module' });
const send = worker.postMessage.bind(worker);
worker.postMessage = (msg) => {
  if (msg.type === 'goal' || msg.type === 'init') scene.setPlanned([]); // a point goal ends any path: drop its line
  send(msg);
};
let last = null, trail = [], rtfSamples = [], lastWall = performance.now(), lastSim = 0;
const bars = document.getElementById('hud-bars');
document.getElementById('hud-names').innerHTML = thrusterNames.map((n) => `<span>${n}</span>`).join('');
bars.innerHTML = thrusterNames.map(() => '<div><span></span></div>').join('');
const barEls = [...bars.querySelectorAll('span')];

worker.onmessage = ({ data: f }) => {
  last = f; window.__last = f;
  scene.setRobot(f.position, f.orientation);
  scene.setThrust(f.command);
  scene.setPrediction(f.prediction);
  scene.setReference(f.refPosition, f.refOrientation);
  if (f.planned) scene.setPlanned(f.planned);
  if (f.goal) scene.setGoal(f.goal);
  trail.push(f.position);
  if (trail.length > 600) trail.shift();
  scene.setTrail(trail);
  scene.follow(f.position);
  const now = performance.now();
  rtfSamples.push((f.t - lastSim) / ((now - lastWall) / 1000));
  if (rtfSamples.length > 20) rtfSamples.shift();
  lastWall = now;
  lastSim = f.t;
  barEls.forEach((b, i) => {
    const v = f.command[i] / 24;
    b.style.top = v >= 0 ? `${50 - v * 50}%` : '50%';
    b.style.height = `${Math.abs(v) * 50}%`;
    b.style.background = v >= 0 ? 'var(--ink)' : 'var(--ink-2)';
  });
};
setInterval(() => {
  if (!last) return;
  document.getElementById('hud-solve').textContent = `${last.solve_ms.toFixed(1)} ms · ${last.qp_iterations} QP it.`;
  const rtf = rtfSamples.reduce((a, b) => a + b, 0) / Math.max(1, rtfSamples.length);
  document.getElementById('hud-rtf').textContent = isFinite(rtf) ? `${Math.min(rtf, 1).toFixed(2)}× real time` : '–';
  const g = last.goal;
  document.getElementById('hud-dist').textContent = g ? `${Math.hypot(g[0] - last.position[0], g[1] - last.position[1], g[2] - last.position[2]).toFixed(2)} m` : 'holding';
}, 250);

// Goal attitude, in degrees (roll, pitch, yaw); the position half of the goal is kept separately.
const att = [0, 0, 0];
const attPanel = document.getElementById('attitude');
const attInputs = [...attPanel.querySelectorAll('input')];
const deg = (d) => (d * Math.PI) / 180;
const wrapDeg = (d) => ((((d + 180) % 360) + 360) % 360) - 180;
const showAttitude = () => attInputs.forEach((i, k) => { i.value = att[k]; i.nextElementSibling.textContent = `${Math.round(att[k])}°`; });
const goalPosition = () => (last.goal || last.refPosition).slice();
const sendAttitude = () => worker.postMessage({ type: 'goal', position: goalPosition(), rpy: att.map(deg) });
let pending = null;
attInputs.forEach((input) => input.addEventListener('input', () => {
  if (!last) return;
  att[+input.dataset.axis] = +input.value;
  showAttitude();
  if (!pending) pending = setTimeout(() => { pending = null; sendAttitude(); }, 120);
}));

// ---------- drawing a path ----------
// In draw mode a drag traces a line at the goal's depth; on release it is simplified
// to waypoints (Ramer-Douglas-Peucker) and flown as one planned path.
const drawBtn = document.querySelector('[data-act="draw"]');
let drawMode = false, stroke = null;
const setDraw = (on) => {
  drawMode = on;
  drawBtn.setAttribute('aria-pressed', String(on));
  drawBtn.textContent = on ? 'Drag on the pool…' : 'Draw a path';
  scene.setDrawing(on);
  if (!on) scene.setSketch([]);
};
function simplify(pts, tol) {
  if (pts.length < 3) return pts;
  const [a, b] = [pts[0], pts[pts.length - 1]];
  const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1e-9;
  let worst = 0, at = 0;
  for (let i = 1; i < pts.length - 1; i++) {
    const d = Math.abs((pts[i][0] - a[0]) * dy - (pts[i][1] - a[1]) * dx) / L;
    if (d > worst) { worst = d; at = i; }
  }
  if (worst < tol) return [a, b];
  return [...simplify(pts.slice(0, at + 1), tol).slice(0, -1), ...simplify(pts.slice(at), tol)];
}
canvas.addEventListener('pointerdown', (e) => {
  if (!drawMode || !last) return;
  const z = (last.goal || last.refPosition)[2];
  const p = scene.pointOnDepth(e.clientX, e.clientY, z);
  if (!p) return;
  canvas.setPointerCapture(e.pointerId);
  stroke = { z, pts: [p] };
});
canvas.addEventListener('pointermove', (e) => {
  if (!stroke) return;
  const p = scene.pointOnDepth(e.clientX, e.clientY, stroke.z);
  const q = stroke.pts[stroke.pts.length - 1];
  if (p && Math.hypot(p[0] - q[0], p[1] - q[1]) > 0.03) {
    stroke.pts.push(p);
    scene.setSketch(stroke.pts);
  }
});
canvas.addEventListener('pointerup', () => {
  if (!stroke) return;
  const s = stroke;
  stroke = null;
  setDraw(false);
  // Keep the shape, drop the jitter; waypoints closer than 25 cm would only make turns in place.
  const keep = simplify(s.pts, 0.06).filter((p, i, a) => i === 0 || Math.hypot(p[0] - a[i - 1][0], p[1] - a[i - 1][1]) > 0.25);
  const length = keep.reduce((sum, p, i) => (i ? sum + Math.hypot(p[0] - keep[i - 1][0], p[1] - keep[i - 1][1]) : 0), 0);
  if (keep.length < 2 && length < 0.3) return;
  trail = [];
  att[0] = att[1] = 0;
  showAttitude();
  worker.postMessage({ type: 'path', points: keep.map((p) => ({ position: [p[0], p[1], s.z], heading: 'PATH' })) });
});

document.querySelectorAll('[data-act]').forEach((b) =>
  b.addEventListener('click', () => {
    const act = b.dataset.act;
    if (act === 'draw') return setDraw(!drawMode);
    if (act === 'attitude') {
      attPanel.hidden = !attPanel.hidden;
      b.setAttribute('aria-expanded', String(!attPanel.hidden));
      return;
    }
    if (!last) return;
    const p = last.position;
    if (act === 'level') { att[0] = att[1] = 0; showAttitude(); sendAttitude(); }
    if (act === 'turn') { att[2] = wrapDeg(att[2] + 90); showAttitude(); sendAttitude(); }
    if (act === 'dive') worker.postMessage({ type: 'goal', position: goalPosition().map((v, i) => (i === 2 ? Math.max(-2.5, v - 0.5) : v)), rpy: att.map(deg) });
    if (act === 'rise') worker.postMessage({ type: 'goal', position: goalPosition().map((v, i) => (i === 2 ? Math.min(-0.5, v + 0.5) : v)), rpy: att.map(deg) });
    if (act === 'reset') { trail = []; scene.setPlanned([]); att.fill(0); showAttitude(); worker.postMessage({ type: 'init' }); }
    if (act === 'path') {
      // The path planner flies level (the port plans yaw only): it starts from the current heading and ends turned around.
      const yaw = deg(att[2]);
      att[0] = att[1] = 0;
      att[2] = wrapDeg(att[2] + 180);
      showAttitude();
      const c = Math.cos(yaw), s = Math.sin(yaw);
      const local = (x, y) => [p[0] + c * x - s * y, p[1] + s * x + c * y, p[2]];
      worker.postMessage({
        type: 'path',
        points: [
          { position: local(2.0, 0), heading: 'PATH' },
          { position: local(2.0, 1.6), heading: 'PATH' },
          { position: local(2.0, 1.6), shape: 'ARC', center: local(2.0, 2.4), sweep: Math.PI, heading: 'PATH' },
          { position: local(0, 3.2), heading: 'WAYPOINT', yaw: yaw + Math.PI },
        ],
      });
    }
  })
);

// ---------- the loop graph ----------
const info = document.getElementById('loop-info');
const blurb = {
  autonomy: ['Autonomy', 'The behaviour tree sends goals: a pose to hold (linear and angular ControllerCommand topics) or a FollowPath action with lines and arcs. It waits on the controller\'s trust signal before moving on.', 'src/mpc_controller_node.cpp'],
  reference: ['Reference generator', 'Turns a goal into a moving reference the vehicle can follow: a jerk-limited S-curve to a pose, or a speed profile along a planned path. The MPC tracks its pose, velocity and acceleration.', 'src/motion_profile.cpp'],
  mpc: ['MPC', 'Predicts 30 stages × 50 ms with the Fossen model, linearizes, and solves a 240-variable box-constrained QP for all eight thrusters. Publishes the first stage.', 'src/mpc.cpp'],
  thrusters: ['Thrusters', 'Eight Blue Robotics T200 thrusters. Force commands are scaled to the 73 N total budget and converted to RPM with the propeller law. Thrust arrives 30 ms late and spins up with an 80 ms lag.', 'src/thruster_dynamics.cpp'],
  talos: ['Talos', 'The plant: 34.3 kg, about 1.7 N positively buoyant, with the centre of buoyancy 22 mm above the centre of mass so it rights itself in roll and pitch.', 'config/models/talos.yaml'],
  sensors: ['Sensors', 'VectorNav IMU (rates and gravity direction), fibre-optic gyro for yaw rate, Nortek DVL for velocity (~125 ms old), pressure depth sensor.', 'src/state_estimator.cpp'],
  estimator: ['State estimator', 'Propagates the same Fossen model with a replica of the commanded thrust, corrects with each sensor, and learns the wrench the model is missing (offset-free MPC).', 'src/state_estimator.cpp'],
  ekf: ['EKF', 'robot_localization\'s EKF. The MPC uses it only to anchor x, y and heading slowly, and as a stand-in when a sensor goes stale. Its velocity lags the DVL by 0.2–0.35 s, too late to control with.', 'src/state_estimator.cpp'],
};
const g = new Graph(document.getElementById('loop-graph'), {
  width: 900,
  height: 370,
  nodes: [
    { id: 'autonomy', x: 90, y: 62, label: 'Autonomy', sub: 'behaviour tree' },
    { id: 'reference', x: 290, y: 62, label: 'Reference', sub: 'profile or path', w: 120 },
    { id: 'mpc', x: 500, y: 62, label: 'MPC', sub: 'QP every 50 ms', w: 124, bold: true },
    { id: 'thrusters', x: 705, y: 62, label: 'Thrusters', sub: '8 × force → RPM', w: 124 },
    { id: 'talos', x: 840, y: 190, label: 'Talos', sub: 'the plant', w: 96 },
    { id: 'sensors', x: 770, y: 315, label: 'Sensors', sub: 'IMU, FOG, DVL, depth', w: 140 },
    { id: 'estimator', x: 500, y: 315, label: 'Estimator', sub: 'model-based observer', w: 140 },
    { id: 'ekf', x: 210, y: 315, label: 'EKF', sub: 'robot_localization', w: 124 },
  ],
  edges: [
    { from: 'autonomy', to: 'reference', label: 'goal' },
    { from: 'reference', to: 'mpc', label: 'x_ref(t)' },
    { from: 'mpc', to: 'thrusters', label: 'u₀' },
    { from: 'thrusters', to: 'talos', label: 'thrust' },
    { from: 'talos', to: 'sensors' },
    { from: 'sensors', to: 'estimator', label: 'measurements' },
    { from: 'estimator', to: 'mpc', label: 'state x̂', bend: -50 },
    { from: 'estimator', to: 'mpc', label: 'disturbance d̂', bend: 50, dash: true },
    { from: 'ekf', to: 'estimator', label: 'x, y, yaw', dash: true },
    { from: 'sensors', to: 'ekf', bend: 95, dash: true, label: 'fused separately' },
    { from: 'mpc', to: 'autonomy', label: 'trust', bend: -80, dash: true },
  ],
  info: (n) => {
    const b = blurb[n ? n.id : 'mpc'];
    info.innerHTML = `<b style="font-size:1rem">${b[0]}</b><br>${b[1]} <a class="src" href="${SRC}/${b[2]}">${b[2]}</a>`;
  },
});
g.highlight(null);
g.pulse((e) => !e.dash, 2.0);
