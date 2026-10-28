// Runs the closed loop (plant + MPC) off the main thread and streams frames.
import { Simulation, makePlantHydro } from '../engine/sim.js';
import { Mode } from '../engine/mpc.js';
import { PathPlan } from '../engine/path.js';
import { pathOptions } from '../engine/talos.js';
import { yawQuat, twist, rpyQuat } from '../engine/math.js';

let sim, reference, running = true, wall0 = 0, sim0 = 0, speed = 1, plannedPoints = null, goal = null;

function init(opts = {}) {
  sim = new Simulation({
    settings: { model_step: 0.01, ...(opts.settings || {}) },
    plantHydro: makePlantHydro(opts.plant || {}),
    learn: opts.learn ?? true,
  });
  sim.reset(opts.depth ?? -1.0, opts.yaw ?? 0);
  reference = sim.holdHere();
  plannedPoints = null;
  goal = null;
  wall0 = performance.now();
  sim0 = 0;
}

function frame(out) {
  const m = sim.model;
  const pose = sim.basePose();
  const pred = out.prediction.map((x) => m.baseLinkPosition(x));
  const live = sim.controller.live.pose;
  return {
    t: sim.t,
    position: pose.position,
    orientation: pose.orientation,
    velocity: m.baseLinkVelocity(sim.x),
    rates: sim.x.slice(10),
    command: out.command,
    wrench: out.wrench,
    solve_ms: out.solve_ms,
    qp_iterations: out.qp_iterations,
    prediction: pred,
    refPosition: live.position,
    refOrientation: live.orientation,
    refVelocity: live.velocity,
    disturbance: sim.controller.model.disturbance.slice(),
    governor: sim.controller.governor ?? 1,
    progress: reference.path ? { s: sim.controller.live.progress.s, length: reference.path.length } : null,
    planned: plannedPoints,
    goal,
  };
}

function loop() {
  if (running && sim) {
    const start = performance.now();
    const wallT = ((start - wall0) / 1000) * speed;
    let ticks = 0, out = null;
    while (sim.t - sim0 < wallT && performance.now() - start < 30) {
      out = sim.tick(reference);
      ticks++;
    }
    if (sim.t - sim0 < wallT - 0.5) { // can't keep up: let simulated time fall behind gracefully
      wall0 = performance.now() - ((sim.t - sim0) / speed) * 1000;
    }
    if (out) {
      const f = frame(out);
      f.realtime = ticks;
      postMessage(f);
      plannedPoints = null;
    }
  }
  setTimeout(loop, 8);
}

onmessage = ({ data }) => {
  if (data.type === 'init') init(data);
  else if (data.type === 'goal') {
    const r = sim.holdHere();
    reference = {
      linear_mode: Mode.POSITION,
      angular_mode: Mode.POSITION,
      position: data.position,
      orientation: data.rpy ? rpyQuat(...data.rpy) : data.yaw === undefined ? reference.orientation : yawQuat(data.yaw),
      path: null,
    };
    goal = data.position;
    plannedPoints = null;
    void r;
  } else if (data.type === 'path') {
    const pose = sim.controller.live.pose;
    const start = { position: pose.position.slice(), yaw: twist(pose.orientation) };
    const plan = PathPlan.build(start, data.points, sim.controller.s.motion, { ...pathOptions, ...(data.options || {}) });
    const end = data.points[data.points.length - 1];
    reference = { linear_mode: Mode.POSITION, angular_mode: Mode.POSITION, position: plan.position(plan.length), orientation: plan.orientation(plan.length), path: plan };
    plannedPoints = plan.polyline(0.03);
    goal = end.position;
  } else if (data.type === 'running') {
    running = data.on;
    wall0 = performance.now();
    sim0 = sim.t;
  } else if (data.type === 'speed') {
    speed = data.speed;
    wall0 = performance.now();
    sim0 = sim.t;
  } else if (data.type === 'learn') {
    sim.observer.enabled = data.on;
    if (!data.on) sim.observer.model.disturbance = [0, 0, 0, 0, 0, 0];
  } else if (data.type === 'settings') {
    Object.assign(sim.controller.s, data.settings);
    if (data.settings.motion) sim.controller.s.motion = { ...sim.controller.s.motion, ...data.settings.motion };
    if (data.settings.horizon) sim.controller.setHorizon(data.settings.horizon);
  }
};

init();
loop();
