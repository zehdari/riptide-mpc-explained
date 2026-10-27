// Closed loop as mpc_controller_node runs it, on a simulated plant:
//   every 50 ms: plant moves (2 ms RK4 steps, real thruster lag), controller.advance(dt),
//   observer learns the missing wrench, compute(), issue() to both the replica and the plant.
import { FossenModel, ThrusterReplica, plantStep } from './model.js';
import { MpcController, Mode } from './mpc.js';
import { hydro, estimatorSettings } from './talos.js';
import { qrot, qrotInv, yawQuat, clamp, norm } from './math.js';

// The offset-free part of riptide_mpc's StateEstimator: velocity innovations the
// model leaves behind are read as a missing wrench, learned with a time constant.
export class DisturbanceObserver {
  constructor(model, settings = estimatorSettings) {
    this.model = model; // a model copy whose disturbance this owns
    this.s = settings;
    this.actuator = new ThrusterReplica(model);
    this.x = null;
    this.enabled = true;
  }
  reset(x) {
    this.x = x.slice();
    this.model.disturbance = [0, 0, 0, 0, 0, 0];
  }
  gate() {
    const f = (v, s) => (s > 0 ? 1 / (1 + (v / s) * (v / s)) : 1);
    return f(norm(this.model.baseLinkVelocity(this.x)), this.s.disturbance_gate_speed) * f(norm(this.x.slice(10)), this.s.disturbance_gate_rate);
  }
  // Propagate dt with the model, then correct velocity/rates from the "DVL" and "gyro" (plant truth).
  update(truth, dt) {
    if (!this.x) this.reset(truth);
    const steps = Math.max(1, Math.round(dt / 0.005));
    for (let i = 0; i < steps; i++) this.x = plantStep(this.model, this.x, this.actuator, dt / steps);
    const q = [this.x[3], this.x[4], this.x[5], this.x[6]];
    const ev = [truth[7] - this.x[7], truth[8] - this.x[8], truth[9] - this.x[9]];
    const ew = [truth[10] - this.x[10], truth[11] - this.x[11], truth[12] - this.x[12]];
    if (this.enabled) {
      const M = this.model.M, d = this.model.disturbance, gate = this.gate();
      const F = [0, 1, 2].map((r) => M[r][0] * ev[0] + M[r][1] * ev[1] + M[r][2] * ev[2]);
      const Fw = qrot(q, F);
      for (let i = 0; i < 3; i++) d[i] += (gate * Fw[i]) / this.s.disturbance_force_time_constant;
      const Tq = [3, 4, 5].map((r) => M[r][3] * ew[0] + M[r][4] * ew[1] + M[r][5] * ew[2]);
      for (let i = 0; i < 3; i++) d[3 + i] += (gate * Tq[i]) / this.s.disturbance_torque_time_constant;
      const fn = Math.hypot(d[0], d[1], d[2]), tn = Math.hypot(d[3], d[4], d[5]);
      if (fn > this.s.max_disturbance_force) for (let i = 0; i < 3; i++) d[i] *= this.s.max_disturbance_force / fn;
      if (tn > this.s.max_disturbance_torque) for (let i = 3; i < 6; i++) d[i] *= this.s.max_disturbance_torque / tn;
    }
    // Velocity follows the DVL (gain 0.7) and the gyro (gain 1); the pose follows the anchors.
    for (let i = 0; i < 3; i++) this.x[7 + i] += this.s.dvl_gain * ev[i];
    for (let i = 0; i < 3; i++) this.x[10 + i] += this.s.gyro_gain * ew[i];
    for (let i = 0; i < 7; i++) this.x[i] = truth[i];
    return this.model.disturbance;
  }
}

export function makePlantHydro({ buoyancy = 0, cobShift = [0, 0, 0], dragScale = 1 } = {}) {
  // buoyancy: extra net buoyancy [N] the controller's model does not know about; dragScale: plant drag / model drag.
  const extraVolume = buoyancy / (hydro.water_density * 9.80665);
  return {
    ...hydro,
    displaced_volume: hydro.displaced_volume + extraVolume,
    cob: hydro.cob.map((c, i) => c + cobShift[i]),
    linear_damping: hydro.linear_damping.map((d) => d * dragScale),
    quadratic_damping: hydro.quadratic_damping.map((d) => d * dragScale),
  };
}

export class Simulation {
  constructor({ settings = {}, plantHydro = hydro, depth = -1.0, learn = true } = {}) {
    this.model = new FossenModel();
    this.plantModel = new FossenModel(plantHydro);
    this.controller = new MpcController(this.model, settings);
    this.observer = new DisturbanceObserver(new FossenModel());
    this.observer.enabled = learn;
    this.plantActuator = new ThrusterReplica(this.plantModel);
    this.t = 0;
    this.reset(depth);
  }
  reset(depth = -1.0, yaw = 0) {
    this.x = this.model.fromBaseLink([0, 0, depth], yawQuat(yaw), [0, 0, 0], [0, 0, 0]);
    this.plantActuator = new ThrusterReplica(this.plantModel);
    this.controller.actuator = new ThrusterReplica(this.model);
    this.controller.clearWarmStart();
    this.observer.actuator = new ThrusterReplica(this.observer.model);
    this.observer.reset(this.x);
    this.t = 0;
    this.last = null;
  }
  holdHere() {
    const p = this.model.baseLinkPosition(this.x);
    const q = [this.x[3], this.x[4], this.x[5], this.x[6]];
    return { linear_mode: Mode.POSITION, angular_mode: Mode.POSITION, position: p, orientation: q, path: null };
  }
  tick(reference) {
    const dt = this.controller.s.dt;
    const steps = Math.round(dt / 0.002);
    for (let i = 0; i < steps; i++) this.x = plantStep(this.plantModel, this.x, this.plantActuator, dt / steps);
    this.t += dt;
    this.controller.advance(dt);
    const d = this.observer.update(this.x, dt);
    this.controller.model.disturbance = d.slice();
    const out = this.controller.compute(this.x, reference);
    this.controller.issue(out.command);
    this.observer.actuator.command(out.command);
    this.plantActuator.command(out.command);
    this.last = out;
    return out;
  }
  basePose() {
    return { position: this.model.baseLinkPosition(this.x), orientation: [this.x[3], this.x[4], this.x[5], this.x[6]] };
  }
}

export { Mode, qrotInv, clamp };
