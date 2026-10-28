// Talos, as riptide_mpc loads it: riptide_descriptions/config/talos.yaml (thruster
// poses, base_link) + riptide_mpc/config/models/talos.yaml (body and hydrodynamics),
// + config/mpc.yaml (controller settings). Values copied from the mpc branch, 2026-10-27,
// except surge added mass: talos.yaml still has 31.9 kg, which identification had been
// rejecting under the old 3x rigid-mass limit. The value here is the re-fit of the
// 2026-10-26 untethered session (talos_20261026_204748) with the per-axis limit:
// 123.8 kg added, 158.1 kg total, R^2 0.97, no extra thrust lag.

export const vehicle = {
  base_link: [-0.14, 0.03, -0.09], // vehicle config frame (origin)
  thrusters: [
    { name: 'VUS', pose: [-0.134, -0.369, 0.242, 0.0, 0.0, 0.0] },
    { name: 'VUP', pose: [-0.134, 0.438, 0.242, 0.0, 0.0, 0.0] },
    { name: 'HUS', pose: [-0.14, -0.26, 0.141, 0.0, -0.785, 1.571] },
    { name: 'HUP', pose: [-0.14, 0.328, 0.141, 0.0, -0.785, -1.571] },
    { name: 'HLS', pose: [-0.14, -0.26, -0.141, 0.0, 0.785, 1.571] },
    { name: 'HLP', pose: [-0.14, 0.328, -0.141, 0.0, 0.785, -1.571] },
    { name: 'VLS', pose: [-0.134, -0.372, -0.244, 0.0, 0.0, 0.0] },
    { name: 'VLP', pose: [-0.134, 0.44, -0.244, 0.0, 0.0, 0.0] },
  ],
  dvl: [-0.119, 0.034, -0.25],
  imu: [-0.25, -0.1492, 0.0222],
  depth: [-0.305, -0.125, 0.045],
};

export const hydro = {
  mass: 34.307,
  com: [-0.15, 0.041, -0.016],
  inertia: [
    [1.8429, 0.0496, 0.0945],
    [0.0496, 1.0051, -0.0083],
    [0.0945, -0.0083, 1.8607],
  ],
  added_mass: [123.7676, 79.8831, 74.1148, 0.0624, 0.0602, 2.0867], // diagonal
  linear_damping: [15.6745, 46.2026, 12.0607, 9.9109, 3.0046, 1.9477], // diagonal
  quadratic_damping: [223.6306, 153.789, 266.4236, 2.7724, 1.4992, 4.9627],
  water_density: 998.2,
  displaced_volume: 0.034535723698578415,
  cob: [0.008184, -0.002308, 0.022498], // relative to COM
  buoyancy_radii: [0.175, 0.415, 0.275],
  water_level: 0,
  thruster: {
    delay: 0.03,
    rise: 0.08,
    fall: 0.06,
    slew: 300,
    forward_max: 24,
    reverse_max: 24,
    propeller_radius: 0.05,
  },
  total_thrust_limit: 73,
  // |F| = K_T rho D^4 (rpm/60)^2, K_T = a + b rpm
  kt_forward: [0.3964, 1.48e-5],
  kt_reverse: [0.3114, 1.25e-5],
  propeller_diameter: 0.076,
};

export const settings = {
  dt: 0.05,
  horizon: 30,
  model_step: 0.002, // what riptide_mpc uses; the browser default is coarser (see mpc.js)
  linearization_step: 0.025,
  sqp_iterations: 1,
  qp_max_iterations: 50,
  compensate_delay: true,
  motion: {
    linear_speed: 0.6,
    linear_accel: 0.1,
    linear_jerk: 0.3,
    linear_speed_vertical: 0.3,
    linear_accel_vertical: 0.2,
    linear_jerk_vertical: -1,
    angular_speed: 1.0,
    angular_accel: 0.8,
    angular_jerk: 3.0,
  },
  max_reference_lag: 1.0,
  max_reference_lag_angle: 0.8,
  governor_lag: 0.15,
  governor_stop_lag: 0.4,
  governor_angle: 0.05,
  governor_stop_angle: 0.25,
  q_position: [400, 400, 400],
  q_attitude: [3000, 3000, 300],
  q_linear_velocity: [100, 100, 100],
  q_angular_velocity: [50, 50, 50],
  q_linear_damping: [20, 20, 20],
  q_angular_damping: [10, 10, 10],
  terminal_factor: 5,
  r_thrust: 0.001,
  r_thrust_rate: 0.05,
};

export const pathOptions = {
  corner_radius: 0.3,
  lateral_accel: 0.3,
  kink_speed: 0.03,
  turn_drift: 0.05,
  heading_blend: 0.3,
};

export const estimatorSettings = {
  dvl_gain: 0.7,
  gyro_gain: 1.0,
  disturbance_force_time_constant: 3.0,
  disturbance_torque_time_constant: 3.0,
  max_disturbance_force: 20,
  max_disturbance_torque: 5,
  disturbance_gate_speed: 0.1,
  disturbance_gate_rate: 0.2,
};

export const thrusterNames = vehicle.thrusters.map((t) => t.name);
