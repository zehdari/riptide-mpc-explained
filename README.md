# How Talos flies

An interactive explainer for [`riptide_mpc`](https://github.com/osu-uwrt/riptide_control/tree/mpc/riptide_mpc), the model predictive controller that flies the Ohio State Underwater Robotics Team's AUV Talos. Live at **https://zehdari.github.io/riptide-mpc-explained/**.

Chapters: the Fossen model, thrusters and allocation, S-curve profiles and paths, the optimal control problem, the solver (condensed Gauss–Newton QP, projected Newton, real-time iteration), state estimation and offset-free MPC, and a recipe for building your own.

## How it works

`src/engine/` is a JavaScript port of the controller's C++ (`marine_dynamics`, `fossen_model`, `thruster_dynamics`, `motion_profile`, `path_plan`, `mpc`, `box_qp`) with Talos's identified model and `config/mpc.yaml` settings. The hero runs the closed loop in a Web Worker; the charts use either live runs of the port, runs precomputed by `tools/precompute.mjs`, or traces of the real C++ controller recorded in the [Nereus](https://github.com/osu-uwrt/nereus) simulator (`tools/drive_maneuvers.py`, `tools/bag_to_json.py`). The Talos model is the Nereus mesh, decimated with `tools/decimate_talos.py`.

```sh
npm ci
npm run dev        # http://localhost:5173/riptide-mpc-explained/
npm test           # closed-loop test of the port
npm run build      # static site in dist/
node tools/precompute.mjs   # regenerate public/data/demo_*.json
```

Pushes to `main` deploy to GitHub Pages via `.github/workflows/pages.yml`.
