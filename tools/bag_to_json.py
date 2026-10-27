#!/usr/bin/env python3
"""Convert a riptide_mpc maneuver bag (recorded by drive_maneuvers.py) into a compact JSON trace.

    python3 bag_to_json.py /tmp/mpcbags/path ../public/data/path.json [--name path] [--date 2026-10-27]

Output (floats rounded to 4 decimals, time in s from the bag start, resampled at 20 Hz by zero-order hold
on the receive time):
    t                 sample times
    pos, rpy          simulator ground-truth base_link position (world = odom here) and roll/pitch/yaw (rad,
                      yaw unwrapped)
    est_pos, est_rpy  EKF estimate (odometry/filtered), the frame the MPC works in
    vel               ground-truth body velocity [u, v, w, p, q, r]
    ref_pos, ref_rpy  controller/mpc/reference (the profiled reference the MPC tracks right now)
    thrust            the 8 commanded thruster forces (N), order VUS VUP HUS HUP HLS HLP VLS VLP
    thrust_actual     the simulator's delayed/rate-limited thruster forces (N)
    disturbance       learned disturbance wrench [Fx Fy Fz (odom frame, N), Tx Ty Tz (body, N m)]
    wrench            controller_debug_wrench [Fx Fy Fz Tx Ty Tz]
    solve_ms, trust   solve time per MPC tick and the settle trust (0..1)
    predicted         every 0.5 s: {t, pts: [[x,y,z], ...]} the MPC's predicted base_link path
    planned           the planned path (controller/mpc/planned_path) once: {t, pts}
    commands          controller/linear and controller/angular setpoints seen in the bag
    stats             summary numbers (settling, overshoot, peak thrust, solve times, path tracking)
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

NS = "/talos/"
RATE = 20.0
PRED_EVERY = 0.5
HORIZON_DT = 0.05  # control_period: one predicted point per MPC step


def rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def read(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {n: get_message(t) for n, t in types.items()}
    out = {n: [] for n in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        out[topic].append((stamp * 1e-9, deserialize_message(data, classes[topic])))
    return out, types


def r4(v):
    if isinstance(v, (list, tuple, np.ndarray)):
        return [r4(x) for x in v]
    if v is None or not math.isfinite(v):
        return None
    return round(float(v), 4)


def sample(series, grid, fn, width):
    """Zero-order hold of fn(msg) on grid; None before the first message."""
    times = [s[0] for s in series]
    vals = [fn(s[1]) for s in series]
    res = []
    for t in grid:
        i = bisect.bisect_right(times, t) - 1
        res.append(vals[i] if i >= 0 else [None] * width)
    return res


def unwrap_col(rows, col):
    last, offset = None, 0.0
    for r in rows:
        v = r[col]
        if v is None:
            continue
        if last is not None:
            d = v + offset - last
            offset -= 2 * math.pi * round(d / (2 * math.pi))
        r[col] = v + offset
        last = r[col]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--name")
    ap.add_argument("--date", default="")
    args = ap.parse_args()
    name = args.name or args.bag.name
    msgs, types = read(args.bag)
    g = lambda t: msgs.get(NS + t, [])  # noqa: E731

    truth, est = g("simulator/ground_truth"), g("odometry/filtered")
    t0 = min(s[0] for s in truth + est)
    t1 = max(s[0] for s in truth + est)
    grid = list(np.arange(t0, t1, 1.0 / RATE))
    rel = lambda t: t - t0  # noqa: E731

    pos = lambda m: [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]  # noqa: E731
    orient = lambda m: rpy(m.pose.pose.orientation)  # noqa: E731
    twist = lambda m: [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z,  # noqa: E731
                       m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z]
    floats = lambda m: list(m.data)  # noqa: E731
    wrench6 = lambda w: [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]  # noqa: E731
    tw6 = lambda m: [m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z]  # noqa: E731

    d = {
        "t": [rel(t) for t in grid],
        "pos": sample(truth, grid, pos, 3),
        "rpy": sample(truth, grid, orient, 3),
        "est_pos": sample(est, grid, pos, 3),
        "est_rpy": sample(est, grid, orient, 3),
        "vel": sample(truth, grid, twist, 6),
        "ref_pos": sample(g("controller/mpc/reference"), grid,
                          lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z], 3),
        "ref_rpy": sample(g("controller/mpc/reference"), grid, lambda m: rpy(m.pose.orientation), 3),
        "thrust": sample(g("thruster_forces"), grid, floats, 8),
        "thrust_actual": sample(g("simulator/actual_thruster_forces"), grid, floats, 8),
        "disturbance": sample(g("controller/mpc/disturbance"), grid, lambda m: wrench6(m.wrench), 6),
        "wrench": sample(g("controller_debug_wrench"), grid, tw6, 6),
        "solve_ms": [v[0] for v in sample(g("controller/mpc/solve_time_ms"), grid, lambda m: [m.data], 1)],
        "trust": [v[0] for v in sample(g("controller/scale/trust"), grid, lambda m: [m.linear.x], 1)],
    }
    for key in ("rpy", "est_rpy", "ref_rpy"):
        unwrap_col(d[key], 2)

    # Predicted-path snapshots every PRED_EVERY s, and the last planned path.
    pred, nxt = [], t0
    for t, m in g("controller/mpc/predicted_path"):
        if t >= nxt and m.poses:
            pred.append({"t": r4(rel(t)), "pts": [r4([p.pose.position.x, p.pose.position.y, p.pose.position.z])
                                                  for p in m.poses]})
            nxt = t + PRED_EVERY - 1e-3
    planned = None
    pp = [s for s in g("controller/mpc/planned_path") if s[1].poses]
    if pp:
        t, m = pp[-1]
        planned = {"t": r4(max(0.0, rel(t))), "frame": m.header.frame_id,
                   "pts": [r4([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for p in m.poses]}

    commands = []
    for kind in ("linear", "angular"):
        for t, m in g("controller/" + kind):
            commands.append({"t": r4(rel(t)), "axis": kind, "mode": m.mode,
                             "vect": r4([m.setpoint_vect.x, m.setpoint_vect.y, m.setpoint_vect.z]),
                             "quat_xyzw": r4([m.setpoint_quat.x, m.setpoint_quat.y, m.setpoint_quat.z,
                                              m.setpoint_quat.w])})
    commands.sort(key=lambda c: c["t"])

    stats = compute_stats(name, d, commands, planned, pred)

    out = {
        "meta": {
            "name": name,
            "date": args.date,
            "model": "sim",
            "source": "Nereus simulator (talos_uwrt, RoboSub 2026 pool), riptide_mpc with mpc_model:=sim",
            "rate_hz": RATE,
            "predicted_every_s": PRED_EVERY,
            "predicted_point_dt_s": HORIZON_DT,
            "frame": "odom (identical to world in this run)",
            "thruster_order": ["VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"],
            "fields": {
                "pos/rpy": NS + "simulator/ground_truth",
                "est_pos/est_rpy": NS + "odometry/filtered",
                "vel": NS + "simulator/ground_truth (twist, body frame)",
                "ref_pos/ref_rpy": NS + "controller/mpc/reference",
                "thrust": NS + "thruster_forces",
                "thrust_actual": NS + "simulator/actual_thruster_forces",
                "disturbance": NS + "controller/mpc/disturbance",
                "wrench": NS + "controller_debug_wrench",
                "solve_ms": NS + "controller/mpc/solve_time_ms",
                "trust": NS + "controller/scale/trust",
                "predicted": NS + "controller/mpc/predicted_path",
                "planned": NS + "controller/mpc/planned_path",
                "commands": NS + "controller/linear, " + NS + "controller/angular",
            },
            "units": "m, rad, m/s, rad/s, N, N m, ms",
            "duration_s": r4(t1 - t0),
        },
        "stats": stats,
        **{k: r4(v) for k, v in d.items()},
        "predicted": pred,
        "planned": planned,
        "commands": commands,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, separators=(",", ":")))
    print(f"{args.out}: {args.out.stat().st_size / 1e3:.0f} kB, {len(grid)} samples, {len(pred)} predictions")
    print(json.dumps(stats, indent=1))


def compute_stats(name, d, commands, planned, pred):
    t = np.array(d["t"])
    P = np.array([p if p[0] is not None else [np.nan] * 3 for p in d["est_pos"]], float)
    Y = np.array([r[2] if r[2] is not None else np.nan for r in d["est_rpy"]], float)
    T = np.array([r if r[0] is not None else [np.nan] * 8 for r in d["thrust"]], float)
    S = np.array([v if v is not None else np.nan for v in d["solve_ms"]], float)
    s = {
        "solve_ms_median": r4(np.nanmedian(S)),
        "solve_ms_p95": r4(np.nanpercentile(S, 95)),
        "solve_ms_max": r4(np.nanmax(S)),
        "peak_thruster_N": r4(np.nanmax(np.abs(T))),
        "peak_total_thrust_N": r4(np.nanmax(np.nansum(np.abs(T), axis=1))),
    }
    cmd_t = next((c["t"] for c in commands), None)
    if name == "path" and planned:
        cmd_t = planned["t"]
    if cmd_t is None:
        return s
    s["command_t"] = r4(cmd_t)
    end = len(t) - 1
    before = np.searchsorted(t, cmd_t) - 1
    p0, pf, y0, yf = P[max(before, 0)], P[end], Y[max(before, 0)], Y[end]
    after = t >= cmd_t
    if name in ("surge_step", "depth_step", "disturbance"):
        step = pf - p0
        L = np.linalg.norm(step)
        u = step / L if L > 1e-6 else step
        along = (P - p0) @ u
        err = np.linalg.norm(P - pf, axis=1)
        s["step_m"] = r4(L)
        s["overshoot_m"] = r4(max(0.0, np.nanmax(along[after]) - L))
        s["overshoot_pct"] = r4(100 * max(0.0, np.nanmax(along[after]) - L) / L) if L > 1e-6 else None
        for tol in (0.05, 0.02):
            out = np.where(after & (err > tol))[0]
            s[f"settle_{int(tol * 100)}cm_s"] = r4(t[out[-1]] - cmd_t) if len(out) else 0.0
        s["rise_10_90_s"] = rise(t, along, L, cmd_t)
        s["final_error_m"] = r4(np.linalg.norm(P[end] - np.array(d["ref_pos"][end], float)))
    if name == "yaw_step":
        step = yf - y0
        ang = (Y - y0) * np.sign(step)
        L = abs(step)
        s["step_deg"] = r4(math.degrees(L))
        s["overshoot_deg"] = r4(math.degrees(max(0.0, np.nanmax(ang[after]) - L)))
        for tol in (2.0, 1.0):
            out = np.where(after & (np.abs(Y - yf) > math.radians(tol)))[0]
            s[f"settle_{int(tol)}deg_s"] = r4(t[out[-1]] - cmd_t) if len(out) else 0.0
        s["rise_10_90_s"] = rise(t, ang, L, cmd_t)
        drift = np.linalg.norm(P[after] - P[max(before, 0)], axis=1)
        s["position_drift_max_m"] = r4(np.nanmax(drift))
    if name == "path" and planned:
        pts = np.array(planned["pts"], float)
        cross = np.array([dist_to_polyline(p, pts) for p in P[after]])
        R = np.array(d["ref_pos"], float)[after]
        s["path_points"] = len(pts)
        s["path_length_m"] = r4(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        s["cross_track_rms_m"] = r4(np.sqrt(np.nanmean(cross ** 2)))
        s["cross_track_max_m"] = r4(np.nanmax(cross))
        s["ref_tracking_rms_m"] = r4(np.sqrt(np.nanmean(np.sum((P[after] - R) ** 2, axis=1))))
        V = np.array([v if v[0] is not None else [np.nan] * 6 for v in d["vel"]], float)
        s["peak_speed_m_s"] = r4(np.nanmax(np.linalg.norm(V[:, :3], axis=1)))
        tr = np.array([v if v is not None else np.nan for v in d["trust"]], float)
        hi = np.where(after & (tr >= 0.8))[0]
        moving = np.where(after & (np.linalg.norm(V[:, :3], axis=1) > 0.05))[0]
        if len(moving):
            s["moving_s"] = r4(t[moving[-1]] - t[moving[0]])
    return s


def rise(t, x, L, cmd_t):
    if L < 1e-6:
        return None
    a = np.where((t >= cmd_t) & (x >= 0.1 * L))[0]
    b = np.where((t >= cmd_t) & (x >= 0.9 * L))[0]
    return r4(t[b[0]] - t[a[0]]) if len(a) and len(b) else None


def dist_to_polyline(p, pts):
    best = np.inf
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        L2 = ab @ ab
        u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, (p - a) @ ab / L2))
        best = min(best, np.linalg.norm(p - (a + u * ab)))
    return best


if __name__ == "__main__":
    main()
