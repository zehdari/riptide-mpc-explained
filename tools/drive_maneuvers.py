#!/usr/bin/env python3
"""Fly Talos (riptide_mpc on the Nereus simulator plant) through scripted maneuvers and bag each one.

Run with the sim already launched:
    ros2 launch nereus sim.launch.py active_control_model:=mpc mpc_model:=sim viewer:=false cameras:=false

    python3 drive_maneuvers.py --out /tmp/mpcbags [--only surge_step,yaw_step,...] [--hook-dir DIR]

Each maneuver starts from a settled hold, records `ros2 bag record` into <out>/<name>, commands the move,
waits until settled + `--tail` seconds, and stops the bag. With --hook-dir, the script touches
<DIR>/<name>.start / .end so an outside screen recorder can follow along.
"""

from __future__ import annotations

import argparse
import math
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from riptide_msgs2.action import FollowPath
from riptide_msgs2.msg import ControllerCommand, KillSwitchReport, PathSegment
from robot_localization.srv import SetPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

NS = "/talos/"
TOPICS = [
    "simulator/ground_truth",
    "odometry/filtered",
    "controller/mpc/predicted_path",
    "controller/mpc/planned_path",
    "controller/mpc/reference",
    "controller/mpc/solve_time_ms",
    "controller/mpc/disturbance",
    "controller/mpc/state",
    "controller/scale/trust",
    "thruster_forces",
    "simulator/actual_thruster_forces",
    "controller_debug_wrench",
    "controller/linear",
    "controller/angular",
    "state/kill",
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class Driver(Node):
    def __init__(self, external_kill=False):
        super().__init__("mpc_explainer_driver")
        self.odom = None
        self.killed = None
        self.create_subscription(Bool, NS + "state/kill", lambda m: setattr(self, "killed", m.data), 10)
        self.create_subscription(Odometry, NS + "odometry/filtered", self._odom, 50)
        self.linear = self.create_publisher(ControllerCommand, NS + "controller/linear", 10)
        self.angular = self.create_publisher(ControllerCommand, NS + "controller/angular", 10)
        self.kill_pub = self.create_publisher(KillSwitchReport, NS + "command/software_kill", 10)
        self.placement = self.create_client(SetPose, NS + "set_sim_pose")
        self.teleop = self.create_client(SetBool, NS + "setTeleop")
        self.path_client = ActionClient(self, FollowPath, NS + "follow_path")
        self.report = KillSwitchReport(kill_switch_id=1, sender_id="mpc_explainer", switch_asserting_kill=True)
        # The pool viewer keeps publishing its own kill state; with external_kill the operator's Enable /
        # KILL in the viewer decides and this script only waits for it.
        if not external_kill:
            self.create_timer(0.25, lambda: self.kill_pub.publish(self.report))
        self.target = None  # (x, y, z, yaw)

    def _odom(self, m):
        self.odom = m

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, end - time.monotonic())))

    def call(self, client, req):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("missing service " + client.srv_name)
        fut = client.call_async(req)
        end = time.monotonic() + 10
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError("timeout " + client.srv_name)
        return fut.result()

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)

    def command(self, x, y, z, yaw):
        self.target = (x, y, z, yaw)
        lin = ControllerCommand(mode=ControllerCommand.POSITION)
        lin.setpoint_vect.x, lin.setpoint_vect.y, lin.setpoint_vect.z = x, y, z
        ang = ControllerCommand(mode=ControllerCommand.POSITION)
        qx, qy, qz, qw = quat_yaw(yaw)
        ang.setpoint_quat.x, ang.setpoint_quat.y, ang.setpoint_quat.z, ang.setpoint_quat.w = qx, qy, qz, qw
        for _ in range(3):
            self.linear.publish(lin)
            self.angular.publish(ang)
            self.spin_for(0.05)

    def error(self):
        x, y, z, yaw = self.pose()
        tx, ty, tz, tyaw = self.target
        tw = self.odom.twist.twist
        dp = math.sqrt((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2)
        da = abs(wrap(yaw - tyaw))
        v = math.sqrt(tw.linear.x ** 2 + tw.linear.y ** 2 + tw.linear.z ** 2)
        w = math.sqrt(tw.angular.x ** 2 + tw.angular.y ** 2 + tw.angular.z ** 2)
        return dp, da, v, w

    def wait_settled(self, timeout=60.0, hold=2.0, pos=0.05, ang=math.radians(3), vel=0.03, rate=0.03):
        start, since = time.monotonic(), None
        while time.monotonic() - start < timeout:
            self.spin_for(0.05)
            dp, da, v, w = self.error()
            ok = dp < pos and da < ang and v < vel and w < rate
            now = time.monotonic()
            if ok:
                since = since or now
                if now - since >= hold:
                    return since - start
            else:
                since = None
        self.get_logger().warn(f"not settled after {timeout} s: err {self.error()}")
        return None


class Bag:
    def __init__(self, out: Path):
        self.out = out
        self.proc = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(out), *[NS + t for t in TOPICS]],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    def stop(self):
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", default="disturbance,surge_step,yaw_step,depth_step,path")
    ap.add_argument("--tail", type=float, default=10.0)
    ap.add_argument("--start", default="8,8,-1.0", help="placement x,y,z in map before releasing the kill")
    ap.add_argument("--hook-dir", type=Path)
    ap.add_argument("--external-kill", action="store_true",
                    help="a pool viewer is running: wait for its Enable instead of publishing the kill")
    ap.add_argument("--wait-hook", action="store_true", help="before the path, wait for <hook-dir>/go")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    only = args.only.split(",")

    rclpy.init()
    n = Driver(args.external_kill)
    log = n.get_logger()

    def hook(name):
        if args.hook_dir:
            args.hook_dir.mkdir(parents=True, exist_ok=True)
            (args.hook_dir / name).touch()

    def release():
        n.report.switch_asserting_kill = False
        if args.external_kill:
            hook("enable")
            log.info("waiting for Enable in the viewer")
            while n.killed is not False:
                n.spin_for(0.1)
            log.info("enabled")

    # Wait for odometry, place the robot (killed) in open water, then release the kill.
    while n.odom is None:
        n.spin_for(0.2)
    sx, sy, sz = (float(v) for v in args.start.split(","))
    req = SetPose.Request()
    req.pose.header.frame_id = "map"
    req.pose.pose.pose.position.x, req.pose.pose.pose.position.y, req.pose.pose.pose.position.z = sx, sy, sz
    req.pose.pose.pose.orientation.w = 1.0
    n.call(n.placement, req)
    n.spin_for(4.0)
    log.info(f"placed; EKF pose {n.pose()}")
    deadline = time.monotonic() + 20
    while not n.call(n.teleop, SetBool.Request(data=False)).success:
        if time.monotonic() > deadline:
            raise RuntimeError("setTeleop false failed")
        n.spin_for(0.5)

    x, y, z, yaw = n.pose()
    hold_z = -1.0

    if "disturbance" in only:
        # Record from the release of the kill: the learned disturbance converging while holding.
        bag = Bag(args.out / "disturbance")
        n.spin_for(2.0)
        release()
        # The viewer's Enable sends DISABLED setpoints: command the hold only after the release.
        n.spin_for(1.0)
        n.command(x, y, hold_z, yaw)
        hook("disturbance.start")
        n.spin_for(25.0)
        bag.stop()
        hook("disturbance.end")
    else:
        release()
        n.spin_for(1.0)
        n.command(x, y, hold_z, yaw)
    log.info(f"initial hold settled after {n.wait_settled(timeout=60)} s")
    n.spin_for(3.0)

    def step(name, fn):
        if name not in only:
            return
        n.command(*n.target)  # re-anchor on the current target
        n.wait_settled(timeout=30)
        bag = Bag(args.out / name)
        n.spin_for(2.0)
        hook(name + ".start")
        t0 = time.monotonic()
        fn()
        st = n.wait_settled(timeout=60, hold=1.0)
        log.info(f"{name}: settled {st} s after command")
        n.spin_for(args.tail)
        hook(name + ".end")
        bag.stop()
        log.info(f"{name}: bag closed ({time.monotonic() - t0:.1f} s)")

    def surge():
        tx, ty, tz, tyaw = n.target
        n.command(tx + 2.0 * math.cos(tyaw), ty + 2.0 * math.sin(tyaw), tz, tyaw)

    def yaw_step():
        tx, ty, tz, tyaw = n.target
        n.command(tx, ty, tz, wrap(tyaw + math.pi / 2))

    def depth():
        tx, ty, tz, tyaw = n.target
        n.command(tx, ty, tz - 0.5, tyaw)

    step("surge_step", surge)
    step("yaw_step", yaw_step)
    step("depth_step", depth)

    if "path" in only:
        # Back to 1 m and heading +x for the path, then the path from the settled hold.
        tx, ty, tz, tyaw = n.target
        n.command(tx, ty, hold_z, 0.0)
        n.wait_settled(timeout=60)
        if args.wait_hook and args.hook_dir:
            hook("path.ready")
            log.info("waiting for go hook")
            while not (args.hook_dir / "go").exists():
                n.spin_for(0.2)
        x0, y0, z0, yaw0 = n.target
        frame = n.odom.header.frame_id
        c, s = math.cos(yaw0), math.sin(yaw0)

        def local(u, v):
            return x0 + c * u - s * v, y0 + s * u + c * v

        goal = FollowPath.Goal()
        # (end point in start frame, end yaw rel. start, shape, arc center, sweep, heading mode)
        legs = [
            ((2.0, 0.0), 0.0, PathSegment.LINE, None, 0.0, PathSegment.HEADING_PATH),
            ((2.0, 1.5), math.pi / 2, PathSegment.LINE, None, 0.0, PathSegment.HEADING_PATH),
            ((0.4, 1.5), -math.pi / 2, PathSegment.ARC, (1.2, 1.5), math.pi, PathSegment.HEADING_PATH),
            ((0.4, 0.2), 0.0, PathSegment.LINE, None, 0.0, PathSegment.HEADING_WAYPOINT),
        ]
        for (u, v), dyaw, shape, center, sweep, heading in legs:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x, ps.pose.position.y = local(u, v)
            ps.pose.position.z = z0
            q = quat_yaw(wrap(yaw0 + dyaw))
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            seg = PathSegment(shape=shape, heading=heading, sweep=sweep)
            if center:
                seg.center.x, seg.center.y = local(*center)
                seg.center.z = z0
            goal.path_points.append(ps)
            goal.segments.append(seg)
        goal.max_speed = 0.0
        if not n.path_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("no follow_path server")
        bag = Bag(args.out / "path")
        n.spin_for(2.0)
        hook("path.start")
        t0 = time.monotonic()
        gf = n.path_client.send_goal_async(goal)
        while not gf.done():
            n.spin_for(0.05)
        gh = gf.result()
        log.info(f"path accepted: {gh.accepted}")
        rf = gh.get_result_async()
        while not rf.done() and time.monotonic() - t0 < 120:
            n.spin_for(0.1)
        res = rf.result().result if rf.done() else None
        log.info(f"path done after {time.monotonic() - t0:.1f} s: "
                 f"{res.error_code if res else 'timeout'} {res.error_msg if res else ''}")
        end = legs[-1]
        ex, ey = local(*end[0])
        n.target = (ex, ey, z0, wrap(yaw0 + end[1]))
        n.spin_for(args.tail)
        hook("path.end")
        bag.stop()

    # Leave it holding (killed) so the vehicle does not wander.
    n.report.switch_asserting_kill = True
    n.spin_for(0.5)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
