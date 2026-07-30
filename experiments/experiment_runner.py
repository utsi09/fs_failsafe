#!/usr/bin/env python3
"""Automated fault-injection experiment runner (host side).

Per case: restart bridge (docker) -> restart Autoware (docker, wait ready)
-> start cdr_monitor + mrm_supervisor (host) -> run leaderboard with the
fault env -> judge the outcome -> kill everything -> log one CSV row.

Assumes the CARLA server is already running on the host (checked via pgrep;
the runner stops if it dies so you can restart it manually).

Usage:
  python3 experiments/experiment_runner.py --smoke        # 1 quick case
  python3 experiments/experiment_runner.py                # full matrix
  python3 experiments/experiment_runner.py --dry          # print commands only
"""
import argparse
import csv
import itertools
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

REPO = Path("/home/taewook/autoware_carla_leaderboard")
RESULTS_JSON = REPO / "simulation_results.json"
OUT_DIR = REPO / "experiments" / "results"

LB_CONTAINER = "aw_carla_leaderboard_0916"
AW_CONTAINER = "autoware_awcl_01"

BRIDGE_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
              "source install/setup.bash && "
              "ros2 launch autoware_carla_cpp_bridge aw_carla_cpp_bridge.launch.py")
AUTOWARE_CMD = ("cd /home/taewook/autoware_awcl/autoware && "
                "source install/setup.bash && "
                "ros2 launch autoware_launch e2e_simulator.launch.xml "
                "diagnostic_graph_type:=carla "
                "vehicle_model:=carla_audi_etron_vehicle "
                "sensor_model:=carla_audi_etron_sensor_kit "
                "map_path:=/home/taewook/awcl_maps/Town07")
LEADERBOARD_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
                   "source install/setup.bash && source carla_envs.sh && "
                   "./run_leaderboard.sh {faults}")
CDR_CMD = "cd ~/fs_ws && source install/setup.bash && ros2 launch cdr_monitor cdr_monitor_awcl.launch.py"
DUMP_CMD = ("source ~/autoware_awcl/autoware/install/setup.bash && "
            "ros2 run autoware_diagnostic_graph_utils dump_node")
SUP_CMD = ("cd ~/fs_ws && source install/setup.bash && "
           "ros2 launch mrm_supervisor mrm_supervisor.launch.py baseline:={baseline}")

AUTOWARE_READY_NODE = "behavior_path_planner"
AUTOWARE_READY_TIMEOUT = 120.0
CASE_TIMEOUT_S = 420.0          # hard cap per case
STOPPED_SPEED = 0.15            # m/s
DEFER_STOP_CONFIRM_S = 30.0     # DEFER + stopped this long -> case over
STUCK_CONFIRM_S = 60.0          # stopped this long without MRM state -> stuck
# Town07 junctions cycle ~10 s green in a ~45 s rotation, so a legitimate
# red-light wait en route to the shoulder can exceed 60 s — give MRM_MOVE
# three full cycles before calling it stuck
MRM_MOVE_STUCK_S = 150.0

# ---------------------------------------------------------------- case matrix
# per-fault severity values (units differ: ln meters, cn pixel sigma 0-255,
# imn gyro rad/s, gn EKF pose meters)
SEVERITY = {
    "ln":  {"mid": 0.3,  "high": 0.6},
    "cn":  {"mid": 15.0, "high": 30.0},
    "imn": {"mid": 0.05, "high": 0.15},
    "gn":  {"mid": 1.0,  "high": 2.5},
}
SEV_LEVELS = ["mid", "high"]
# ln: BOTH lidars noise; cn: camera noise; imn: IMU noise; gn: GNSS/EKF pose
# noise; cf: camera destroyed; lf: front lidar destroyed; nf: no-fault control
FAULT_KINDS = ["ln", "cn", "imn", "gn", "cf", "lf", "nf"]
NO_SEVERITY = {"cf", "lf", "nf"}  # no severity dimension (nf: no trigger either)
TRIGGER_TIMES = [10, 30]
NPC_STYLES = ["def", "agg"]
METHODS = ["ours", "baseline"]      # baseline = stock Autoware MRM (in-lane stop)
REPEATS = 15


def build_cases(args):
    methods = METHODS if args.method == "both" else [args.method]
    if args.smoke:
        return [{"method": m,
                 "fault": args.fault or "ln",
                 "sev": args.sev or "mid",
                 "t": args.t if args.t is not None else 40,
                 "npc": args.npc or "def", "rep": 0} for m in methods]
    cases = []
    for method, fault, sev, t, npc, rep in itertools.product(
            methods, FAULT_KINDS, SEV_LEVELS, TRIGGER_TIMES, NPC_STYLES,
            range(args.reps if args.reps is not None else REPEATS)):
        if fault in NO_SEVERITY and sev != "mid":
            continue  # destruction faults have no severity dimension
        if fault == "nf" and t != TRIGGER_TIMES[0]:
            continue  # no-fault control has no trigger-time dimension
        # full runs: the same flags act as matrix filters
        if args.fault and fault != args.fault:
            continue
        if args.sev and sev != args.sev:
            continue
        if args.t is not None and t != args.t:
            continue
        if args.npc and npc != args.npc:
            continue
        cases.append({"method": method, "fault": fault, "sev": sev, "t": t,
                      "npc": npc, "rep": rep})
    return cases


def fault_args(case, dg="1"):
    if case["fault"] == "nf":
        args = ""                                   # control arm: no fault
    elif case["fault"] in NO_SEVERITY:
        args = f"{case['fault']}={case['t']}"
    else:
        args = f"{case['fault']}={case['t']}:{SEVERITY[case['fault']][case['sev']]}"
    # per-rep TM seed: ours/baseline share it (matched pairs), reps differ.
    # Without this the config seed 0 makes every repetition identical traffic.
    args += f" TM_SEED={100 + case['rep']}"
    if case["npc"] == "agg":
        args += " npc=agg"
    # guaranteed chaser behind the ego (rear-end pressure on MRM stops)
    args += " tail=10"
    if dg:
        # direct-goal: start -> final destination only, no intermediate
        # waypoints (same setup as the working 7/28 demo). dg="" disables.
        args += f" dg={dg}"
    return args


# ---------------------------------------------------------------- processes
class Procs:
    def __init__(self, dry):
        self.dry = dry
        self.host_procs = []

    def docker_bg(self, container, cmd, name, user=None):
        # force domain 0 regardless of the env baked into the container;
        # -u matches the user of the manual (working) terminals
        full = ["docker", "exec", "-e", "ROS_DOMAIN_ID=0"]
        if user:
            full += ["-u", user]
        full += [container, "bash", "-lc", cmd]
        return self._spawn(full, name)

    def host_bg(self, cmd, name):
        # DDS vendor must match the containers (CycloneDDS): cross-vendor
        # services hang unanswered even though topics mostly work
        env = ("export ROS_DOMAIN_ID=0; "
               "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
               "export CYCLONEDDS_URI=/home/taewook/cyclone_localhost.xml; ")
        return self._spawn(["bash", "-lc", env + cmd], name)

    def _spawn(self, argv, name):
        print(f"[runner] start {name}: {' '.join(argv[:3])} ...")
        if self.dry:
            return None
        # "wb": keep only the current case's log — appending across a 1000+
        # case campaign grows multi-GB files. The handle is closed in
        # kill_all; leaving it open leaks ~5 fds per case until EMFILE.
        # The previous case survives as .prev for post-mortems.
        path = OUT_DIR / f"{name}.log"
        if path.exists():
            path.replace(OUT_DIR / f"{name}.log.prev")
        log = open(path, "wb")
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        self.host_procs.append((name, p, log))
        return p

    def kill_all(self):
        print("[runner] cleanup...")
        if self.dry:
            return
        # kill the container-side process trees (docker exec clients don't
        # propagate signals into the container). "ros-args" matches every ROS
        # node binary — launch pkills alone leave zombie component_containers
        # behind, whose duplicated node names then break DDS discovery
        # (observed: 4 stacked autoware instances -> map never loads).
        for container, pats in [
            (LB_CONTAINER, ["ros-args", "leaderboard_evaluator", "ros2 launch",
                            "run_leaderboard"]),
            (AW_CONTAINER, ["ros-args", "component_container", "robot_state_publisher",
                            "rviz2", "ros2 launch"]),
        ]:
            for pat in pats:
                subprocess.run(["docker", "exec", container, "pkill", "-9", "-f", pat],
                               capture_output=True)
        # host side (cdr / supervisor / stray launches)
        for pat in ["cdr_monitor", "mrm_supervisor", "awcl_adapter", "ros2 launch"]:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        # per-launch ROS log dirs are never reused — left alone they grow by
        # thousands of dirs (GBs) over a long campaign
        subprocess.run(["docker", "exec", "-u", "aw", AW_CONTAINER, "bash", "-c",
                        "rm -rf /home/aw/.ros/log/* /tmp/launch_params_* 2>/dev/null"],
                       capture_output=True)
        subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-c",
                        "rm -rf /root/.ros/log/* /tmp/launch_params_* 2>/dev/null"],
                       capture_output=True)
        subprocess.run(["bash", "-c", "rm -rf ~/.ros/log/* 2>/dev/null"],
                       capture_output=True)
        # reset the ros2 CLI daemon so stale discovery caches don't linger
        subprocess.run(["bash", "-lc", "ros2 daemon stop"], capture_output=True)
        for name, p, log in self.host_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            log.close()
        self.host_procs.clear()
        if carla_alive():
            carla_unstick(False)
        time.sleep(3.0)


def carla_alive():
    return subprocess.run(["pgrep", "-f", "CarlaUE4"], capture_output=True).returncode == 0


CARLA_DIR = "/home/taewook"          # CarlaUE4.sh lives here, launched as ./CarlaUE4.sh
CARLA_LAUNCH = "cd /home/taewook && ./CarlaUE4.sh --ros2"
CARLA_CHECK_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
                   "source carla_envs.sh >/dev/null 2>&1; "
                   "python3 -c \"import carla; c=carla.Client('localhost',2000); "
                   "c.set_timeout(6.0); print(c.get_server_version())\"")


CARLA_UNSTICK_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
                     "source carla_envs.sh >/dev/null 2>&1; "
                     "python3 experiments/carla_unstick.py")


def carla_unstick(dry):
    """SIGKILLed cases leave the server in synchronous mode with no ticker —
    load_world() then times out forever. Force it back to async."""
    if dry:
        return True
    try:
        r = subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-lc",
                            CARLA_UNSTICK_CMD], capture_output=True, timeout=30)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def carla_responsive(dry):
    """pgrep only proves the process exists; a hung server still passes it.
    Ask the 0.9.16 client inside the LB container for the server version."""
    if dry:
        return True
    try:
        r = subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-lc",
                            CARLA_CHECK_CMD], capture_output=True, timeout=30)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def restart_carla(dry):
    print("[runner] CARLA unresponsive — restarting the server")
    if dry:
        return True
    subprocess.run(["pkill", "-9", "-f", "CarlaUE4"], capture_output=True)
    time.sleep(8.0)
    log = open(OUT_DIR / "carla.log", "wb")   # stays open for CARLA's lifetime
    subprocess.Popen(["bash", "-lc", CARLA_LAUNCH], stdout=log,
                     stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    t0 = time.time()
    while time.time() - t0 < 120.0:
        time.sleep(10.0)
        if carla_responsive(False):
            print("[runner] CARLA back up")
            return True
    return False


def ego_alive(dry, timeout=90.0):
    """Wait until the leaderboard has spawned the ego (odometry flowing)."""
    if dry:
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(
            ["bash", "-lc",
             "timeout 4 ros2 topic echo --once /localization/kinematic_state >/dev/null 2>&1"],
        )
        if r.returncode == 0:
            return True
        time.sleep(2.0)
    return False


def autoware_ready(dry, timeout=AUTOWARE_READY_TIMEOUT):
    if dry:
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(["bash", "-lc", "ros2 node list 2>/dev/null"],
                           capture_output=True, text=True)
        if AUTOWARE_READY_NODE in r.stdout:
            time.sleep(5.0)  # settle
            return True
        time.sleep(3.0)
    return False


# ---------------------------------------------------------------- judge
class Judge:
    """Watches supervisor state + ego odometry, decides how the case ended."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node as RclpyNode
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from autoware_adapi_v1_msgs.msg import MrmState

        rclpy.init()
        self.node = RclpyNode("experiment_judge")
        self.rclpy = rclpy
        print(f"[judge] rmw: {rclpy.utilities.get_rmw_implementation_identifier()}")
        self.MrmState = MrmState
        self.state = ""
        self.state_times = {}          # state -> first-seen wall time
        self.pose = None
        self.speed = None
        self.stopped_since = None
        self.stationary_since = None
        self.moved = False
        self.aw_state = 0              # MrmState.state from mrm_handler
        self.aw_behaviors = []         # behavior names in first-seen order
        self.fault_time = None         # wall time the fault went live
        self.deg_dist = 0.0            # meters driven after the fault
        self.min_gap = None            # nearest obstacle while stationary post-fault
        self._last_xy = None

        self.node.create_subscription(
            String, "/mrm_supervisor/state", self._state_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_cb, 10)
        self.node.create_subscription(
            MrmState, "/system/fail_safe/mrm_state", self._mrm_cb, 10)
        # operation-mode transitions explain "kept driving the old route" cases
        from autoware_adapi_v1_msgs.msg import OperationModeState
        self.op_mode = None
        self.node.create_subscription(
            OperationModeState, "/api/operation_mode/state", self._op_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        from std_msgs.msg import Bool, Float64
        self.node.create_subscription(
            Bool, "/fault_injection/active", self._fault_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            Float64, "/monitor/nearest_obstacle_m", self._gap_cb, 10)

        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    def _fault_cb(self, msg):
        if msg.data and self.fault_time is None:
            self.fault_time = time.time()
            print("[judge] fault active")

    def _gap_cb(self, msg):
        # near-miss metric: closest object while stopped after the fault
        # (rear traffic closing in on an in-lane MRM stop)
        if (self.fault_time is not None and self.speed is not None
                and self.speed < 0.5 and msg.data < 900.0):
            if self.min_gap is None or msg.data < self.min_gap:
                self.min_gap = msg.data

    OP_MODE_NAMES = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}

    def _op_cb(self, msg):
        label = (self.OP_MODE_NAMES.get(msg.mode, str(msg.mode))
                 + (" (in transition)" if msg.is_in_transition else ""))
        if label != self.op_mode:
            self.op_mode = label
            print(f"[judge] operation mode: {label}")

    def _spin_loop(self):
        # keep spinning through transient errors — a dead spin thread leaves
        # the judge blind (empty state, nan pose) for every remaining case
        while self.rclpy.ok():
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.2)
            except Exception:
                time.sleep(0.2)

    def _state_cb(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.state_times.setdefault(msg.data, time.time())
            print(f"[judge] supervisor state: {msg.data}")

    AW_BEHAVIOR_NAMES = {2: "EMERGENCY_STOP", 3: "COMFORTABLE_STOP", 4: "PULL_OVER"}

    def _mrm_cb(self, msg):
        if msg.state != self.aw_state:
            self.aw_state = msg.state
            print(f"[judge] autoware mrm state: {msg.state} behavior: {msg.behavior}")
        name = self.AW_BEHAVIOR_NAMES.get(msg.behavior)
        if name and name not in self.aw_behaviors:
            self.aw_behaviors.append(name)

    def _odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        now = time.time()
        if self.fault_time is not None and self._last_xy is not None:
            step = math.dist(self.pose, self._last_xy)
            if step < 5.0:                  # ignore teleports/resets
                self.deg_dist += step
        self._last_xy = self.pose
        # stationary time independent of `moved`: a fault can hit before the
        # ego ever moves (red light at spawn) and the case must still close
        if self.speed < STOPPED_SPEED:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None
        if self.speed > 1.0 and not self.moved:
            self.moved = True
            self.aw_behaviors = []   # drop startup-phase MRM noise from the record
        if self.speed < STOPPED_SPEED and self.moved:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None

    def outcome(self, case_t0, lb_proc):
        """Poll until a terminal condition; returns (outcome, detail)."""
        last_beat = 0.0
        while True:
            now = time.time()
            elapsed = now - case_t0
            if now - last_beat > 30.0:
                last_beat = now
                print(f"[judge] t+{elapsed:.0f}s state={self.state or '(none)'} "
                      f"speed={self.speed if self.speed is not None else 'no-odom'} "
                      f"mode={self.op_mode or '?'}")
            stopped_for = (now - self.stopped_since) if self.stopped_since else 0.0
            stationary_for = (now - self.stationary_since) if self.stationary_since else 0.0

            # supervisor publishes "STATE:target", e.g. "MRM_PARKED:shoulder_X"
            if self.state.startswith("MRM_PARKED"):
                return "SHOULDER_SUCCESS" if "shoulder" in self._target() else "PARKED_SUCCESS", self._detail()
            # stock Autoware MRM finished (comfortable/emergency stop done):
            # more informative than the generic "DEFER + stopped" fallback.
            # Only counts after the ego actually drove — during startup the
            # not-yet-ready stack "emergency stops" the parked vehicle and
            # reaches MRM_SUCCEEDED immediately.
            # stopped_for guard: with a flapping hazard, MRM_SUCCEEDED shows up
            # transiently mid-drive — require an actual sustained stop
            if (self.aw_state == self.MrmState.MRM_SUCCEEDED and self.moved
                    and stopped_for > 5.0):
                behavior = self.aw_behaviors[-1] if self.aw_behaviors else "UNKNOWN"
                return f"AW_{behavior}_DONE", self._detail()
            if self.state.startswith("MRM_DEFER") and stationary_for > DEFER_STOP_CONFIRM_S:
                return "DEFER_STOPPED", self._detail()
            if lb_proc is not None and lb_proc.poll() is not None:
                # judge can miss latched states (observed once after a DDS
                # vendor mix-up) — recover the verdict from the supervisor log
                if not self.state:
                    try:
                        log = (OUT_DIR / "supervisor.log").read_text(errors="ignore")
                        for line in reversed(log.splitlines()):
                            if "-> MRM_PARKED" in line:
                                return "SHOULDER_SUCCESS(log)", self._detail()
                            if "-> MRM_DEFER" in line:
                                return "DEFER_STOPPED(log)", self._detail()
                    except OSError:
                        pass
                return "SCENARIO_ENDED", self._detail()
            if self.moved and self.state == "" and stopped_for > STUCK_CONFIRM_S:
                return "STUCK_NO_MRM", self._detail()
            if self.state.startswith("MRM_MOVE") and stopped_for > MRM_MOVE_STUCK_S:
                return "STUCK_IN_MRM_MOVE", self._detail()
            if elapsed > CASE_TIMEOUT_S:
                return "TIMEOUT", self._detail()
            if not carla_alive():
                return "CARLA_DIED", self._detail()
            time.sleep(1.0)

    def _target(self):
        # state strings may carry the target in the future; fall back to state
        return self.state.lower()

    def _detail(self):
        px, py = self.pose if self.pose else (float("nan"), float("nan"))
        t_move = self.state_times.get("MRM_MOVE")
        t_end = self.stopped_since or time.time()
        tts = (t_end - t_move) if t_move else float("nan")
        det, tts_fault = "", ""
        if self.fault_time is not None:
            mrm_ts = [t for s_, t in self.state_times.items()
                      if s_.startswith("MRM_") and t >= self.fault_time - 1.0]
            if mrm_ts:
                det = round(min(mrm_ts) - self.fault_time, 1)
            if self.stationary_since is not None:
                tts_fault = round(self.stationary_since - self.fault_time, 1)
        return {"final_state": self.state, "x": round(px, 1), "y": round(py, 1),
                "time_to_stop_s": round(tts, 1) if not math.isnan(tts) else "",
                "det_latency_s": det, "tts_fault_s": tts_fault,
                "deg_dist_m": round(self.deg_dist, 1) if self.fault_time else "",
                "min_gap_m": round(self.min_gap, 1) if self.min_gap is not None else "",
                "aw_mrm": "+".join(self.aw_behaviors)}

    def reset(self):
        self.state = ""
        self.state_times.clear()
        self.pose = None
        self.speed = None
        self.stopped_since = None
        self.stationary_since = None
        self.moved = False
        self.aw_state = 0
        self.aw_behaviors = []
        self.op_mode = None
        self.fault_time = None
        self.deg_dist = 0.0
        self.min_gap = None
        self._last_xy = None

    def shutdown(self):
        self.rclpy.shutdown()


_MAP = None


def lane_occupancy(x, y):
    """(overlap_m, block_pct): how much of the 1.9 m-wide ego footprint lies
    inside the nearest DRIVING lane at the final stop pose. 0 = fully clear
    (shoulder), 100% of lane width = fully blocking. Narrow Town07 shoulders
    (1.6-2.4 m) force partial overlaps — this measures them continuously."""
    global _MAP
    try:
        if _MAP is None:
            import sys
            sys.path.insert(0, "/home/taewook/fs_ws/src/mrm_supervisor")
            from mrm_supervisor.map_model import MapModel
            _MAP = MapModel("/home/taewook/awcl_maps/Town07/lanelet2_map.osm")
        lid = _MAP.nearest_road_lanelet(x, y)
        ll = _MAP.lanelets[lid]
        c = ll["center"]
        # lateral offset from the centerline (nearest vertex is enough at
        # the ~1 m fidelity we need)
        lat = min(math.dist((x, y), p) for p in c)
        lpts = _MAP._ways.get(ll["left"], [])
        rpts = _MAP._ways.get(ll["right"], [])
        if not lpts or not rpts:
            return "", ""
        lp = min(lpts, key=lambda p: math.dist((x, y), p))
        rp = min(rpts, key=lambda p: math.dist((x, y), p))
        width = math.dist(lp, rp)
        half_car = 0.95
        overlap = max(0.0, min(lat + half_car, width / 2) - max(lat - half_car, -width / 2))
        return round(overlap, 2), round(100.0 * overlap / width, 1)
    except Exception:
        return "", ""


def parse_infractions():
    """(total collisions, per-type breakdown) from the leaderboard json."""
    try:
        import json
        data = json.loads(RESULTS_JSON.read_text())
        recs = data.get("_checkpoint", {}).get("records", [])
        if not recs:
            return "", ""
        inf = recs[-1].get("infractions", {})
        counts = {k.replace("collisions_", ""): len(v)
                  for k, v in inf.items() if "collision" in k and v}
        total = sum(counts.values())
        return total, "+".join(f"{k}:{n}" for k, n in counts.items())
    except Exception:
        return "", ""


# ---------------------------------------------------------------- main loop
def run(args):
    os.environ["ROS_DOMAIN_ID"] = "0"   # judge + ros2 CLI polling
    # judge must speak the containers' DDS dialect too
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = "/home/taewook/cyclone_localhost.xml"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%m%d_%H%M")
    csv_path = OUT_DIR / f"results_{stamp}.csv"
    cases = build_cases(args)
    print(f"[runner] {len(cases)} cases -> {csv_path}")

    procs = Procs(args.dry)
    judge = None if args.dry else Judge()

    with open(csv_path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["idx", "method", "fault", "severity", "trigger_s", "npc", "rep",
                     "outcome", "final_state", "aw_mrm", "stop_x", "stop_y",
                     "lane_overlap_m", "lane_block_pct",
                     "det_latency_s", "tts_fault_s", "deg_dist_m", "min_gap_m",
                     "time_to_stop_s", "collisions", "collision_types", "wall_s"])
        interrupted = False
        for i, case in enumerate(cases):
            if interrupted:
                break
            # UE4 leaks memory on every load_world; a preemptive restart every
            # 40 cases beats waiting for the slow-then-crash phase
            if i > 0 and i % 40 == 0 and not args.dry:
                restart_carla(False)
            if not args.dry and not (carla_alive() and carla_responsive(False)
                                     and carla_unstick(False)):
                if not restart_carla(args.dry):
                    print("[runner] CARLA could not be recovered — stopping.")
                    break
            print(f"\n===== case {i + 1}/{len(cases)}: {case} =====")
            t0 = time.time()
            try:
                procs.kill_all()
                if judge:
                    judge.reset()
                RESULTS_JSON.unlink(missing_ok=True)

                procs.docker_bg(LB_CONTAINER, BRIDGE_CMD, "bridge")
                procs.docker_bg(AW_CONTAINER, AUTOWARE_CMD, "autoware", user="aw")
                if not autoware_ready(args.dry):
                    raise RuntimeError("autoware not ready in time")
                # monitors start BEFORE the leaderboard: their diagnostic-graph
                # units read stale (= ERROR) while absent, which flaps the stock
                # MRM and blocks engagement. Startup transients are safe now —
                # cdr reports OK during sensor warmup and the supervisor stays
                # disarmed until it has seen healthy scores once.
                procs.host_bg(CDR_CMD, "cdr")
                baseline = "true" if case["method"] == "baseline" else "false"
                procs.host_bg(SUP_CMD.format(baseline=baseline), "supervisor")
                # per-unit graph levels — to identify which unit flaps the MRM
                procs.host_bg(DUMP_CMD, "diag_dump")
                lb = procs.docker_bg(LB_CONTAINER,
                                     LEADERBOARD_CMD.format(
                                         faults=fault_args(case, args.dg)),
                                     "leaderboard")
                if not ego_alive(args.dry):
                    raise RuntimeError("ego odometry never appeared")
                if args.dry:
                    outcome, detail = "DRY", {}
                else:
                    outcome, detail = judge.outcome(t0, lb)
            except KeyboardInterrupt:
                outcome, detail = "INTERRUPTED", (judge._detail() if judge else {})
                interrupted = True
            except Exception as e:
                outcome, detail = "SETUP_FAIL", {"final_state": str(e)[:60]}
            finally:
                procs.kill_all()

            wall = round(time.time() - t0, 1)
            n_coll, coll_types = parse_infractions()
            sx, sy = detail.get("x", ""), detail.get("y", "")
            ov, pct = lane_occupancy(sx, sy) if isinstance(sx, float) and not math.isnan(sx) else ("", "")
            row = [i, case["method"], case["fault"], case["sev"], case["t"],
                   case["npc"], case["rep"],
                   outcome, detail.get("final_state", ""), detail.get("aw_mrm", ""),
                   sx, sy, ov, pct,
                   detail.get("det_latency_s", ""), detail.get("tts_fault_s", ""),
                   detail.get("deg_dist_m", ""), detail.get("min_gap_m", ""),
                   detail.get("time_to_stop_s", ""),
                   n_coll, coll_types, wall]
            wr.writerow(row)
            f.flush()
            print(f"[runner] result: {outcome} ({wall}s)")

    if judge:
        judge.shutdown()
    print(f"\n[runner] done -> {csv_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true", help="run a single quick case")
    ap.add_argument("--dry", action="store_true", help="print commands, run nothing")
    ap.add_argument("--method", choices=["ours", "baseline", "both"], default="both",
                    help="which MRM method arm(s) to run")
    ap.add_argument("--fault", choices=FAULT_KINDS,
                    help="fault kind (smoke: sets it; full run: filters matrix)")
    ap.add_argument("--sev", choices=SEV_LEVELS,
                    help="severity (smoke: sets it; full run: filters matrix)")
    ap.add_argument("--t", type=int,
                    help="fault trigger time [s] (smoke: sets it; full run: filters)")
    ap.add_argument("--npc", choices=NPC_STYLES,
                    help="npc style (smoke: sets it; full run: filters matrix)")
    ap.add_argument("--reps", type=int,
                    help=f"repetitions per cell (default {REPEATS})")
    ap.add_argument("--dg", default="-102:95:-8",
                    help='final goal: default "-102:95:-8" — 354 m route through '
                         'the tagged-shoulder corridor (center road west, then '
                         'north); "1" = route\'s own destination, '
                         '"x:y[:yaw_deg]" custom, "" full waypoints')
    args = ap.parse_args()
    run(args)
