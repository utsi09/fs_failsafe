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
import json
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
# [우측 실주차장 데모 임시] CDR_DEMO_NO_ERROR=1: gn 노이즈 스파이크가 Autoware
# 자체 MRM을 격발해 주차 기동을 끊는 것 방지 (WARN 구간 소유권은 supervisor 설계).
# supervisor DEFER 핸드오버 진단은 유지. 데모 후 원복 필수.
CDR_CMD = "cd ~/fs_ws && source install/setup.bash && CDR_DEMO_NO_ERROR=1 ros2 launch cdr_monitor cdr_monitor_awcl.launch.py"
DUMP_CMD = ("source ~/autoware_awcl/autoware/install/setup.bash && "
            "ros2 run autoware_diagnostic_graph_utils dump_node")
# [v4.9k] autoware install 선소스: tier4_planning_msgs(속도 캡 메시지) 확보
SUP_CMD = ("source ~/autoware_awcl/autoware/install/setup.bash >/dev/null 2>&1; "
           "cd ~/fs_ws && source install/setup.bash && "
           "ros2 launch mrm_supervisor mrm_supervisor.launch.py baseline:={baseline}")

AUTOWARE_READY_NODE = "behavior_path_planner"
AUTOWARE_READY_TIMEOUT = 120.0
CASE_TIMEOUT_S = 900.0          # 임시: 주차 데모용 (원복 필요)
STOPPED_SPEED = 0.15            # m/s
DEFER_STOP_CONFIRM_S = 30.0     # DEFER + stopped this long -> case over
# Town07 lights: ~10 s green per ~45 s rotation. Missing ONE green is
# tolerated; standing through TWO full cycles in NORMAL state means the
# stock stack lost the light (recognition miss) -> fail fast, next case
JUNCTION_STUCK_S = 100.0
# Town07 junctions cycle ~10 s green in a ~45 s rotation, so a legitimate
# red-light wait en route to the shoulder can exceed 60 s — give MRM_MOVE
# three full cycles before calling it stuck
MRM_MOVE_STUCK_S = 150.0
# 주차장 타깃은 정차 5s+scenario 전환+costmap+A*(30s+)+freespace 기동으로
# 정지 구간이 길다 — 150 s로 자르면 기동 중 케이스를 사살한다.
# v3.6b: supervisor stall 재시도 체인(180s×3)까지 수용하도록 560 s.
# 타이머 위계: supervisor 재시도(~540) < judge 560 < leaderboard blocked 600
# < CASE_TIMEOUT 900 (8/11 실측: 321s 걸려 성공하는 트래픽 케이스 존재)
MRM_MOVE_STUCK_PARKING_S = 560.0
# --mrm-off (FTTI 도출 모드): 고장 후 이 시간 동안 무사고면 SURVIVED,
# 고장 이후 시작된 정지가 이 시간 이상 지속되면 HAZARD_STOPPED
HAZARD_SURVIVE_S = 360.0
HAZARD_STOP_S = 60.0
# fresh CARLA per case: the server degrades progressively (boot 57s -> 150s+
# by case 3-4, sim-rate drop re-flaps the 3 s diag timeouts and poisons
# verdicts). ~1 min/restart is cheaper than a wasted 7-min case, and a fresh
# world per case is better experimental hygiene anyway (no leftover actors).
CARLA_RESTART_EVERY = 1
DEST_XY = None   # [v4.7] --dg 목적지 (x, y) — 완주 근사 판정용 (main에서 설정)

# ---------------------------------------------------------------- case matrix
# per-fault severity values (units differ: ln meters, cn pixel sigma 0-255,
# imn gyro rad/s, gn EKF pose meters)
SEVERITY = {
    "ln":  {"mid": 0.3,  "high": 0.6},
    "cn":  {"mid": 15.0, "high": 30.0},
    # [v4.6h] imn 재보정: 감지기(EKF innovation EMA)가 느려 0.05→det 210s,
    # 0.15→171s = 미션長 초과 -> 실질 미감지. 0.5→53.6s (프로브 실측). mid는
    # "미션 내 감지 가능" 원칙에 맞춘 스트레스 강도로 문서화.
    "imn": {"mid": 0.5, "high": 1.5},
    "gn":  {"mid": 1.0,  "high": 2.5},
    "imb": {"mid": 0.3,  "high": 1.0},   # v2: 자이로 바이어스 [rad/s]
    "imd": {"mid": 0.005, "high": 0.02},  # 자이로 바이어스 램프(드리프트) [rad/s^2]
                                          # mid: 60s에 imb-mid 수준 도달, high: 15s
    # [v4.6] 현실형 라이다 열화 2종 (2026-08-12 문헌조사: 현실 빈도 1·2위 유형,
    # 물리 파라미터화 — lso: 커버투과율T:섹터폭deg (Trierweiler/Goodin),
    # lcl: 강우량mm/h:클러터점수 (Beer-Lambert alpha=0.01*Rr^0.6, LISA 클러터))
    # lso T 보정(v4.6b): 섹터-far 감지기 기준 역산 — mid 유효거리 ~55m(전방 원거리
    # 절반 잔존 -> 게이트 통과, 이동 MRC), high ~12m(전방 실명 -> DEFER).
    # CARLA intensity가 재질 무시(~e^-0.004d)라 문헌 T와 절대값은 다름
    "lso": {"mid": "0.33:90", "high": "0.07:150"},
    "lcl": {"mid": "5:1500", "high": "25:4000"},
    # [gm] GNSS Gauss-Markov 상관편향: sigma[m]:tau[s] — 멀티패스/NLOS의
    # 시간상관 오차 (Langel'21 GM 모델, Bhatti'07 느린오차=최악). EXTRA 폴트.
    "gm": {"mid": "3.0:30", "high": "8.0:30"},
}
SEV_LEVELS = ["mid", "high"]
# ln: BOTH lidars noise; cn: camera noise; imn: IMU noise; gn: GNSS/EKF pose
# noise; cf: camera destroyed; lf: front lidar destroyed; nf: no-fault control
FAULT_KINDS = ["cn", "imn", "gn", "lso", "lcl", "cf", "lf", "nf"]
# v2 추가 고장: imb=자이로 바이어스, rain=폭우 프리셋, gb=GNSS 블랙아웃.
# 기본 매트릭스에는 넣지 않고 --fault 로 명시 선택했을 때만 실행된다.
EXTRA_FAULT_KINDS = ["ln", "gm", "imb", "imd", "rain", "gb", "cfl", "mix", "cfl2", "lfl2"]
# [v4.6] ln(가우시안 반경노이즈)은 본 매트릭스에서 제외 — 현실 대표성 낮음
# (문헌: 지터는 실제 열화의 최소 성분). lso/lcl이 대체, ln은 ablation용 EXTRA.
NO_SEVERITY = {"cf", "lf", "nf", "rain", "gb", "cfl", "mix", "cfl2", "lfl2"}  # no severity dimension (nf: no trigger either)
# fault time is a RANDOM COVARIATE, not a matrix dimension: fixed trigger
# times pinned the fault to the same spots so the supervisor always picked
# the same shoulder. Per-case t is drawn deterministically from the cell key
# (reproducible; ours/baseline share it like TM_SEED) and recorded in the CSV.
TRIGGER_RANGE = (10, 45)   # goal_planner owns the last ~25 s of the ~85 s drive
# v3: 트래픽 밀도를 매트릭스 차원으로 승격 — 무교통/기본/공격적 3수준
NPC_STYLES = ["none", "def", "agg"]
# 매트릭스 밖 보조 스타일 (--npc 로 명시 선택): low=4대 상한, high=기본+8대
EXTRA_NPC_STYLES = ["low", "high"]
METHODS = ["ours", "baseline"]      # baseline = stock Autoware MRM (in-lane stop)
REPEATS = 15


def build_cases(args):
    methods = METHODS if args.method == "both" else [args.method]
    if args.retry_failed:
        # v2: 기존 결과 CSV에서 CARLA_DIED/SETUP_FAIL 케이스만 같은
        # (fault,sev,npc,rep,method,t) 조건으로 다시 실행
        cases = []
        with open(args.retry_failed, newline="") as f:
            for row in csv.DictReader(f):
                if row.get("outcome") in ("CARLA_DIED", "SETUP_FAIL"):
                    cases.append({"method": row["method"], "fault": row["fault"],
                                  "sev": row["severity"], "t": int(row["trigger_s"]),
                                  "npc": row["npc"], "rep": int(row["rep"])})
        return cases
    if args.smoke:
        return [{"method": m,
                 "fault": args.fault or "ln",
                 "sev": args.sev or "mid",
                 "t": args.t if args.t is not None else 40,
                 "npc": args.npc or "def", "rep": 0} for m in methods]
    import random
    cases = []
    # v2 고장은 --fault 로 명시했을 때만 매트릭스에 합류
    fault_kinds = FAULT_KINDS + [f for f in EXTRA_FAULT_KINDS if f == args.fault]
    # 밀도 스타일(none/low/high)도 --npc 로 명시했을 때만 매트릭스에 합류
    npc_styles = NPC_STYLES + [s for s in EXTRA_NPC_STYLES if s == args.npc]
    for fault, sev, npc, rep, method in itertools.product(
            fault_kinds, SEV_LEVELS, npc_styles,
            range(args.reps if args.reps is not None else REPEATS), methods):
        if fault in NO_SEVERITY and sev != "mid":
            continue  # destruction faults have no severity dimension
        # reproducible per-cell draw, identical for ours/baseline (paired)
        t = (args.t if args.t is not None else
             random.Random(f"{fault}/{sev}/{npc}/{rep}").randint(*TRIGGER_RANGE))
        # full runs: the same flags act as matrix filters
        if args.fault and fault != args.fault:
            continue
        if args.sev and sev != args.sev:
            continue
        if args.npc and npc != args.npc:
            continue
        cases.append({"method": method, "fault": fault, "sev": sev, "t": t,
                      "npc": npc, "rep": rep})
    if args.limit:
        # sanity runs: spread the slice across the matrix instead of taking
        # the first N (which would all be the same method/fault)
        step = max(1, len(cases) // args.limit)
        cases = cases[::step][:args.limit]
    return cases


def fault_args(case, dg="1"):
    if case["fault"] == "nf":
        args = ""                                   # control arm: no fault
    elif case["fault"] == "mix":
        # 복합 고장: 라이다+카메라 동시 노이즈(악천후 유사) — min() 합성 검증
        args = f"ln={case['t']}:0.3 cn={case['t']}:15.0"
    elif case["fault"] in NO_SEVERITY:
        args = f"{case['fault']}={case['t']}"
    else:
        args = f"{case['fault']}={case['t']}:{SEVERITY[case['fault']][case['sev']]}"
    # per-rep TM seed: ours/baseline share it (matched pairs), reps differ.
    # Without this the config seed 0 makes every repetition identical traffic.
    args += f" TM_SEED={100 + case['rep']}"
    if case["npc"] == "agg":
        args += " npc=agg"
    elif case["npc"] == "none":
        args += " npccap=0"
    elif case["npc"] == "low":
        args += " npccap=4"
    elif case["npc"] == "high":
        args += " npcadd=8"
    # guaranteed chaser behind the ego (rear-end pressure on MRM stops)
    if os.environ.get("DEMO_CLEARNPC"):
        args += " clearnpc=1"      # [우측 실주차장 데모: 배경차 제거, 꼬리차 생략]
    elif case["npc"] == "none":
        pass                       # 무교통 조건: 꼬리차도 생략
    else:
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
        # [v5 절제] 시드 env 패스스루 — 오케스트레이터가 케이스별로 설정
        for _sk in ("FAULT_SEED", "FAULT_TRACE", "WALKER_SEED", "CASE_TAG", "DETECT_CAP", "FREEZE_TL"):
            _sv = os.environ.get(_sk)
            if _sv:
                full += ["-e", f"{_sk}={_sv}"]
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
                            "run_leaderboard", "tl_v2x_relay", "hazard_injector"]),
            (AW_CONTAINER, ["ros-args", "component_container", "robot_state_publisher",
                            "rviz2", "ros2 launch"]),
        ]:
            for pat in pats:
                subprocess.run(["docker", "exec", container, "pkill", "-9", "-f", pat],
                               capture_output=True)
        # host side (cdr / supervisor / stray launches)
        # dump_node included: orphaned dump instances duplicate the node name ->
        # duplicated_node_checker ERROR -> every mode unavailable -> permanent
        # emergency stop (three strays found blocking departure, 0731)
        # dummy_defer 추가(0815 실측): 러너 강제종료가 남긴 dummy_defer 2마리가
        # 같은 노드명으로 떠서 duplicated_node_checker ERROR -> 전 케이스 출발불가
        for pat in ["cdr_monitor", "mrm_supervisor", "awcl_adapter", "ros2 launch",
                    "diagnostic_graph_utils dump_node", "dummy_defer"]:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        # the pattern above only matches the `ros2 run` wrapper; killing it -9
        # orphans the actual dump_node binary (path has a slash, not a space),
        # and one orphan per case re-triggers duplicated_node_checker
        subprocess.run(["pkill", "-9", "-x", "dump_node"], capture_output=True)
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
# [0813 밤] 모니터 전원 분리로 X 출력 0개 -> UE4가 창 초기화 세그폴트 반복
# (carla_archive.log Signal=11 연쇄, lcl 141/180 조기종료의 원인). 헤드리스
# 렌더링이면 디스플레이 유무와 무관 — 센서 캡처는 동일하게 렌더된다.
CARLA_LAUNCH = "cd /home/taewook && ./CarlaUE4.sh --ros2 -RenderOffScreen"
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


def ego_alive(dry, timeout=180.0):
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


POST_MRC_OBS_S = 110.0   # 정차 후 관측창(판정 전 10 s 유량 관측과 합쳐 ~120 s)


def post_mrc_observe(window_s):
    """정차 후 NPC 상충 대리 지표(급감속·근접통과·min TTC) 계측 — carla 클라이언트."""
    try:
        r = subprocess.run(
            ["bash", "-lc",
             "cd /home/taewook/autoware_carla_leaderboard && "
             "source carla_envs.sh >/dev/null 2>&1; "
             f"python3 experiments/post_mrc_observe.py {window_s:.0f}"],
            capture_output=True, text=True, timeout=window_s + 30.0)
        line = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "{}"
        return json.loads(line)
    except Exception as e:
        print(f"[judge] post-MRC observe failed: {e}")
        return {}


def capture_stall_factor():
    """STUCK/DEFER 원인 추적: 확정 직전 velocity factor의 behavior를 1회 캡처."""
    try:
        r = subprocess.run(
            ["bash", "-lc",
             "timeout 8 ros2 topic echo --once /api/planning/velocity_factors"],
            capture_output=True, text=True, timeout=12)
        vals = []
        for line in r.stdout.splitlines():
            s = line.strip()
            if s.startswith("behavior:"):
                v = s.split(":", 1)[1].strip().strip("'\"")
                if v and v not in vals:
                    vals.append(v)
        return "+".join(vals)
    except Exception:
        return ""   # 실패 시 빈칸 (케이스 진행에는 영향 없음)


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
        self.fault_time_sim = None     # sim time (/clock) the fault went live
        self.state_times_sim = {}      # state -> first-seen sim time
        self.sim_now = None            # latest /clock [s]
        self.deg_dist = 0.0            # meters driven after the fault
        self.min_gap = None            # nearest obstacle while stationary post-fault
        self._last_xy = None
        self._obs_start = None         # traffic-flow observation window
        self._obs_samples = []
        self.traffic = ""              # cleared / blocked after the stop
        self.stall_factor = ""         # STUCK/DEFER 시점의 정지 요인 (v2)
        self.npc_style = ""            # 관측창 게이트용 (none 은 상충 정의상 0)
        self.obs = {}                  # post-MRC 관측창 대리 지표 (v4)

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
        from rosgraph_msgs.msg import Clock
        self.node.create_subscription(
            Bool, "/fault_injection/active", self._fault_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            Float64, "/monitor/nearest_obstacle_m", self._gap_cb, 10)
        # [0822] 심시간 지연 측정: destroy 히치 동안 /clock이 멈추므로
        # 벽시계 지연은 히치를 포함해 과대측정된다 — 안전 지연은 심시간 병기
        self.node.create_subscription(Clock, "/clock", self._clock_cb, 10)

        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    def _clock_cb(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _fault_cb(self, msg):
        if msg.data and self.fault_time is None:
            self.fault_time = time.time()
            self.fault_time_sim = self.sim_now
            print("[judge] fault active")

    def _gap_cb(self, msg):
        # near-miss metric: closest object while stopped after the fault
        # (rear traffic closing in on an in-lane MRM stop)
        if (self.fault_time is not None and self.speed is not None
                and self.speed < 0.5 and msg.data < 900.0):
            if self.min_gap is None or msg.data < self.min_gap:
                self.min_gap = msg.data
        if self._obs_start is not None and msg.data < 900.0:
            self._obs_samples.append(msg.data)

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
        while getattr(self, "_spin_alive", True) and self.rclpy.ok():
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.2)
            except Exception:
                time.sleep(0.2)

    def _state_cb(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.state_times.setdefault(msg.data, time.time())
            if self.sim_now is not None:
                self.state_times_sim.setdefault(msg.data, self.sim_now)
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
            verdict = None
            if self.state.startswith("MRM_PARKED"):
                verdict = ("SHOULDER_SUCCESS" if "shoulder" in self._target()
                           else "PARKED_SUCCESS")
            # stock Autoware MRM finished (comfortable/emergency stop done):
            # more informative than the generic "DEFER + stopped" fallback.
            # Only counts after the ego actually drove — during startup the
            # not-yet-ready stack "emergency stops" the parked vehicle and
            # reaches MRM_SUCCEEDED immediately.
            # stopped_for guard: with a flapping hazard, MRM_SUCCEEDED shows up
            # transiently mid-drive — require an actual sustained stop
            if (verdict is None
                    and self.aw_state == self.MrmState.MRM_SUCCEEDED and self.moved
                    and stopped_for > 5.0
                    # 주차장 MOVE 중에는 보류: Parking A* 스톨(수 초 틱 정지)로
                    # rate 체크가 일괄 ERROR -> AW emergency가 일시 SUCCEEDED 되는
                    # 게 정상 경과 (8/10 실측 — 회복 후 주차 계속). 케이스 상한은
                    # MRM_MOVE_STUCK_PARKING_S(240s)와 CASE_TIMEOUT이 담당.
                    and not (self.state.startswith("MRM_MOVE")
                             and ":parking" in self.state)):
                behavior = self.aw_behaviors[-1] if self.aw_behaviors else "UNKNOWN"
                verdict = f"AW_{behavior}_DONE"
            if (verdict is None and self.state.startswith("MRM_DEFER")
                    and stationary_for > DEFER_STOP_CONFIRM_S):
                verdict = "DEFER_STOPPED"
            if verdict is not None:
                # don't close the case yet: watch ~10 s whether rear traffic
                # actually gets PAST the stopped ego (shoulder park should
                # read "cleared", an in-lane stop with a chaser "blocked")
                if self._obs_start is None:
                    self._obs_start = now
                    self._obs_samples = []
                    if verdict == "DEFER_STOPPED":
                        # 확정 직전 1회: 무엇이 차를 세우고 있는지 기록 (v2)
                        self.stall_factor = capture_stall_factor()
                    print(f"[judge] {verdict} — observing traffic flow 10 s")
                lb_dead = lb_proc is not None and lb_proc.poll() is not None
                if now - self._obs_start >= 10.0 or lb_dead:
                    smp = self._obs_samples
                    if smp:
                        gmin = min(smp)
                        gend = sum(smp[-3:]) / len(smp[-3:])
                        self.traffic = ("blocked" if gend < 12.0 and gend - gmin < 3.0
                                        else "cleared")
                    # [v4 관측창] 정차 후 NPC 상충 대리 지표 — 트래픽이 있는
                    # 케이스만(none 은 정의상 0). AgentBlocked 600 s 이내라
                    # 시나리오는 계속 살아 있다.
                    if not lb_dead and self.npc_style in ("def", "agg", "low", "high"):
                        win = min(POST_MRC_OBS_S, max(0.0, CASE_TIMEOUT_S - 20.0 - elapsed))
                        if win > 20.0:
                            print(f"[judge] post-MRC observation {win:.0f} s")
                            self.obs = post_mrc_observe(win)
                    return verdict, self._detail()
                time.sleep(1.0)
                continue
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
            if (self.moved and self.state in ("", "NORMAL")
                    and stationary_for > JUNCTION_STUCK_S):
                # [v4.7] 목적지 30m 이내 정지 = 사실상 완주. dg 지점(-102,95)의
                # 도착 pull-over가 goal_planner "0 candidates"로 만성 실패(7/31
                # 부터 전 배터리 동일)라 24m 앞 정지가 STUCK으로 오분류되던 것 —
                # 임무 수행은 끝났고 마지막 기동만 맵 특이점이므로 완주로 판정.
                if self.pose is not None and DEST_XY is not None \
                        and math.dist(self.pose, DEST_XY) <= 30.0:
                    return "ROUTE_COMPLETE_NEAR", self._detail()
                self.stall_factor = capture_stall_factor()   # 확정 직전 1회 (v2)
                return "STUCK_AT_JUNCTION", self._detail()
            if self.state.startswith("MRM_MOVE") and stopped_for > (
                    MRM_MOVE_STUCK_PARKING_S if ":parking" in self.state
                    else MRM_MOVE_STUCK_S):
                self.stall_factor = capture_stall_factor()   # 확정 직전 1회 (v2)
                return "STUCK_IN_MRM_MOVE", self._detail()
            if elapsed > CASE_TIMEOUT_S:
                return "TIMEOUT", self._detail()
            if not carla_alive():
                return "CARLA_DIED", self._detail()
            time.sleep(1.0)

    def outcome_hazard(self, case_t0, lb_proc):
        """--mrm-off (FTTI 도출): 개입 없이 고장 -> 위험 사건까지 관찰만 한다.

        위험 사건: 충돌(HAZARD_COLLISION), 차로 이탈(HAZARD_LANE_DEPART),
        지속 정지(HAZARD_STOPPED). 고장 후 6분 무사고면 HAZARD_SURVIVED."""
        last_beat = 0.0
        while True:
            now = time.time()
            elapsed = now - case_t0
            if now - last_beat > 30.0:
                last_beat = now
                print(f"[judge] t+{elapsed:.0f}s (hazard watch) "
                      f"speed={self.speed if self.speed is not None else 'no-odom'} "
                      f"fault={'yes' if self.fault_time else 'no'}")
            # 고장 이후에만 leaderboard 로그의 충돌/이탈 이벤트를 인정
            # (시동 단계 이벤트 오염 방지)
            if self.fault_time is not None:
                hz = self._leaderboard_hazard()
                if hz:
                    return hz, self._detail_hazard(now)
                if (self.stationary_since is not None
                        and self.stationary_since >= self.fault_time
                        and now - self.stationary_since > HAZARD_STOP_S):
                    # 정지 시작 시점을 사건 시각으로 기록
                    return "HAZARD_STOPPED", self._detail_hazard(self.stationary_since)
                if now - self.fault_time > HAZARD_SURVIVE_S:
                    return "HAZARD_SURVIVED", self._detail_hazard(None)
            if lb_proc is not None and lb_proc.poll() is not None:
                # 시나리오가 먼저 끝났으면 결과 json으로 충돌 여부 최종 확인
                n_coll, _types = parse_infractions()
                if isinstance(n_coll, int) and n_coll > 0:
                    return "HAZARD_COLLISION", self._detail_hazard(now)
                return "SCENARIO_ENDED", self._detail_hazard(None)
            if elapsed > CASE_TIMEOUT_S + HAZARD_SURVIVE_S:
                return "TIMEOUT", self._detail_hazard(None)
            if not carla_alive():
                return "CARLA_DIED", self._detail_hazard(None)
            time.sleep(1.0)

    def _leaderboard_hazard(self):
        # leaderboard 로그를 훑어 충돌/차로이탈 이벤트를 즉시 감지
        try:
            log = (OUT_DIR / "leaderboard.log").read_text(errors="ignore")
        except OSError:
            return None
        low = log.lower()
        # "collided": 실제 사건 문구만 매칭 ("collision"은 종료 요약표에도 등장)
        if "collided" in low:
            return "HAZARD_COLLISION"
        if "outside its route lanes" in low or "lane invasion" in low:
            return "HAZARD_LANE_DEPART"
        return None

    def _detail_hazard(self, hz_time):
        # 고장 -> 위험 사건까지 걸린 시간을 t_hazard_s 로 덧붙인다
        d = self._detail()
        d["t_hazard_s"] = (round(hz_time - self.fault_time, 1)
                           if hz_time is not None and self.fault_time is not None
                           else "")
        return d

    def _target(self):
        # state strings may carry the target in the future; fall back to state
        return self.state.lower()

    def _detail(self):
        px, py = self.pose if self.pose else (float("nan"), float("nan"))
        # states carry the target ("MRM_MOVE:shoulder_X") — exact get() never hit
        t_move = min((t for s_, t in self.state_times.items()
                      if s_.startswith("MRM_MOVE")), default=None)
        t_end = self.stopped_since or time.time()
        tts = (t_end - t_move) if t_move else float("nan")
        det, tts_fault, det_sim = "", "", ""
        if self.fault_time is not None:
            mrm_ts = [t for s_, t in self.state_times.items()
                      if s_.startswith("MRM_") and t >= self.fault_time - 1.0]
            if mrm_ts:
                det = round(min(mrm_ts) - self.fault_time, 1)
            if self.fault_time_sim is not None:
                mrm_sim = [t for s_, t in self.state_times_sim.items()
                           if s_.startswith("MRM_") and t >= self.fault_time_sim - 1.0]
                if mrm_sim:
                    det_sim = round(min(mrm_sim) - self.fault_time_sim, 2)
            if self.stationary_since is not None:
                tts_fault = round(self.stationary_since - self.fault_time, 1)
        return {"final_state": self.state, "x": round(px, 1), "y": round(py, 1),
                "time_to_stop_s": round(tts, 1) if not math.isnan(tts) else "",
                "det_latency_s": det, "det_latency_sim_s": det_sim, "tts_fault_s": tts_fault,
                "deg_dist_m": round(self.deg_dist, 1) if self.fault_time else "",
                "min_gap_m": round(self.min_gap, 1) if self.min_gap is not None else "",
                "traffic": self.traffic,
                "stall_factor": self.stall_factor,
                "aw_mrm": "+".join(self.aw_behaviors),
                "obs_hard_decel": self.obs.get("obs_hard_decel", ""),
                "obs_pass_events": self.obs.get("obs_pass_events", ""),
                "obs_min_pass_gap_m": self.obs.get("obs_min_pass_gap_m", ""),
                "obs_min_ttc_s": self.obs.get("obs_min_ttc_s", ""),
                "obs_npc_seen": self.obs.get("obs_npc_seen", ""),
                "obs_window_s": self.obs.get("obs_window_s", "")}

    def reset(self):
        self.state = ""
        self.state_times.clear()
        self.state_times_sim.clear()
        self.pose = None
        self.speed = None
        self.stopped_since = None
        self.stationary_since = None
        self.moved = False
        self.aw_state = 0
        self.aw_behaviors = []
        self.op_mode = None
        self.fault_time = None
        self.fault_time_sim = None
        self.deg_dist = 0.0
        self.min_gap = None
        self._last_xy = None
        self._obs_start = None
        self._obs_samples = []
        self.obs = {}
        self.traffic = ""
        self.stall_factor = ""

    def shutdown(self):
        self._spin_alive = False
        try:
            self._spin.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass   # already shut down (Ctrl+C path)


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
        def seg_dist(pt, pts):
            # point-to-POLYLINE distance: straight lanelets have vertices
            # 30 m apart, so nearest-vertex distance wildly overestimates
            best = float("inf")
            for i in range(len(pts) - 1):
                (x1, y1), (x2, y2) = pts[i], pts[i + 1]
                dx, dy = x2 - x1, y2 - y1
                L2 = dx * dx + dy * dy
                t = 0.0 if L2 == 0 else max(0.0, min(1.0,
                    ((pt[0] - x1) * dx + (pt[1] - y1) * dy) / L2))
                best = min(best, math.dist(pt, (x1 + t * dx, y1 + t * dy)))
            return best

        # nearest road lanelet by SEGMENT distance too
        best_lid, best_d = None, float("inf")
        for cand_lid, cand in _MAP.lanelets.items():
            if cand["subtype"] != "road":
                continue
            d = seg_dist((x, y), cand["center"])
            if d < best_d:
                best_lid, best_d = cand_lid, d
        ll = _MAP.lanelets[best_lid]
        lat = best_d
        lpts = _MAP._ways.get(ll["left"], [])
        rpts = _MAP._ways.get(ll["right"], [])
        if len(lpts) < 2 or len(rpts) < 2:
            return "", ""
        width = seg_dist((x, y), lpts) + seg_dist((x, y), rpts)
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


def archive_case_logs(idx, case, dry):
    """케이스 종료 시 cdr/supervisor 로그를 삭제 대신 gzip으로 보관 (v2).

    results/case_logs/<idx>_<method>_<fault>/ 로 이동. --no-log-archive 로 끔."""
    if dry:
        return
    import gzip
    import shutil
    # 실행(run)마다 폴더 구분 — idx만 쓰면 다른 런의 같은 idx가 덮어씀
    # (cfl ablation에서 fixed 런 로그가 dynamic 런에 덮인 사고, 0803)
    run_tag = datetime.now().strftime("%m%d_%H%M")[:9]
    dst = OUT_DIR / "case_logs" / f"{run_tag}_{idx}_{case['method']}_{case['fault']}"
    # autoware.log 포함: 행동계획기 동결(빈 궤적) 부검에 필요 (8/11 1003003 실측 —
    # 다음 케이스가 로그를 잘라 사인 유실)
    for name in ("cdr", "supervisor", "autoware"):
        src = OUT_DIR / f"{name}.log"
        try:
            if not src.exists():
                continue
            dst.mkdir(parents=True, exist_ok=True)
            with open(src, "rb") as fin, \
                    gzip.open(dst / f"{name}.log.gz", "wb") as fout:
                shutil.copyfileobj(fin, fout)
            src.unlink()
        except OSError:
            pass   # 보관 실패가 캠페인을 멈추게 하지 않는다


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

    # v2: 신선도 데드라인 / 동적 FTTI ablation 을 cdr launch 인자로 전달
    # (파라미터 수용은 cdr_monitor 쪽 몫; 러너는 전달만 한다)
    cdr_cmd = CDR_CMD
    if args.fresh_ms is not None:
        cdr_cmd += f" fresh_ms:={args.fresh_ms}"
    if args.variant:
        cdr_cmd += " ttc_dynamic:=" + ("true" if args.variant == "dynamic" else "false")

    with open(csv_path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["idx", "method", "fault", "severity", "trigger_s", "npc", "rep",
                     "outcome", "final_state", "aw_mrm", "stop_x", "stop_y",
                     "lane_overlap_m", "lane_block_pct",
                     "det_latency_s", "det_latency_sim_s", "tts_fault_s", "deg_dist_m", "min_gap_m", "traffic",
                     "time_to_stop_s", "collisions", "collision_types",
                     "stall_factor", "t_hazard_s", "wall_s",
                     "obs_hard_decel", "obs_pass_events", "obs_min_pass_gap_m",
                     "obs_min_ttc_s", "obs_npc_seen", "obs_window_s"])  # v4 관측창
        interrupted = False
        for i, case in enumerate(cases):
            if interrupted:
                break
            # UE4 leaks memory on every load_world; a preemptive restart
            # beats waiting for the slow-then-crash phase
            if i % CARLA_RESTART_EVERY == 0 and not args.dry:
                restart_carla(False)   # case 0 included: never start a
                                       # campaign on an hours-old server
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
                    judge.npc_style = case["npc"]
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
                if not args.mrm_off:
                    procs.host_bg(cdr_cmd, "cdr")
                    baseline = "true" if case["method"] == "baseline" else "false"
                    sup_cmd = SUP_CMD.format(baseline=baseline)
                    if args.no_shoulder:
                        sup_cmd += " demo_no_shoulder:=true"
                    procs.host_bg(sup_cmd, "supervisor")
                else:
                    # --mrm-off 1차 배터리 실패 원인: cdr/supervisor를 아예 안
                    # 띄우면 진단 그래프의 두 유닛이 STALE -> autonomous 잠금
                    # -> 출발 불가. cdr은 정상 기동(점수만 낼 뿐 개입 없음),
                    # supervisor 유닛은 더미 OK 발행으로 그래프만 만족시킨다.
                    procs.host_bg(cdr_cmd, "cdr")
                    # [v5 절제] SUP_SHADOW=1: A1(native)에도 진짜 supervisor를
                    # 섀도(전이만·무행동)로 — 4팔 동일 감지기로 t_detect 동등화.
                    # 섀도가 defer_request OK를 계속 내므로 더미는 생략.
                    if os.environ.get("SUP_SHADOW") == "1":
                        procs.host_bg(
                            "export SUP_SHADOW=1; "
                            + SUP_CMD.format(baseline="false"), "supervisor")
                        _shadow_sup = True
                    else:
                        _shadow_sup = False
                    if not _shadow_sup:
                        procs.host_bg(
                            "export ROS_DOMAIN_ID=0; "
                            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
                            "export CYCLONEDDS_URI=/home/taewook/cyclone_localhost.xml; "
                            "source /opt/ros/humble/setup.bash && "
                            "python3 /home/taewook/autoware_carla_leaderboard/experiments/dummy_defer.py",
                            "dummy_defer")
                # --mrm-off: 모니터·MRM 개입 없이 고장 진행만 관찰 (FTTI 도출)
                # per-unit graph levels — to identify which unit flaps the MRM
                procs.host_bg(DUMP_CMD, "diag_dump")
                lb = procs.docker_bg(LB_CONTAINER,
                                     LEADERBOARD_CMD.format(
                                         faults=fault_args(case, args.dg)),
                                     "leaderboard")
                # [v4.6g] V2X 신호등 릴레이 — agent 프로세스의 rclpy가
                # TrafficLightGroupArray 발행을 wire에 못 싣는 문제(0812)를
                # 독립 프로세스로 우회. 카메라-폴트 케이스는 카메라 인식 유지.
                if case["fault"] not in ("cf", "cn", "cfl", "cfl2", "mix"):
                    procs.docker_bg(
                        LB_CONTAINER,
                        "source /opt/ros/humble/setup.bash && "
                        "source /home/taewook/autoware_carla_leaderboard/install/setup.bash && "
                        "python3 /home/taewook/autoware_carla_leaderboard/experiments/tl_v2x_relay.py",
                        "tl_relay")
                # [v4.9] 엣지 케이스 주입기 (HAZARD=ped=5:22:3 식) — tl_relay와
                # 같은 독립 프로세스 패턴. env 미설정이면 완전 무동작.
                _hz = os.environ.get("HAZARD", "")
                if _hz:
                    _hmode, _hspec = _hz.split("=", 1)
                    procs.docker_bg(
                        LB_CONTAINER,
                        "cd /home/taewook/autoware_carla_leaderboard && "
                        "source /opt/ros/humble/setup.bash && "
                        "source install/setup.bash && "
                        "source carla_envs.sh >/dev/null 2>&1; "
                        # --spec=값 형태 필수: shoulder 좌표가 음수로 시작하면
                        # argparse가 옵션으로 오파싱 (0815 실측: 블로커 전원 미스폰)
                        # until 래퍼: 케이스 초반 월드 로드 레이스에서 CARLA 클라가
                        # C++ terminate로 즉사(0816 실측: 로그 0바이트) — 성공(정상
                        # 종료)까지 2 s 간격 재기동. cleanup pkill이 래퍼도 잡는다.
                        # [sd 0827] 오케스트레이터가 CASE_TAG를 미리 주면 존중
                        # (docker_bg -e 패스스루) — 시드x조건 배치의 산출물 귀속용
                        f"export CASE_TAG=${{CASE_TAG:-{case['fault']}-{case['sev']}-"
                        f"{case['method']}{'-off' if args.mrm_off else ''}-r{case['rep']}}}; "
                        "until python3 experiments/hazard_injector.py "
                        f"--mode={_hmode} --spec={_hspec} --fault-t={case['t']}; "
                        "do sleep 2; done",
                        "hazard")
                if not ego_alive(args.dry):
                    raise RuntimeError("ego odometry never appeared")
                if args.dry:
                    outcome, detail = "DRY", {}
                elif args.mrm_off:
                    outcome, detail = judge.outcome_hazard(t0, lb)
                else:
                    outcome, detail = judge.outcome(t0, lb)
            except KeyboardInterrupt:
                outcome, detail = "INTERRUPTED", (judge._detail() if judge else {})
                interrupted = True
            except Exception as e:
                outcome, detail = "SETUP_FAIL", {"final_state": str(e)[:60]}
            finally:
                procs.kill_all()
                # 로그 보관은 기본 켜짐; --mrm-off 땐 cdr/supervisor 자체가 없음
                if not args.no_log_archive and not args.mrm_off:
                    archive_case_logs(i, case, args.dry)

            wall = round(time.time() - t0, 1)
            n_coll, coll_types = parse_infractions()
            sx, sy = detail.get("x", ""), detail.get("y", "")
            ov, pct = lane_occupancy(sx, sy) if isinstance(sx, float) and not math.isnan(sx) else ("", "")
            row = [i, case["method"], case["fault"], case["sev"], case["t"],
                   case["npc"], case["rep"],
                   outcome, detail.get("final_state", ""), detail.get("aw_mrm", ""),
                   sx, sy, ov, pct,
                   detail.get("det_latency_s", ""), detail.get("det_latency_sim_s", ""), detail.get("tts_fault_s", ""),
                   detail.get("deg_dist_m", ""), detail.get("min_gap_m", ""),
                   detail.get("traffic", ""),
                   detail.get("time_to_stop_s", ""),
                   n_coll, coll_types,
                   detail.get("stall_factor", ""), detail.get("t_hazard_s", ""),
                   wall,
                   detail.get("obs_hard_decel", ""), detail.get("obs_pass_events", ""),
                   detail.get("obs_min_pass_gap_m", ""), detail.get("obs_min_ttc_s", ""),
                   detail.get("obs_npc_seen", ""), detail.get("obs_window_s", "")]
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
    ap.add_argument("--fault", choices=FAULT_KINDS + EXTRA_FAULT_KINDS,
                    help="fault kind (smoke: sets it; full run: filters matrix; "
                         "imb/rain/gb are v2-only and run only when named here)")
    ap.add_argument("--sev", choices=SEV_LEVELS,
                    help="severity (smoke: sets it; full run: filters matrix)")
    ap.add_argument("--t", type=int,
                    help="fault trigger time [s] (smoke: sets it; full run: filters)")
    ap.add_argument("--npc", choices=NPC_STYLES + EXTRA_NPC_STYLES,
                    help="npc style (smoke: sets it; full run: filters matrix); "
                         "none/low/high = 밀도 조건 (매트릭스 밖)")
    ap.add_argument("--reps", type=int,
                    help=f"repetitions per cell (default {REPEATS})")
    ap.add_argument("--limit", type=int,
                    help="run only N cases, sampled evenly across the matrix")
    ap.add_argument("--dg", default="-102:95:-8",
                    help='final goal: default "-102:95:-8" — 354 m route through '
                         'the tagged-shoulder corridor (center road west, then '
                         'north); "1" = route\'s own destination, '
                         '"x:y[:yaw_deg]" custom, "" full waypoints')
    # ---------------- 재실험 v2 옵션 (기본값이면 v1과 동작 동일)
    ap.add_argument("--fresh-ms", type=int, choices=[500, 850, 1000],
                    help="cdr 신선도 데드라인 [ms] — cdr launch에 fresh_ms:= 전달")
    ap.add_argument("--variant", choices=["fixed", "dynamic"],
                    help="동적 FTTI ablation — cdr launch에 ttc_dynamic:= 전달")
    ap.add_argument("--mrm-off", action="store_true",
                    help="FTTI 도출 모드: 모니터/MRM 없이 고장->위험사건 시간 기록")
    ap.add_argument("--retry-failed", metavar="CSV",
                    help="기존 결과 CSV의 CARLA_DIED/SETUP_FAIL 케이스만 재실행")
    ap.add_argument("--no-shoulder", action="store_true",
                    help="demo: exclude shoulder candidates so a parking lot wins")
    ap.add_argument("--no-log-archive", action="store_true",
                    help="케이스별 cdr/supervisor 로그 gzip 보관을 끈다")
    args = ap.parse_args()
    try:
        if args.dg and args.dg not in ("", "1"):
            _p = args.dg.split(":")
            DEST_XY = (float(_p[0]), float(_p[1]))
    except (ValueError, IndexError):
        DEST_XY = None
    run(args)
