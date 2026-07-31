#!/usr/bin/env python3
"""자동 고장주입(fault-injection) 실험의 "총감독" 스크립트 (호스트에서 실행).

┌──────────────────────────────────────────────────────────────────────┐
│  전체 구조 (한 케이스 = 한 번의 주행 실험)                            │
│                                                                      │
│  build_cases()  ← 실험 조건들의 조합표(케이스 행렬)를 미리 다 만든다  │
│       │                                                              │
│       ▼                                                              │
│  ┌─ 케이스 루프 (run() 안의 for문) ────────────────────────────┐     │
│  │ 1. CARLA 서버 재시작 (매 케이스 새 세상으로 리셋)            │     │
│  │ 2. kill_all()  ← 이전 케이스의 잔여 프로세스 전부 청소       │     │
│  │ 3. 컨테이너/호스트 프로그램들을 순서대로 켠다:               │     │
│  │      bridge(도커) → Autoware(도커, 준비될 때까지 대기)       │     │
│  │      → cdr_monitor + mrm_supervisor(호스트)                  │     │
│  │      → leaderboard(도커, 고장 조건을 환경변수로 전달)        │     │
│  │ 4. Judge가 ROS 토픽을 구독하며 "이 케이스가 어떻게 끝났나"   │     │
│  │    를 판정 (갓길 성공? 차로 위 정지? 멈춤? 타임아웃?)        │     │
│  │ 5. 판정 + 각종 지표를 CSV에 한 줄 기록                       │     │
│  │ 6. kill_all()  ← 전부 죽이고 다음 케이스로                   │     │
│  └──────────────────────────────────────────────────────────────┘     │
│                                                                      │
│  전제: CARLA 서버는 호스트에서 이미 돌고 있다 (pgrep으로 확인;        │
│  죽으면 러너가 알아서 재시작을 시도하고, 복구 불가면 멈춘다).         │
└──────────────────────────────────────────────────────────────────────┘

사용법:
  python3 experiments/experiment_runner.py --smoke        # 빠른 1케이스 테스트
  python3 experiments/experiment_runner.py                # 전체 행렬 실행
  python3 experiments/experiment_runner.py --dry          # 명령만 출력, 실행 안 함
"""
import argparse
import csv
import itertools
import math
import os
import signal          # 프로세스에 "죽어라" 신호(SIGKILL 등)를 보낼 때 사용
import subprocess      # 파이썬에서 터미널 명령(docker, pkill 등)을 실행하는 표준 모듈
import threading       # Judge의 구독 콜백을 돌리는 별도 일꾼 스레드용
import time
from datetime import datetime
from pathlib import Path

REPO = Path("/home/taewook/autoware_carla_leaderboard")
RESULTS_JSON = REPO / "simulation_results.json"   # leaderboard가 남기는 채점 결과
OUT_DIR = REPO / "experiments" / "results"        # CSV와 로그가 쌓이는 곳

# 도커 컨테이너 이름들. "docker exec <이름> <명령>" 형태로
# 이미 떠 있는 컨테이너 안에서 명령을 실행할 때 쓴다.
LB_CONTAINER = "aw_carla_leaderboard_0916"   # leaderboard + bridge가 도는 컨테이너
AW_CONTAINER = "autoware_awcl_01"            # Autoware가 도는 컨테이너

# 아래 *_CMD 들은 "그 컨테이너/호스트 터미널에서 사람이 직접 쳤을 명령"을
# 문자열로 미리 적어둔 것. bash -lc 로 넘겨서 실행한다.
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
                   "./run_leaderboard.sh {faults}")   # {faults}에 고장 조건이 들어감
CDR_CMD = "cd ~/fs_ws && source install/setup.bash && ros2 launch cdr_monitor cdr_monitor_awcl.launch.py"
DUMP_CMD = ("source ~/autoware_awcl/autoware/install/setup.bash && "
            "ros2 run autoware_diagnostic_graph_utils dump_node")
SUP_CMD = ("cd ~/fs_ws && source install/setup.bash && "
           "ros2 launch mrm_supervisor mrm_supervisor.launch.py baseline:={baseline}")

AUTOWARE_READY_NODE = "behavior_path_planner"   # 이 노드가 보이면 Autoware 준비 완료로 간주
AUTOWARE_READY_TIMEOUT = 120.0
CASE_TIMEOUT_S = 420.0          # 케이스당 절대 상한(초). 이 이상 걸리면 강제 종료
STOPPED_SPEED = 0.15            # 이 속도(m/s) 미만이면 "정지"로 판정
DEFER_STOP_CONFIRM_S = 30.0     # DEFER 상태 + 이만큼 정지 지속 → 케이스 종료
# Town07 신호등: 약 45초 주기 중 초록불 약 10초. 초록불 한 번 놓치는 건
# 봐주지만, NORMAL 상태로 두 주기를 통째로 서 있으면 순정 스택이
# 신호등을 놓친 것(인식 실패) → 빨리 실패 처리하고 다음 케이스로
JUNCTION_STUCK_S = 100.0
# Town07 교차로는 ~45초 주기에 초록불 ~10초라, 갓길로 가는 길에
# 정상적인 빨간불 대기만으로도 60초를 넘길 수 있다 —
# MRM_MOVE 상태에는 세 주기를 다 기다려 준 뒤에야 "막힘"으로 판정
MRM_MOVE_STUCK_S = 150.0
# 매 케이스 CARLA를 새로 켜는 이유: UE4 서버는 load_world를 할 때마다
# 메모리가 새서(leak) 점점 느려진다 (부팅 57초 → 케이스 3-4쯤 150초+,
# 시뮬 속도가 떨어지면 3초짜리 진단 타임아웃이 다시 널뛰어 판정을 오염).
# 재시작 ~1분이 망친 7분짜리 케이스보다 싸고, 케이스마다 새 세상이면
# 남은 액터(NPC 차량 등)도 없어서 실험 위생상으로도 더 낫다.
CARLA_RESTART_EVERY = 1

# ---------------------------------------------------------------- 케이스 행렬
# 고장 종류별 심각도 값 (단위가 제각각: ln은 미터, cn은 픽셀 시그마 0-255,
# imn은 자이로 rad/s, gn은 EKF 위치 미터)
SEVERITY = {
    "ln":  {"mid": 0.3,  "high": 0.6},
    "cn":  {"mid": 15.0, "high": 30.0},
    "imn": {"mid": 0.05, "high": 0.15},
    "gn":  {"mid": 1.0,  "high": 2.5},
}
SEV_LEVELS = ["mid", "high"]
# ln: 라이다 2개 모두 노이즈; cn: 카메라 노이즈; imn: IMU 노이즈;
# gn: GNSS/EKF 위치 노이즈; cf: 카메라 파괴; lf: 전방 라이다 파괴;
# nf: 고장 없음(대조군)
FAULT_KINDS = ["ln", "cn", "imn", "gn", "cf", "lf", "nf"]
NO_SEVERITY = {"cf", "lf", "nf"}  # 심각도 차원이 없는 고장들 (nf는 트리거도 없음)
# 고장 시점은 행렬의 축이 아니라 "무작위 공변량": 트리거 시각을 고정하면
# 고장이 항상 같은 지점에서 터져서 supervisor가 늘 같은 갓길만 골랐다.
# 케이스별 t는 셀 키 문자열에서 결정론적으로 뽑고(재현 가능;
# ours/baseline이 TM_SEED처럼 같은 값을 공유) CSV에 기록한다.
TRIGGER_RANGE = (10, 45)   # ~85초 주행 중 마지막 ~25초는 goal_planner 구간이라 제외
NPC_STYLES = ["def", "agg"]          # 주변 차량 스타일: 기본 / 공격적
METHODS = ["ours", "baseline"]       # baseline = 순정 Autoware MRM (차로 내 정지)
REPEATS = 15                         # 셀당 반복 횟수


def build_cases(args):
    """실험 조건들의 모든 조합(케이스 리스트)을 미리 만들어 돌려준다."""
    methods = METHODS if args.method == "both" else [args.method]
    if args.smoke:
        # smoke 모드: 옵션으로 지정한(없으면 기본값) 케이스 하나만 빠르게
        return [{"method": m,
                 "fault": args.fault or "ln",
                 "sev": args.sev or "mid",
                 "t": args.t if args.t is not None else 40,
                 "npc": args.npc or "def", "rep": 0} for m in methods]
    import random
    cases = []
    # itertools.product = 모든 조합을 도는 다중 for문의 축약형.
    # 순서가 (fault, sev, npc, rep, method)인 게 핵심:
    # 마지막 축(method)이 "가장 안쪽 루프"라서 ours 케이스 바로 다음에
    # 같은 조건(fault/sev/npc/rep 동일)의 baseline 케이스가 붙는다.
    # → 캠페인이 중간에 끊겨도 이미 실행된 부분은 ours/baseline 짝이
    #   완결되어 있어, 짝지은(paired) 비교 데이터가 보존된다.
    for fault, sev, npc, rep, method in itertools.product(
            FAULT_KINDS, SEV_LEVELS, NPC_STYLES,
            range(args.reps if args.reps is not None else REPEATS), methods):
        if fault in NO_SEVERITY and sev != "mid":
            continue  # 파괴형 고장은 심각도 차원이 없다 (mid 하나만 남김)
        # 고장 시점 t: random.Random(시드)로 "시드가 같으면 항상 같은 값"이
        # 나오는 난수 생성기를 만든다. 시드가 셀 키 문자열이므로
        # 같은 케이스는 언제 다시 돌려도 같은 t(재현 가능)이고,
        # method는 시드에 안 들어가므로 ours/baseline이 같은 t를 공유한다.
        t = (args.t if args.t is not None else
             random.Random(f"{fault}/{sev}/{npc}/{rep}").randint(*TRIGGER_RANGE))
        # 전체 실행에서는 같은 커맨드라인 플래그들이 "행렬 필터"로 동작
        if args.fault and fault != args.fault:
            continue
        if args.sev and sev != args.sev:
            continue
        if args.npc and npc != args.npc:
            continue
        cases.append({"method": method, "fault": fault, "sev": sev, "t": t,
                      "npc": npc, "rep": rep})
    if args.limit:
        # 소규모 점검 실행: 앞에서부터 N개를 자르면 전부 같은
        # method/fault만 나오므로, 행렬 전체에 걸쳐 골고루 뽑는다
        step = max(1, len(cases) // args.limit)
        cases = cases[::step][:args.limit]
    return cases


def fault_args(case, dg="1"):
    """케이스 딕셔너리를 run_leaderboard.sh에 넘길 인자 문자열로 변환."""
    if case["fault"] == "nf":
        args = ""                                   # 대조군: 고장 없음
    elif case["fault"] in NO_SEVERITY:
        args = f"{case['fault']}={case['t']}"       # 예: "cf=23" (시점만)
    else:
        # 예: "ln=23:0.3" (시점:심각도)
        args = f"{case['fault']}={case['t']}:{SEVERITY[case['fault']][case['sev']]}"
    # 반복(rep)별 교통 시뮬레이터(Traffic Manager) 시드:
    # ours와 baseline은 같은 rep이면 같은 시드 → 같은 교통 흐름을 본다
    # (짝지은 실험). rep이 다르면 시드가 달라 교통이 달라진다.
    # 이게 없으면 config의 시드 0 때문에 모든 반복이 똑같은 교통이 된다.
    args += f" TM_SEED={100 + case['rep']}"
    if case["npc"] == "agg":
        args += " npc=agg"
    # ego 뒤에 반드시 따라붙는 추격 차량 (MRM 정지 시 후방 추돌 압박용)
    args += " tail=10"
    if dg:
        # direct-goal: 출발지 → 최종 목적지만 주고 중간 경유점은 생략
        # (잘 동작했던 7/28 데모와 같은 설정). dg="" 이면 비활성화.
        args += f" dg={dg}"
    return args


# ---------------------------------------------------------------- 프로세스 관리
class Procs:
    """백그라운드 프로세스들을 켜고(spawn) 끄는(kill_all) 담당 클래스.

    subprocess.Popen = "명령을 실행하고 끝나기를 기다리지 않고 바로 리턴"
    (백그라운드 실행). 리턴된 Popen 객체로 나중에 생사 확인/종료가 가능.
    """

    def __init__(self, dry):
        self.dry = dry                # --dry 모드면 실제로 아무것도 안 켬
        self.host_procs = []          # (이름, Popen객체, 로그파일핸들) 목록

    def docker_bg(self, container, cmd, name, user=None):
        """도커 컨테이너 안에서 cmd를 백그라운드로 실행.

        docker exec = 이미 떠 있는 컨테이너 안에 들어가 명령을 실행하는 것.
        -e ROS_DOMAIN_ID=0: 컨테이너에 구워진 환경변수와 무관하게 도메인 0 강제
        (ROS2는 도메인 번호가 같아야 서로 통신함).
        -u: 수동으로 (잘 동작하던) 터미널에서 쓰던 사용자와 맞춤.
        """
        full = ["docker", "exec", "-e", "ROS_DOMAIN_ID=0"]
        if user:
            full += ["-u", user]
        # bash -lc "명령": 로그인 셸을 열어 문자열 명령을 실행 (source가 필요해서)
        full += [container, "bash", "-lc", cmd]
        return self._spawn(full, name)

    def host_bg(self, cmd, name):
        """호스트(내 컴퓨터)에서 cmd를 백그라운드로 실행."""
        # DDS(ROS2의 통신 계층) 구현체는 컨테이너들과 같아야 한다(CycloneDDS):
        # 서로 다른 벤더끼리는 토픽은 대충 되는데 서비스 호출이
        # 응답 없이 영원히 멈춰버린다.
        env = ("export ROS_DOMAIN_ID=0; "
               "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
               "export CYCLONEDDS_URI=/home/taewook/cyclone_localhost.xml; ")
        return self._spawn(["bash", "-lc", env + cmd], name)

    def _spawn(self, argv, name):
        """실제 실행부: 로그 파일을 열고 Popen으로 백그라운드 시작."""
        print(f"[runner] start {name}: {' '.join(argv[:3])} ...")
        if self.dry:
            return None
        # "wb"(덮어쓰기 모드): 현재 케이스의 로그만 남긴다 — 1000+ 케이스
        # 캠페인 동안 이어쓰기(append)하면 파일이 수 GB로 자란다.
        # 파일 핸들은 kill_all에서 닫는다; 안 닫으면 케이스당 fd(파일
        # 디스크립터)가 ~5개씩 새서 결국 EMFILE(더 못 엶) 에러가 난다.
        # 직전 케이스 로그는 .prev로 이름을 바꿔 사후 분석용으로 보존.
        path = OUT_DIR / f"{name}.log"
        if path.exists():
            path.replace(OUT_DIR / f"{name}.log.prev")
        log = open(path, "wb")
        # preexec_fn=os.setsid: 자식을 "새 프로세스 그룹의 리더"로 만든다.
        # 나중에 killpg(그룹 전체 죽이기)로 자식의 자식까지 한 방에 정리 가능.
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        self.host_procs.append((name, p, log))
        return p

    def kill_all(self):
        """켰던 것 + 잔여물 전부를 과격하게(-9) 청소한다.

        왜 이렇게 과격한가:
        - docker exec로 켠 프로세스는 호스트 쪽 클라이언트만 죽어서는
          컨테이너 안의 실제 프로세스가 안 죽는다 (신호가 컨테이너 안으로
          전달되지 않음). 그래서 컨테이너 안에서 pkill을 따로 돌려야 한다.
        - pkill -9(SIGKILL)는 "정리할 틈도 없이 즉사"라, 죽은 부모의
          자식 프로세스는 고아(orphan)로 살아남는다. 그 고아들이 다음
          케이스를 오염시키므로(아래 dump_node 사건) 패턴별로 다 잡는다.
        - 로그/임시 파일도 케이스마다 지운다 (안 지우면 용량 폭발).
        """
        print("[runner] cleanup...")
        if self.dry:
            return
        # 컨테이너 안쪽 프로세스 트리 청소 (docker exec 클라이언트는 신호를
        # 컨테이너 안으로 전달하지 못하므로 컨테이너 안에서 직접 pkill).
        # "ros-args" 패턴은 모든 ROS 노드 바이너리에 걸린다 — launch만
        # 죽이면 좀비 component_container가 남고, 그 중복된 노드 이름들이
        # DDS 디스커버리를 깨뜨린다 (실측: Autoware 4겹 적층 → 맵이 영영 안 뜸).
        for container, pats in [
            (LB_CONTAINER, ["ros-args", "leaderboard_evaluator", "ros2 launch",
                            "run_leaderboard"]),
            (AW_CONTAINER, ["ros-args", "component_container", "robot_state_publisher",
                            "rviz2", "ros2 launch"]),
        ]:
            for pat in pats:
                subprocess.run(["docker", "exec", container, "pkill", "-9", "-f", pat],
                               capture_output=True)
        # 호스트 쪽 (cdr / supervisor / 떠도는 launch들)
        # dump_node 포함 이유: 고아가 된 dump 인스턴스가 노드 이름을 중복시켜
        # duplicated_node_checker ERROR → 모든 모드 사용 불가 → 영구 비상정지.
        # (0731에 출발을 막고 있던 고아 3마리를 실제로 발견)
        for pat in ["cdr_monitor", "mrm_supervisor", "awcl_adapter", "ros2 launch",
                    "diagnostic_graph_utils dump_node"]:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        # 위 패턴은 `ros2 run` 래퍼(포장 프로세스)에만 걸린다; 래퍼를 -9로
        # 죽이면 진짜 dump_node 바이너리가 고아가 되고 (경로에 공백이 아닌
        # 슬래시가 있어 -f 패턴에 안 걸림), 케이스마다 고아가 하나씩 늘며
        # duplicated_node_checker를 재점화한다 → 정확히 이름 일치(-x)로 처형
        subprocess.run(["pkill", "-9", "-x", "dump_node"], capture_output=True)
        # launch 1회당 ROS 로그 디렉터리가 새로 생기고 재사용되지 않는다 —
        # 방치하면 긴 캠페인 동안 수천 개 디렉터리(GB 단위)로 불어난다
        subprocess.run(["docker", "exec", "-u", "aw", AW_CONTAINER, "bash", "-c",
                        "rm -rf /home/aw/.ros/log/* /tmp/launch_params_* 2>/dev/null"],
                       capture_output=True)
        subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-c",
                        "rm -rf /root/.ros/log/* /tmp/launch_params_* 2>/dev/null"],
                       capture_output=True)
        subprocess.run(["bash", "-c", "rm -rf ~/.ros/log/* 2>/dev/null"],
                       capture_output=True)
        # ros2 CLI 데몬 재시작: 죽은 노드들의 디스커버리 캐시가 남지 않도록
        subprocess.run(["bash", "-lc", "ros2 daemon stop"], capture_output=True)
        # 우리가 직접 켠 호스트 프로세스들: setsid로 만든 그룹 전체를 SIGKILL
        for name, p, log in self.host_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass   # 이미 죽었으면 그냥 넘어감
            log.close()   # fd 누수 방지 (위 _spawn 주석 참고)
        self.host_procs.clear()
        if carla_alive():
            carla_unstick(False)   # SIGKILL 여파로 동기모드에 갇힌 CARLA 해제
        time.sleep(3.0)


def carla_alive():
    """CARLA 서버 프로세스가 존재하는지만 확인 (pgrep = 이름으로 프로세스 찾기)."""
    return subprocess.run(["pgrep", "-f", "CarlaUE4"], capture_output=True).returncode == 0


CARLA_DIR = "/home/taewook"          # CarlaUE4.sh가 있는 곳, ./CarlaUE4.sh 로 실행
CARLA_LAUNCH = "cd /home/taewook && ./CarlaUE4.sh --ros2"
CARLA_CHECK_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
                   "source carla_envs.sh >/dev/null 2>&1; "
                   "python3 -c \"import carla; c=carla.Client('localhost',2000); "
                   "c.set_timeout(6.0); print(c.get_server_version())\"")


CARLA_UNSTICK_CMD = ("cd /home/taewook/autoware_carla_leaderboard && "
                     "source carla_envs.sh >/dev/null 2>&1; "
                     "python3 experiments/carla_unstick.py")


def carla_unstick(dry):
    """SIGKILL로 끝난 케이스는 서버를 '동기(synchronous) 모드인데 아무도
    tick을 안 쳐주는' 고아 상태로 남긴다 — 그러면 다음 load_world()가
    영원히 타임아웃. 강제로 비동기 모드로 되돌린다 (carla_unstick.py 참고)."""
    if dry:
        return True
    try:
        r = subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-lc",
                            CARLA_UNSTICK_CMD], capture_output=True, timeout=30)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def carla_responsive(dry):
    """pgrep은 '프로세스가 존재한다'까지만 증명한다; 멈춰버린(hang) 서버도
    통과한다. LB 컨테이너 안의 0.9.16 클라이언트로 서버 버전을 실제로
    물어봐서 '응답까지 하는지' 확인한다."""
    if dry:
        return True
    try:
        r = subprocess.run(["docker", "exec", LB_CONTAINER, "bash", "-lc",
                            CARLA_CHECK_CMD], capture_output=True, timeout=30)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def restart_carla(dry):
    """CARLA 서버를 죽이고 새로 켠 뒤 응답할 때까지 기다린다."""
    print("[runner] CARLA unresponsive — restarting the server")
    if dry:
        return True
    subprocess.run(["pkill", "-9", "-f", "CarlaUE4"], capture_output=True)
    time.sleep(8.0)
    log = open(OUT_DIR / "carla.log", "wb")   # CARLA가 사는 동안 계속 열어둠
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
    """leaderboard가 ego 차량을 스폰할 때까지 대기 (odometry 토픽이 흐르면 OK)."""
    if dry:
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        # ros2 topic echo --once: 메시지 1개만 받고 종료. timeout 4초 안에
        # 1개라도 오면 성공(리턴코드 0) → ego가 살아서 위치를 쏘고 있다는 뜻
        r = subprocess.run(
            ["bash", "-lc",
             "timeout 4 ros2 topic echo --once /localization/kinematic_state >/dev/null 2>&1"],
        )
        if r.returncode == 0:
            return True
        time.sleep(2.0)
    return False


def autoware_ready(dry, timeout=AUTOWARE_READY_TIMEOUT):
    """ros2 node list에 핵심 노드가 나타날 때까지 폴링하며 대기."""
    if dry:
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(["bash", "-lc", "ros2 node list 2>/dev/null"],
                           capture_output=True, text=True)
        if AUTOWARE_READY_NODE in r.stdout:
            time.sleep(5.0)  # 노드가 떴어도 잠깐 안정화 시간을 준다
            return True
        time.sleep(3.0)
    return False


# ---------------------------------------------------------------- 판정(Judge)
class Judge:
    """supervisor 상태 + ego 주행 정보를 구독하며 케이스의 결말을 판정하는 심판.

    ROS2 파이썬(rclpy) 기초:
    - Node를 만들고 create_subscription(타입, 토픽이름, 콜백, QoS)을 걸면
      해당 토픽에 메시지가 올 때마다 콜백 함수가 불린다.
    - 단, 콜백은 저절로 불리지 않는다 — 누군가 spin(펌프질)을 계속 돌려
      "도착한 메시지를 콜백에 배달"해줘야 한다. 여기서는 전용 일꾼
      스레드(_spin_loop)가 그 일을 맡는다.
    - QoS의 TRANSIENT_LOCAL(latched): 늦게 구독해도 마지막 메시지를
      바로 받아볼 수 있는 설정. 상태류 토픽에 쓴다.

    구독 토픽별 판정 재료:
    - /mrm_supervisor/state        : 우리 supervisor의 상태 문자열
                                     (NORMAL / MRM_MOVE / MRM_PARKED / MRM_DEFER)
                                     → 성공/보류 판정의 1순위 근거
    - /localization/kinematic_state: ego의 위치+속도 (정지 여부, 주행거리,
                                     최종 정지 좌표 계산)
    - /system/fail_safe/mrm_state  : 순정 Autoware MRM의 상태/행동
                                     (baseline이 어떤 정지를 했는지)
    - /api/operation_mode/state    : 운전 모드 전환 기록 ("왜 옛 경로를
                                     계속 달렸나" 류의 사후 설명용)
    - /fault_injection/active      : 고장이 실제로 켜진 순간 (모든
                                     "고장 후 ~" 지표의 기준 시각)
    - /monitor/nearest_obstacle_m  : 가장 가까운 장애물 거리 (근접
                                     아차사고 지표 + 교통 흐름 관찰창 재료)
    """

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
        self.state_times = {}          # 상태 → 처음 목격한 벽시계 시각
        self.pose = None
        self.speed = None
        self.stopped_since = None      # (주행 후) 정지가 시작된 시각
        self.stationary_since = None   # (주행 여부 무관) 정지가 시작된 시각
        self.moved = False             # ego가 한 번이라도 제대로 움직였나
        self.aw_state = 0              # mrm_handler가 쏘는 MrmState.state
        self.aw_behaviors = []         # 목격한 행동 이름들 (처음 본 순서)
        self.fault_time = None         # 고장이 켜진 벽시계 시각
        self.deg_dist = 0.0            # 고장 이후 주행 거리 [m]
        self.min_gap = None            # 고장 후 정지 중 가장 가까웠던 장애물 거리
        self._last_xy = None
        self._obs_start = None         # 교통 흐름 관찰창 시작 시각
        self._obs_samples = []
        self.traffic = ""              # 정지 후 교통이 cleared / blocked

        self.node.create_subscription(
            String, "/mrm_supervisor/state", self._state_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_cb, 10)
        self.node.create_subscription(
            MrmState, "/system/fail_safe/mrm_state", self._mrm_cb, 10)
        # 운전 모드 전환 이력은 "옛 경로를 계속 달렸다" 케이스를 설명해준다
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

        # 구독 콜백을 계속 돌려주는 전용 일꾼 스레드.
        # daemon=True: 메인이 끝나면 이 스레드도 같이 사라지게 함
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    def _fault_cb(self, msg):
        # 고장이 처음 켜진 순간만 기록 (이후의 True는 무시)
        if msg.data and self.fault_time is None:
            self.fault_time = time.time()
            print("[judge] fault active")

    def _gap_cb(self, msg):
        # 근접 아차사고(near-miss) 지표: 고장 후 정지 중 가장 가까웠던 물체
        # (차로 내 MRM 정지에 후방 차량이 얼마나 바짝 붙었나)
        if (self.fault_time is not None and self.speed is not None
                and self.speed < 0.5 and msg.data < 900.0):
            if self.min_gap is None or msg.data < self.min_gap:
                self.min_gap = msg.data
        # 관찰창이 열려 있으면 샘플 수집 (900 이상 = "감지 없음" 센티널 값)
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
        # spin_once = "도착해 있는 메시지를 최대 0.2초 기다렸다가 콜백에 배달"
        # 을 무한 반복. 일시적 에러가 나도 계속 돈다 — 스핀 스레드가 죽으면
        # 심판이 장님이 되어(상태 빈 문자열, 위치 nan) 남은 모든 케이스를 망친다
        while getattr(self, "_spin_alive", True) and self.rclpy.ok():
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.2)
            except Exception:
                time.sleep(0.2)

    def _state_cb(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            # setdefault: 그 상태를 "처음 본" 시각만 기록 (재진입은 무시)
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
        # 고장 이후 주행 거리(deg_dist): 위치 변화량을 계속 누적
        if self.fault_time is not None and self._last_xy is not None:
            step = math.dist(self.pose, self._last_xy)
            if step < 5.0:                  # 한 틱에 5m 이상 = 순간이동/리셋 → 무시
                self.deg_dist += step
        self._last_xy = self.pose
        # stationary(정지)는 moved와 무관하게 잰다: 출발도 하기 전(스폰 직후
        # 빨간불)에 고장이 터질 수 있고, 그 케이스도 닫혀야 하기 때문
        if self.speed < STOPPED_SPEED:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None
        if self.speed > 1.0 and not self.moved:
            self.moved = True
            self.aw_behaviors = []   # 출발 전 단계의 MRM 잡음은 기록에서 제거
        # stopped는 "실제 주행을 했다가 멈춘" 경우만 (moved 조건)
        if self.speed < STOPPED_SPEED and self.moved:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None

    def outcome(self, case_t0, lb_proc):
        """종결 조건이 나올 때까지 1초 간격 폴링; (판정, 상세)를 리턴.

        판정 우선순위 (위가 이길수록 먼저 확정):
          1. SHOULDER_SUCCESS / PARKED_SUCCESS  — supervisor가 갓길/주차 완료
          2. AW_*_DONE                          — 순정 Autoware MRM 정지 완료
          3. DEFER_STOPPED                      — supervisor가 보류(DEFER) 상태로 정지
          4. (판정 후에도 바로 안 끝냄: 10초 교통 관찰창 → cleared/blocked)
          그 외: SCENARIO_ENDED / STUCK_* / TIMEOUT / CARLA_DIED
        """
        last_beat = 0.0
        while True:
            now = time.time()
            elapsed = now - case_t0
            if now - last_beat > 30.0:   # 30초마다 생존 신호(하트비트) 출력
                last_beat = now
                print(f"[judge] t+{elapsed:.0f}s state={self.state or '(none)'} "
                      f"speed={self.speed if self.speed is not None else 'no-odom'} "
                      f"mode={self.op_mode or '?'}")
            stopped_for = (now - self.stopped_since) if self.stopped_since else 0.0
            stationary_for = (now - self.stationary_since) if self.stationary_since else 0.0

            # supervisor는 "상태:목표" 형식으로 쏜다. 예: "MRM_PARKED:shoulder_X"
            verdict = None
            if self.state.startswith("MRM_PARKED"):
                verdict = ("SHOULDER_SUCCESS" if "shoulder" in self._target()
                           else "PARKED_SUCCESS")
            # 순정 Autoware MRM 완료 (comfortable/emergency 정지 끝):
            # 뭉뚱그린 "DEFER + 정지" 폴백보다 정보가 많아서 우선한다.
            # moved 조건: 출발 전에는 아직 준비 안 된 스택이 주차된 차를
            # "비상정지"시키고 곧바로 MRM_SUCCEEDED에 도달해버리기 때문.
            # stopped_for 조건: 위험 신호가 깜빡이는(flapping) 상황에선
            # 주행 중에도 MRM_SUCCEEDED가 스쳐 지나간다 — 실제로
            # 지속된 정지가 있어야만 인정
            if (verdict is None
                    and self.aw_state == self.MrmState.MRM_SUCCEEDED and self.moved
                    and stopped_for > 5.0):
                behavior = self.aw_behaviors[-1] if self.aw_behaviors else "UNKNOWN"
                verdict = f"AW_{behavior}_DONE"
            if (verdict is None and self.state.startswith("MRM_DEFER")
                    and stationary_for > DEFER_STOP_CONFIRM_S):
                verdict = "DEFER_STOPPED"
            if verdict is not None:
                # 아직 케이스를 닫지 않는다: ~10초간 뒤 차들이 정지한 ego를
                # 실제로 "지나쳐 가는지" 관찰 (갓길 주차면 cleared,
                # 추격차 딸린 차로 내 정지면 blocked로 읽혀야 함)
                if self._obs_start is None:
                    self._obs_start = now
                    self._obs_samples = []
                    print(f"[judge] {verdict} — observing traffic flow 10 s")
                # poll() = 프로세스가 살아 있으면 None, 죽었으면 종료코드
                lb_dead = lb_proc is not None and lb_proc.poll() is not None
                if now - self._obs_start >= 10.0 or lb_dead:
                    smp = self._obs_samples
                    if smp:
                        gmin = min(smp)                          # 창 안 최소 거리
                        gend = sum(smp[-3:]) / len(smp[-3:])     # 창 끝 무렵 평균 거리
                        # 막힘 판정: 끝나는 시점에도 가까이(<12m) 있고,
                        # 창 내 최소치에서 3m도 못 멀어졌다 = 뒤 차가
                        # 지나가지 못하고 그대로 서 있다는 뜻
                        self.traffic = ("blocked" if gend < 12.0 and gend - gmin < 3.0
                                        else "cleared")
                    return verdict, self._detail()
                time.sleep(1.0)
                continue
            if lb_proc is not None and lb_proc.poll() is not None:
                # 심판이 latched 상태를 놓칠 수 있다 (DDS 벤더가 꼬였을 때
                # 한 번 목격) — supervisor 로그에서 판정을 복구
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
            # NORMAL 상태로 너무 오래 서 있음 = 순정 스택이 신호등을 놓침
            if (self.moved and self.state in ("", "NORMAL")
                    and stationary_for > JUNCTION_STUCK_S):
                return "STUCK_AT_JUNCTION", self._detail()
            # 갓길로 가는 중(MRM_MOVE)인데 너무 오래 정지 = 진짜 막힘
            if self.state.startswith("MRM_MOVE") and stopped_for > MRM_MOVE_STUCK_S:
                return "STUCK_IN_MRM_MOVE", self._detail()
            if elapsed > CASE_TIMEOUT_S:
                return "TIMEOUT", self._detail()
            if not carla_alive():
                return "CARLA_DIED", self._detail()
            time.sleep(1.0)

    def _target(self):
        # 상태 문자열이 미래에 목표를 따로 실을 수도 있음; 지금은 상태 전체로 대체
        return self.state.lower()

    def _detail(self):
        """CSV에 실릴 상세 지표 딕셔너리를 계산한다.

        지표 정의:
        - time_to_stop_s : MRM_MOVE 진입 → 최종 정지까지 걸린 시간
        - det_latency_s  : 고장 발생 → supervisor가 MRM_* 상태로 반응하기까지
                           (탐지 지연)
        - tts_fault_s    : 고장 발생 → 차량이 완전히 멈추기까지 (총 대응 시간)
        - deg_dist_m     : 고장난 채로 달린 거리 (성능저하 주행거리)
        - min_gap_m      : 고장 후 정지 중 가장 가까웠던 장애물 거리 (아차사고)
        """
        px, py = self.pose if self.pose else (float("nan"), float("nan"))
        # 상태 문자열에 목표가 붙는다 ("MRM_MOVE:shoulder_X") — 정확한 키로
        # get() 하면 절대 안 걸리므로 startswith로 최초 진입 시각을 찾는다
        t_move = min((t for s_, t in self.state_times.items()
                      if s_.startswith("MRM_MOVE")), default=None)
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
                "traffic": self.traffic,
                "aw_mrm": "+".join(self.aw_behaviors)}

    def reset(self):
        """다음 케이스를 위해 심판의 기억을 전부 초기화 (노드/구독은 재사용)."""
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
        self._obs_start = None
        self._obs_samples = []
        self.traffic = ""

    def shutdown(self):
        # 스핀 스레드에 종료 신호 → 합류(join) 대기 → rclpy 정리
        self._spin_alive = False
        try:
            self._spin.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass   # 이미 종료됨 (Ctrl+C 경로)


_MAP = None   # 지도는 무거우니 한 번만 로드해서 전역에 캐시


def lane_occupancy(x, y):
    """(겹침폭 m, 차로 차단 %): 최종 정지 지점에서 폭 1.9 m짜리 ego 차체가
    가장 가까운 주행(DRIVING) 차로 안에 얼마나 걸쳐 있는지.
    0 = 완전히 비켜남(갓길), 차로폭의 100% = 완전 차단.
    Town07 갓길은 좁아서(1.6-2.4 m) 부분 겹침이 불가피 — 이걸 이분법이
    아니라 연속값으로 잰다."""
    global _MAP
    try:
        if _MAP is None:
            import sys
            sys.path.insert(0, "/home/taewook/fs_ws/src/mrm_supervisor")
            from mrm_supervisor.map_model import MapModel
            _MAP = MapModel("/home/taewook/awcl_maps/Town07/lanelet2_map.osm")
        def seg_dist(pt, pts):
            # 점-폴리라인(연속 선분들) 거리: 직선 차로는 꼭짓점 간격이
            # 30 m나 되어서, "가장 가까운 꼭짓점까지 거리"로 재면 실제
            # 선까지의 거리를 크게 과대평가한다. 그래서 각 선분에 점을
            # 수직으로 투영(내적/길이² = 투영 비율 t, 0~1로 클램프)해
            # 진짜 최단거리를 구한다.
            best = float("inf")
            for i in range(len(pts) - 1):
                (x1, y1), (x2, y2) = pts[i], pts[i + 1]
                dx, dy = x2 - x1, y2 - y1
                L2 = dx * dx + dy * dy
                t = 0.0 if L2 == 0 else max(0.0, min(1.0,
                    ((pt[0] - x1) * dx + (pt[1] - y1) * dy) / L2))
                best = min(best, math.dist(pt, (x1 + t * dx, y1 + t * dy)))
            return best

        # 가장 가까운 도로 lanelet도 같은 "선분 거리"로 찾는다
        best_lid, best_d = None, float("inf")
        for cand_lid, cand in _MAP.lanelets.items():
            if cand["subtype"] != "road":
                continue
            d = seg_dist((x, y), cand["center"])
            if d < best_d:
                best_lid, best_d = cand_lid, d
        ll = _MAP.lanelets[best_lid]
        lat = best_d                            # 차로 중심선까지의 횡방향 거리
        lpts = _MAP._ways.get(ll["left"], [])
        rpts = _MAP._ways.get(ll["right"], [])
        if len(lpts) < 2 or len(rpts) < 2:
            return "", ""
        # 차로폭 = 좌측 경계까지 거리 + 우측 경계까지 거리
        width = seg_dist((x, y), lpts) + seg_dist((x, y), rpts)
        half_car = 0.95    # 차 폭 1.9 m의 절반
        # 차체 구간 [lat-0.95, lat+0.95] 와 차로 구간 [-폭/2, +폭/2] 의
        # 교집합 길이 = 차로 위에 걸친 폭
        overlap = max(0.0, min(lat + half_car, width / 2) - max(lat - half_car, -width / 2))
        return round(overlap, 2), round(100.0 * overlap / width, 1)
    except Exception:
        return "", ""    # 지도 문제 등으로 실패하면 CSV에 빈칸으로


def parse_infractions():
    """leaderboard 결과 json에서 (총 충돌 수, 유형별 내역)을 뽑는다."""
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


# ---------------------------------------------------------------- 메인 루프
def run(args):
    # 러너 자신(judge + ros2 CLI 폴링)도 컨테이너들과 통신 설정을 맞춘다
    os.environ["ROS_DOMAIN_ID"] = "0"
    # judge도 컨테이너들과 같은 DDS 방언(CycloneDDS)을 써야 한다
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
                     "det_latency_s", "tts_fault_s", "deg_dist_m", "min_gap_m", "traffic",
                     "time_to_stop_s", "collisions", "collision_types", "wall_s"])
        interrupted = False
        for i, case in enumerate(cases):
            if interrupted:
                break
            # UE4는 load_world 때마다 메모리가 샌다; 느려지다 죽는 단계를
            # 기다리느니 선제적으로 재시작하는 편이 낫다 (위 상수 주석 참고)
            if i % CARLA_RESTART_EVERY == 0 and not args.dry:
                restart_carla(False)   # 케이스 0 포함: 몇 시간 묵은 서버로
                                       # 캠페인을 시작하지 않는다
            # 프로세스 존재 + 실제 응답 + 동기모드 해제, 셋 다 통과해야 진행
            if not args.dry and not (carla_alive() and carla_responsive(False)
                                     and carla_unstick(False)):
                if not restart_carla(args.dry):
                    print("[runner] CARLA could not be recovered — stopping.")
                    break
            print(f"\n===== case {i + 1}/{len(cases)}: {case} =====")
            t0 = time.time()
            try:
                procs.kill_all()               # 이전 케이스 잔여물 청소
                if judge:
                    judge.reset()              # 심판 기억 초기화
                RESULTS_JSON.unlink(missing_ok=True)   # 옛 채점 결과 삭제

                # ── 기동 순서 (순서가 중요) ──────────────────────────
                procs.docker_bg(LB_CONTAINER, BRIDGE_CMD, "bridge")
                procs.docker_bg(AW_CONTAINER, AUTOWARE_CMD, "autoware", user="aw")
                if not autoware_ready(args.dry):
                    raise RuntimeError("autoware not ready in time")
                # 모니터들은 leaderboard보다 먼저 켠다: 얘네가 없으면
                # 진단 그래프의 해당 유닛이 stale(= ERROR)로 읽혀서 순정
                # MRM이 널뛰고 출발(engagement)이 막힌다. 기동 초기의
                # 과도 상태는 안전 — cdr은 센서 워밍업 중 OK를 보고하고,
                # supervisor는 건강한 점수를 한 번 보기 전엔 무장 안 함.
                procs.host_bg(CDR_CMD, "cdr")
                baseline = "true" if case["method"] == "baseline" else "false"
                procs.host_bg(SUP_CMD.format(baseline=baseline), "supervisor")
                # 유닛별 그래프 레벨 덤프 — 어느 유닛이 MRM을 널뛰게 하는지 식별용
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
                    outcome, detail = judge.outcome(t0, lb)   # 판정 대기
            except KeyboardInterrupt:
                # Ctrl+C: 지금 케이스는 INTERRUPTED로 기록하고 루프 탈출
                outcome, detail = "INTERRUPTED", (judge._detail() if judge else {})
                interrupted = True
            except Exception as e:
                outcome, detail = "SETUP_FAIL", {"final_state": str(e)[:60]}
            finally:
                procs.kill_all()               # 어떤 경로로 끝났든 반드시 청소

            # ── CSV 한 줄 기록 ──────────────────────────────────────
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
                   detail.get("traffic", ""),
                   detail.get("time_to_stop_s", ""),
                   n_coll, coll_types, wall]
            wr.writerow(row)
            f.flush()   # 버퍼를 즉시 디스크로: 러너가 죽어도 지금까지 행은 보존
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
    ap.add_argument("--limit", type=int,
                    help="run only N cases, sampled evenly across the matrix")
    ap.add_argument("--dg", default="-102:95:-8",
                    help='final goal: default "-102:95:-8" — 354 m route through '
                         'the tagged-shoulder corridor (center road west, then '
                         'north); "1" = route\'s own destination, '
                         '"x:y[:yaw_deg]" custom, "" full waypoints')
    args = ap.parse_args()
    run(args)
