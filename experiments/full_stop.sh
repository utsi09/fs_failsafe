#!/bin/bash
# 배터리 완전 정지: 워처 -> 체인 -> 러너 -> 호스트 ROS -> 컨테이너 고아 -> CARLA
# (부분 정지가 만든 사고들: CARLA 중복(load 60), LB 고아(TM 포트 점유),
#  AW 고아(RViz 이중, duplicated_node) — 반드시 이 스크립트로만 중단할 것)
pkill -f "armC_waiter" 2>/dev/null
for pat in launch_v3 launch_repro shoot_lot reshoot2; do
  pgrep -f "$pat" | xargs -r kill -9 2>/dev/null
done
pgrep -f "experiment_runner.py" | xargs -r kill -9 2>/dev/null
# 호스트측 cdr/supervisor/diag
pgrep -f "cdr_monitor" | xargs -r kill -9 2>/dev/null
pgrep -f "mrm_supervisor" | xargs -r kill -9 2>/dev/null
pgrep -f "dump_node" | xargs -r kill -9 2>/dev/null
# LB 컨테이너 고아
docker exec aw_carla_leaderboard_0916 bash -c \
  "pgrep -f 'leaderboard_evaluator|video_once' | xargs -r kill -9" 2>/dev/null
# AW 컨테이너 고아 (rviz 포함 전체 ROS)
docker exec autoware_awcl_01 bash -c \
  "pgrep -f 'rviz2|ros2 launch|component_container|robot_state_publisher|autoware' | xargs -r kill -9" 2>/dev/null
sleep 2
pgrep -f "CarlaUE4" | xargs -r kill -9 2>/dev/null
sleep 2
echo "host runner: $(pgrep -fc experiment_runner.py || echo 0)"
echo "carla: $(pgrep -fc CarlaUE4 || echo 0)"
echo "aw rviz: $(docker exec autoware_awcl_01 bash -c 'pgrep -c rviz2' 2>/dev/null || echo 0)"
echo "lb eval: $(docker exec aw_carla_leaderboard_0916 bash -c 'pgrep -fc leaderboard_evaluator' 2>/dev/null || echo 0)"
echo FULL_STOP_DONE
