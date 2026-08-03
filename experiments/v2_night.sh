#!/bin/bash
# 야간 3종: A 비단조 기제(녹화 동반) / B 제2 경로 재현 / C 오경보 표본 확대
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"
ROSENV="export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/home/taewook/cyclone_localhost.xml; source ~/autoware_awcl/autoware/install/setup.bash"

echo "=== A. 비단조 기제 (녹화기 가동, ln mid/high 각 4) $(date +%T) ==="
bash -lc "$ROSENV && python3 experiments/mech_recorder.py" &
REC=$!
$R --fault ln --sev mid  --method ours --npc agg --reps 4
$R --fault ln --sev high --method ours --npc agg --reps 4
kill $REC 2>/dev/null

echo "=== B. 제2 경로 (북동, dg=60.6:133.9:-64) 스모크 $(date +%T) ==="
$R --smoke --fault nf --method ours --npc agg --dg=60.6:133.9:-64
echo "=== B. 제2 경로 본실험 (ln high/gn mid/cf 각 5) $(date +%T) ==="
$R --fault ln --sev high --method ours --npc agg --reps 5 --dg=60.6:133.9:-64
$R --fault gn --sev mid  --method ours --npc agg --reps 5 --dg=60.6:133.9:-64
$R --fault cf            --method ours --npc agg --reps 5 --dg=60.6:133.9:-64

echo "=== C. 오경보 표본 확대 (nf 50) $(date +%T) ==="
$R --fault nf --method ours --reps 25

echo "=== 야간 체인 완료 $(date +%T) ==="
