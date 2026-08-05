#!/bin/bash
# 무효 2단계 재실행: gb(브리지 mute 수정) + MRM-off(더미 defer 수정)
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"

echo "=== 재실행 A: gb GNSS 블랙아웃 (10) $(date +%T) ==="
$R --fault gb --npc agg --reps 5

echo "=== 재실행 B: MRM-off FTTI 도출 (25) $(date +%T) ==="
$R --mrm-off --method ours --npc agg --reps 5 --fault ln --sev high
$R --mrm-off --method ours --npc agg --reps 5 --fault gn --sev high
$R --mrm-off --method ours --npc agg --reps 5 --fault cf
$R --mrm-off --method ours --npc agg --reps 5 --fault lf
$R --mrm-off --method ours --npc agg --reps 5 --fault imb --sev high

echo "=== 재실행 완료 $(date +%T) ==="
