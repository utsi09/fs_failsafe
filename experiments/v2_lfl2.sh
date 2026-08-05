#!/bin/bash
# 칼날 시나리오: 선행차 급제동 + TTC<1.4s 전방 라이다 파괴 — 고정 vs 동적 결과 차이
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"
echo "=== lfl2 스모크 (고정/동적 각 1) $(date +%T) ==="
$R --smoke --variant fixed   --fault lfl2 --method ours --npc agg --t 30
$R --smoke --variant dynamic --fault lfl2 --method ours --npc agg --t 30
echo "=== lfl2 고정 850ms (8) $(date +%T) ==="
$R --variant fixed   --fault lfl2 --method ours --npc agg --reps 8
echo "=== lfl2 동적 (8) $(date +%T) ==="
$R --variant dynamic --fault lfl2 --method ours --npc agg --reps 8
echo "=== lfl2 완료 $(date +%T) ==="
