#!/bin/bash
# 동적 감지예산 이득 분리 실험: TTC<0.9s 순간 카메라 파괴, 고정 vs 동적 각 8회
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"
echo "=== cfl2 고정 850ms (8) $(date +%T) ==="
$R --variant fixed   --fault cfl2 --method ours --npc agg --reps 8
echo "=== cfl2 동적 (8) $(date +%T) ==="
$R --variant dynamic --fault cfl2 --method ours --npc agg --reps 8
echo "=== cfl2 완료 $(date +%T) ==="
