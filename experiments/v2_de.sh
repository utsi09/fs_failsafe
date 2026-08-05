#!/bin/bash
# D/E 재실행 (트리거 1.8s 상향판): cfl2 스모크 게이트 + 8x2, lfl2 스모크 + 8x2
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"

echo "=== D0. cfl2 트리거 스모크 $(date +%T) ==="
$R --smoke --fault cfl2 --method ours --npc agg --t 28
echo "=== D. cfl2 고정 (8) $(date +%T) ==="
$R --variant fixed   --fault cfl2 --method ours --npc agg --reps 8
echo "=== D. cfl2 동적 (8) $(date +%T) ==="
$R --variant dynamic --fault cfl2 --method ours --npc agg --reps 8

echo "=== E0. lfl2 스모크 (고정/동적 각1) $(date +%T) ==="
$R --smoke --variant fixed   --fault lfl2 --method ours --npc agg --t 30
$R --smoke --variant dynamic --fault lfl2 --method ours --npc agg --t 30
echo "=== E. lfl2 고정 (8) $(date +%T) ==="
$R --variant fixed   --fault lfl2 --method ours --npc agg --reps 8
echo "=== E. lfl2 동적 (8) $(date +%T) ==="
$R --variant dynamic --fault lfl2 --method ours --npc agg --reps 8
echo "=== D/E 완료 $(date +%T) ==="
