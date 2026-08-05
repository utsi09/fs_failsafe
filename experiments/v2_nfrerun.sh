#!/bin/bash
# v2.1 침묵감쇠(시뮬시간 기준) 재검증: gb 스모크 1(진짜 침묵 여전히 잡히나) + nf 50
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"
echo "=== gb 회귀 스모크 $(date +%T) ==="
$R --smoke --fault gb --method ours --npc agg --t 25
echo "=== nf 오탐 재검증 50 $(date +%T) ==="
$R --fault nf --method ours --reps 25
echo "=== v2.1 재검증 완료 $(date +%T) ==="
