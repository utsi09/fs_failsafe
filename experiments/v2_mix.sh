#!/bin/bash
# 복합(동시) 고장 실험: 라이다 0.3 + 카메라 15 동시 주입, ours/baseline x agg x 5반복
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
echo "=== 복합 고장 mix (10) $(date +%T) ==="
python3 experiments/experiment_runner.py --fault mix --npc agg --reps 5
echo "=== 복합 고장 완료 $(date +%T) ==="
