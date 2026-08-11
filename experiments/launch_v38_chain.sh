#!/bin/bash
# v3.3 배터리 체인: Arm B(주차 270) -> Arm A(본 990) — 버그 픽스 후 재시작
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in gn ln cn imn nf; do
  python3 experiments/experiment_runner.py --method both --fault $f --reps 5 --no-shoulder \
    >> experiments/results/v38_armB_parking.log 2>&1
done
python3 experiments/experiment_runner.py --method both \
  > experiments/results/v38_armA_main.log 2>&1
