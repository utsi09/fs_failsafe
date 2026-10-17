#!/bin/bash
# v4.6b 재검증: cdr 신형 감지기 + 전방섹터 게이트 + lso 재보정
#  1) lso mid : 섹터 결손 감지 -> band 진입, 전방섹터 ~0.5 -> 게이트 통과, 이동 MRC 성공 기대
#  2) lso high: 전방섹터 ~0.2 -> 게이트 DEFER 기대
#  3) lcl mid : 신형 감지기(클러터/원거리) 경유로도 감지 유지 확인 (DEFER 기대)
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in "lso mid" "lso high" "lcl mid"; do
  set -- $f
  echo "=== VERIFY fault=$1 sev=$2 ==="
  timeout 900 python3 experiments/experiment_runner.py \
    --method ours --fault $1 --sev $2 --npc none --reps 1 --t 20 \
    >> experiments/results/v46b_verify.log 2>&1
  tail -1 $(ls -t experiments/results/results_*.csv | head -1)
done
echo V46B_VERIFY_DONE
