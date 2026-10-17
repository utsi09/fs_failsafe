#!/bin/bash
# v4.6d 최종: 카운트 샤프커브 일원화 + far/섹터 감지기 폐기(로그만) + 추종가드 0.95
#  1) nf      : 오탐 없이 완주 (필수 통과 조건)
#  2) lso mid : 카운트 0.91 -> score 0.49 -> 게이트 통과 -> 이동 MRC 성공
#  3) lso high: 카운트 ~0.78 -> score ~0.07 -> DEFER
#  4) lcl mid : 감지 유지 (클러터+시그마 채널) -> DEFER
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in "nf mid" "lso mid" "lso high" "lcl mid"; do
  set -- $f
  echo "=== VERIFY fault=$1 sev=$2 ==="
  timeout 900 python3 experiments/experiment_runner.py \
    --method ours --fault $1 --sev $2 --npc none --reps 1 --t 20 \
    >> experiments/results/v46d_verify.log 2>&1
  tail -1 $(ls -t experiments/results/results_*.csv | head -1)
done
echo V46D_VERIFY_DONE
