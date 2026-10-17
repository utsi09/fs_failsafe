#!/bin/bash
# v4.6c 최종 검증: 섹터-far 감지기 + 30% 슬랙 + 게이트 히스테리시스 + lso T=0.33
#  1) lso mid : 전방 원거리 절반 잔존 -> 게이트 통과 -> 이동 MRC 성공 기대
#  2) lso high: 전방 실명 -> 게이트 DEFER 기대 (히스테리시스로 플래핑 차단)
#  3) nf      : 무고장 대조 -> 신형 감지기 오탐 없이 완주 기대
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in "lso mid" "lso high" "nf mid"; do
  set -- $f
  echo "=== VERIFY fault=$1 sev=$2 ==="
  timeout 900 python3 experiments/experiment_runner.py \
    --method ours --fault $1 --sev $2 --npc none --reps 1 --t 20 \
    >> experiments/results/v46c_verify.log 2>&1
  tail -1 $(ls -t experiments/results/results_*.csv | head -1)
done
echo V46C_VERIFY_DONE
