#!/bin/bash
# v4.6 검증: B게이트(라이다 이동 하한) + 신규 폴트 2종(lso/lcl) + 카메라 대조
#  1) ln-mid  ours: obstacle 0.14 -> 게이트 DEFER (스턱 대신 깨끗한 정차) 기대
#  2) cf      ours: 카메라만 고장 -> obstacle 정상 -> 이동 MRC 계속 수행 기대
#  3) lso-mid ours: 전방 섹터 T=0.35 -> cdr 감지 + (팬텀 없음이라) 이동 성공 기대
#  4) lcl-mid ours: 강우 5mm/h 클러터 -> cdr 감지, 팬텀이면 게이트/폴백 동작 관찰
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in "ln mid" "cf mid" "lso mid" "lcl mid"; do
  set -- $f
  echo "=== VERIFY fault=$1 sev=$2 ==="
  timeout 900 python3 experiments/experiment_runner.py \
    --method ours --fault $1 --sev $2 --npc none --reps 1 --t 20 \
    >> experiments/results/v46_verify.log 2>&1
  tail -1 $(ls -t experiments/results/results_*.csv | head -1)
done
echo V46_VERIFY_DONE
