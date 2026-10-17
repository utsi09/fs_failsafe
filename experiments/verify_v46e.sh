#!/bin/bash
# v4.6e: V2X TL 픽스(osm 직접 파싱, 17개 규제요소) 검증 — 풀루트 완주 2케이스
#  1) nf      : 교차로 통과 완주 기대 (V2X 9416 포함 여부는 [TL SOURCE] 로그로 확인)
#  2) lso mid : 미감지 경미 오염 -> 계속 주행 -> 완주 기대
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in "nf mid" "lso mid"; do
  set -- $f
  echo "=== VERIFY fault=$1 sev=$2 ==="
  timeout 900 python3 experiments/experiment_runner.py \
    --method ours --fault $1 --sev $2 --npc none --reps 1 --t 20 \
    >> experiments/results/v46e_verify.log 2>&1
  tail -1 $(ls -t experiments/results/results_*.csv | head -1)
done
echo V46E_VERIFY_DONE
