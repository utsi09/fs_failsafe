#!/bin/bash
# v4.6h: cn 블록(180케이스, v46_cn_block_KEEP.csv 보존) 이후 잔여 블록 체인.
# imn 강도 재보정(0.05->프로브 확정값) 후 imn부터 폴트별로 재개.
# 분석 시 cn CSV + 본 체인 CSV들을 병합한다.
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
for f in imn gn lso lcl cf lf nf; do
  echo "===== BLOCK $f ====="
  python3 experiments/experiment_runner.py --method both --fault $f \
    >> experiments/results/v46_rest.log 2>&1
done
bash experiments/launch_v3_armC.sh >> experiments/results/v46_armC.log 2>&1
echo V46_REST_DONE
# [v4.6i] agg 랜덤 부주의 적용으로 cn 블록 재수집 (구 agg와 통일)
python3 experiments/experiment_runner.py --method both --fault cn \
  >> experiments/results/v46_rest.log 2>&1
echo V46_CN_RERUN_DONE
