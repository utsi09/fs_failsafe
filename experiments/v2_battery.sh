#!/bin/bash
# v2 보강 배터리 — rerun_spec_v2.md 순서(⑥→①→⑤→②→③→④→⑦), 총 ~250케이스
# 각 단계는 독립 CSV(results_<시각>.csv)로 기록됨. 실패해도 다음 단계 진행(;)
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
R="python3 experiments/experiment_runner.py"

echo "=== ⑥ nf 오탐 게이트 (20) $(date +%T) ==="
$R --fault nf --reps 5

echo "=== ① cf/lf v2 재검증 (80) $(date +%T) ==="
$R --fault cf --reps 10
$R --fault lf --reps 10

echo "=== ⑤ 현실 고장 3종 (40) $(date +%T) ==="
$R --fault imb --npc agg --reps 5
$R --fault rain --npc agg --reps 5
$R --fault gb --npc agg --reps 5

echo "=== ② MRM-off FTTI 도출 (25) $(date +%T) ==="
$R --mrm-off --method ours --npc agg --reps 5 --fault ln --sev high
$R --mrm-off --method ours --npc agg --reps 5 --fault gn --sev high
$R --mrm-off --method ours --npc agg --reps 5 --fault cf
$R --mrm-off --method ours --npc agg --reps 5 --fault lf
$R --mrm-off --method ours --npc agg --reps 5 --fault imb --sev high

echo "=== ③ 동적 FTTI ablation (30) $(date +%T) ==="
for f in ln cn gn; do
  $R --variant fixed   --fault $f --sev mid --method ours --npc agg --reps 5
  $R --variant dynamic --fault $f --sev mid --method ours --npc agg --reps 5
done

echo "=== ④ τ_fresh 민감도 (20) $(date +%T) ==="
for ms in 500 1000; do
  $R --fresh-ms $ms --fault ln --sev mid --method ours --npc agg --reps 5
  $R --fresh-ms $ms --fault cf --method ours --npc agg --reps 5
done

echo "=== ⑦ v1 구멍 메꾸기 (~35) $(date +%T) ==="
$R --retry-failed experiments/results/results_0731_1701.csv
$R --retry-failed experiments/results/results_0801_1354.csv

echo "=== 배터리 전체 완료 $(date +%T) ==="
