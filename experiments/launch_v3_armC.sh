#!/bin/bash
# v3 Arm C: 휴리스틱 민감도 스윕 (OAT) — 체인(Arm B->A) 종료 후 실행.
# 기본값(min_loc 0.5 / lower 0.3) 데이터는 Arm B가 커버하므로 스윕 값만 돈다.
# ~30케이스 x ~6분 = 약 3시간. ours만 (우리 방법의 파라미터 민감도이므로).
cd /home/taewook/autoware_carla_leaderboard
export PYTHONUNBUFFERED=1
unset MRM_FORCE_TARGET
LOG=experiments/results/v3_armC_sensitivity.log

# 1) min_loc_to_move 스윕: 주차 시도 vs DEFER 게이트
for v in 0.3 0.4 0.6 0.7; do
  MRM_MIN_LOC=$v python3 experiments/experiment_runner.py --method ours \
    --fault gn --sev mid --npc none --reps 3 --no-shoulder >> $LOG 2>&1
done
for v in 0.3 0.7; do
  MRM_MIN_LOC=$v python3 experiments/experiment_runner.py --method ours \
    --fault imd --sev mid --npc none --reps 3 --no-shoulder >> $LOG 2>&1
done

# 2) lower_threshold(하드 플로어) 스윕: 즉시 DEFER 발동선
for v in 0.2 0.4; do
  MRM_LOWER=$v python3 experiments/experiment_runner.py --method ours \
    --fault gn --sev high --npc none --reps 3 --no-shoulder >> $LOG 2>&1
  MRM_LOWER=$v python3 experiments/experiment_runner.py --method ours \
    --fault imd --sev mid --npc none --reps 3 --no-shoulder >> $LOG 2>&1
done
# 3) shoulder_mrc_penalty 스윕: 노측/주차 선택 혼합비의 민감도 ("왜 175인가" 방어)
#    갓길 포함 모드(--no-shoulder 없음)에서만 의미 있음
for v in 0 100 250; do
  MRM_SHOULDER_PEN=$v python3 experiments/experiment_runner.py --method ours \
    --fault gn --sev mid --npc none --reps 3 >> $LOG 2>&1
done
echo "armC done" >> $LOG
