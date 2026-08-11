#!/bin/bash
# v3 배터리 주차 플로우 헬스체크: 오늘 성공 패턴 대비 이탈 감지
R=/home/taewook/autoware_carla_leaderboard/experiments/results
echo "=== 진행률 (Arm B 이후 CSV) ==="
for f in $(ls -t $(find $R -name "results_*.csv" -newermt "2026-08-10 17:50") 2>/dev/null | tac); do
  n=$(($(wc -l < $f) - 1)); echo "$(basename $f): $n rows"
done
echo "=== 판정 분포 ==="
tail -q -n +2 $(find $R -name "results_*.csv" -newermt "2026-08-10 17:50") 2>/dev/null \
  | awk -F, '{print $8}' | sort | uniq -c | sort -rn
echo "=== 주차 타깃 선택 분포 (final_state) ==="
tail -q -n +2 $(find $R -name "results_*.csv" -newermt "2026-08-10 17:50") 2>/dev/null \
  | awk -F, '{print $9}' | sort | uniq -c | sort -rn
echo "=== 최근 6케이스 freespace 오류 ==="
for d in $(ls -t $R/case_logs | head -6); do
  n=$(zcat $R/case_logs/$d/supervisor.log.gz 2>/dev/null | grep -ac "ERROR")
  echo "$d: supervisor_err=$n"
done
grep -ac "Invalid start or goal" $R/autoware.log 2>/dev/null | xargs echo "현재 케이스 invalid-goal 라인:"
echo "=== 디스크/러너 ==="
df -h / | tail -1
ps aux | grep "[e]xperiment_runner" | wc -l
