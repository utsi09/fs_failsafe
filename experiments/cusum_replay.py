#!/usr/bin/env python3
"""CUSUM h/k 오프라인 민감도 리플레이 (Arm C 보조, 시뮬 불필요).

case_logs/*/cdr.log.gz 의 라이다 체커 시계열
  "LIDAR_front quality: q (sigma Xm, base Ym, pts A/B, cusum S)"
에서 (sigma, base) 열을 추출해 S=max(0,S+(sigma-base)-k) 를 h·k 그리드로
재계산한다. 케이스 폴더명(..._<fault>)으로 고장/무고장(nf)을 구분해
  - 고장 케이스: fault 주입 후 첫 S>h 까지의 지연(틱)
  - nf 케이스: S>h 발생 = 오경보
를 집계한다. 출력: results/cusum_replay.csv (h,k_scale,fault,케이스,지연틱,오경보)
"""
import csv
import glob
import gzip
import os
import re
import sys

ROOT = "/home/taewook/autoware_carla_leaderboard/experiments/results/case_logs"
OUT = "/home/taewook/autoware_carla_leaderboard/experiments/results/cusum_replay.csv"
PAT = re.compile(r"(LIDAR_\w+) quality: [\d.e+-]+ \(sigma ([\d.e+-]+)m, base ([\d.e+-]+)m")

H_GRID = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0]
KSCALE_GRID = [0.25, 0.5, 1.0]   # k = scale * baseline

def series(path):
    out = {}
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", errors="ignore") as f:
        for line in f:
            m = PAT.search(line)
            if m:
                out.setdefault(m.group(1), []).append(
                    (float(m.group(2)), float(m.group(3))))
    return out

def replay(sig, h, kscale):
    s, alarm_at = 0.0, None
    for i, (x, base) in enumerate(sig):
        s = max(0.0, s + (x - base) - kscale * base)
        if alarm_at is None and s > h:
            alarm_at = i
    return alarm_at

def main():
    rows = []
    dirs = sorted(glob.glob(os.path.join(ROOT, "*")))
    for d in dirs:
        logp = os.path.join(d, "cdr.log.gz")
        if not os.path.exists(logp):
            continue
        fault = os.path.basename(d).split("_")[-1]
        try:
            per_sensor = series(logp)
        except Exception:
            continue
        for sensor, sig in per_sensor.items():
            if len(sig) < 100:
                continue
            for h in H_GRID:
                for ks in KSCALE_GRID:
                    a = replay(sig, h, ks)
                    rows.append({
                        "case": os.path.basename(d), "fault": fault,
                        "sensor": sensor, "h": h, "k_scale": ks,
                        "n_ticks": len(sig),
                        "alarm_tick": a if a is not None else "",
                        "false_alarm": 1 if (fault == "nf" and a is not None) else 0,
                    })
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"{len(dirs)} case dirs -> {len(rows)} rows -> {OUT}")

if __name__ == "__main__":
    sys.exit(main())
