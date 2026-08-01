#!/usr/bin/env python3
"""지연 아카이브 파서 — 내부(진짜) 감지시간 산출.

입력 (수집기가 캠페인 동안 계속 추가):
  latency_archive_sup.log : supervisor 상태전환 줄 (ROS 스탬프 포함)
      [supervisor-1] [WARN] [1785541450.382...] [...]: state NORMAL -> MRM_MOVE: ...
      [supervisor-1] [INFO] [...] [...]: armed: perception 1.00 healthy for 3 s
  latency_archive_cdr.log : 수집기가 벽시계 프리픽스를 붙인 cdr 줄
      1754026123.456 [cdr...] CAMERA_front timeout: 250ms / ftti: 850ms
      1754026123.456 [cdr...] [Scores] perception 0.575 | ...

케이스 구분: supervisor의 armed 줄(케이스당 1회)로 세그먼트 분할.
고장 시점(t0) 추정:
  침묵형: 첫 timeout>=150ms 줄에서 t0 = ts - timeout  (age가 마지막 프레임 시각을 알려줌)
  노이즈형: timeout 줄이 없으면 perception이 처음 0.9 아래로 내려간 시각(해상도 ~1s)
내부 감지시간 = 첫 MRM_* 전환 스탬프 - t0
"""
import re, sys, statistics as st

SUP = sys.argv[1] if len(sys.argv) > 1 else "latency_archive_sup.log"
CDR = sys.argv[2] if len(sys.argv) > 2 else "latency_archive_cdr.log"

# --- supervisor: (ts, kind, detail) 이벤트 목록
sup_events = []
for l in open(SUP, errors="ignore"):
    m = re.search(r"\[(\d{10}\.\d+)\]", l)
    if not m:
        continue
    ts = float(m.group(1))
    if "armed:" in l:
        sup_events.append((ts, "armed", ""))
    else:
        m2 = re.search(r"state (\S+) -> (\S+)", l)
        if m2:
            sup_events.append((ts, "trans", f"{m2.group(1)}->{m2.group(2)}"))

# --- cdr: (ts, kind, value) — timeout 상승 / perception 하락
cdr_events = []
for l in open(CDR, errors="ignore"):
    m = re.match(r"(\d{10}\.\d+) ", l)
    if not m:
        continue
    ts = float(m.group(1))
    m2 = re.search(r"(\w+) timeout: (\d+)ms", l)
    if m2 and int(m2.group(2)) >= 150:
        cdr_events.append((ts, "timeout", (m2.group(1), int(m2.group(2)))))
        continue
    m2 = re.search(r"\[Scores\] perception ([\d.]+)", l)
    if m2:
        cdr_events.append((ts, "perc", float(m2.group(1))))

# --- armed 시각으로 케이스 세그먼트 분할
armed_ts = [ts for ts, k, _ in sup_events if k == "armed"]
rows = []
for i, a in enumerate(armed_ts):
    end = armed_ts[i + 1] if i + 1 < len(armed_ts) else float("inf")
    # 이 케이스의 첫 MRM 전환
    trans = [(ts, d) for ts, k, d in sup_events
             if k == "trans" and a < ts < end and d.startswith("NORMAL->MRM")]
    if not trans:
        continue
    t_trans, tdetail = trans[0]
    # t0 추정: 침묵형 우선
    t0, channel = None, ""
    touts = [(ts, v) for ts, k, v in cdr_events if k == "timeout" and a < ts <= t_trans + 1.0]
    if touts:
        ts0, (sensor, age) = touts[0]
        t0, channel = ts0 - age / 1000.0, f"silence:{sensor}"
    else:
        percs = [(ts, v) for ts, k, v in cdr_events if k == "perc" and a < ts <= t_trans]
        drop = [(ts, v) for ts, v in percs if v < 0.9]
        if drop:
            t0, channel = drop[0][0], "quality(perc<0.9, ±1s)"
    if t0 is None:
        continue
    rows.append((a, channel, tdetail, t_trans - t0))

print(f"{'armed(대략 케이스 시작)':>22} | {'채널':<24} | {'전환':<22} | 내부지연 s")
print("-" * 84)
for a, ch, td, lat in rows:
    print(f"{a:>22.1f} | {ch:<24} | {td:<22} | {lat:6.2f}")
if rows:
    lats = [r[3] for r in rows]
    sil = [r[3] for r in rows if r[1].startswith("silence")]
    qual = [r[3] for r in rows if not r[1].startswith("silence")]
    print("-" * 84)
    print(f"전체 n={len(lats)} 평균 {st.mean(lats):.2f}s  최대 {max(lats):.2f}s")
    if sil:
        print(f"침묵형 n={len(sil)} 평균 {st.mean(sil):.2f}s  P95 {sorted(sil)[int(len(sil)*0.95)-1 if len(sil)>1 else 0]:.2f}s")
    if qual:
        print(f"품질형 n={len(qual)} 평균 {st.mean(qual):.2f}s (해상도 ±1s)")
