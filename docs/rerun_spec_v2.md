# 재실험 스펙 v2 (rerun_spec_v2)

대상: `~/fs_failsafe` v2 브랜치 (cdr_monitor + mrm_supervisor + experiment_runner).
목적: v1 캠페인(880케이스)에서 드러난 감지 사각지대·측정 오차·실험 설계 결함을
수정한 v2 스택으로 무엇을, 얼마나, 어떤 순서로 다시 돌릴지 확정한다.

---

## 1. v1 캠페인 요약과 한계

**규모**: 고장 7종(ln·cn·imn·gn·cf·lf·nf) × 심각도(mid/high, 해당 고장만) ×
NPC(def/agg) × 방법(ours/baseline) × 10반복 = **880케이스**.
트리거 시점은 셀 키 시드 랜덤(10–45 s, ours/baseline 짝 공유), TM_SEED 짝 공유.

**주요 발견 5건**:

| # | 발견 | 내용 | v2 대응 |
|---|------|------|---------|
| 1 | **ln 비단조** | 라이다 노이즈 심각도(mid 0.3 m ↔ high 0.6 m)와 결과 심각도가 단조가 아님. 주입 노이즈가 phantom 장애물 생성과 포인트 품질 저하 두 경로로 갈라지고, 품질 체커가 같은 프레임을 이중 학습·패딩 포인트를 유효 취급하는 주입/측정 아티팩트가 의심됨 | 라이다 픽스 3종 (§2.2) |
| 2 | **imn 사각지대** | IMU 백색노이즈 40/40 케이스 **전건 미감지**. EKF가 자이로 백색노이즈를 흡수해 출력 잔차(loc_jump)에도 입력-출력 잔차(loc_meas)에도 나타나지 않음 | IMU 원신호 감시 (§2.2) |
| 3 | **gn 단계적 대응 (긍정)** | GNSS 노이즈 σ1.0 m → 밴드 → 갓길 회차, σ2.5 m → 플로어 → DEFER. 심각도 비례 대응이 설계대로 동작함을 실증 | 유지 (회귀 확인만) |
| 4 | **goal_planner 교착** | 목적지 인접 구간(마지막 ~100 m)은 goal_planner가 소유 → 재라우팅 서비스 무응답, 짧은 갓길 pocket "generated 0 pull over path candidates" 정지 교착 | 목적지 40 m 이격 + pocket 필터 (§2.1) |
| 5 | **측정 지연 부풀림** | supervisor 로그상 감지(전이) **0.93 s** vs judge 관측 **2.2 s**. 상태 토픽이 래치 1회 발행 + judge 폴링 구조라 관측 지연에 인위적 오프셋이 섞임 | 즉시+주기 발행 (§2.3) |

**한계**: (a) 케이스별 cdr/supervisor 로그를 truncate — 사후 원인 분석 불가;
(b) 충돌 판정이 종료 후 결과 json에만 의존 — 케이스 중 충돌 시각 미기록;
(c) nf 오탐율이 v2 감지기(CUSUM·k-미스·IMU 원신호) 기준이 아님;
(d) 정지 케이스의 정지 **요인**(신호 대기/AEB/goal_planner) 미기록.

---

## 2. v2 변경 전체 목록

### 2.1 설계 (실험 설계·시나리오)

| 항목 | v1 | v2 | 근거 |
|------|----|----|------|
| 목적지 위치 | 교차로 인접 (goal_planner 소유 구간과 겹침) | **교차로에서 40 m 이상 이격**한 직선 구간 | 발견 #4: 목적지 부근 재라우팅 무응답/교착 제거 |
| 신선도 데드라인 τ_fresh | 850 ms 하드코딩 | **500 ms 파라미터화** (`--fresh-ms`, cdr `fresh_ms:=`) | FDTI 예산 분해 (아래 표) |
| 고장 주입 유효성 | 주입 시점 ego 상태 미확인 | **주행중 주입 재확인**: 주입 시점 속도 >1 m/s + `/fault_injection/active` 래치 수신 확인, 미충족 시 케이스 무효 처리 | 정차 중 주입된 케이스가 결과를 오염 (v1 신호 대기 중 주입 사례) |

**FDTI 예산 1.5 s 분해표** (고장 발생 → MRM 상태 전이 발행까지):

| 구간 | 예산 | 근거 |
|------|------|------|
| 센서 이상 확정 (τ_fresh / k-미스) | 0.50 s | τ_fresh 500 ms; 침묵형은 k-미스가 3×공칭주기(~0.15–0.3 s)에 조기 페널티 |
| cdr BT 평가 (10 Hz, 1 tick) | 0.10 s | 모니터 주기 |
| 점수 EMA 임계 통과 | 0.20 s | 품질 점수 평활 수렴 (CUSUM은 별도 경로로 즉시 0.2 강하) |
| /monitor/scores 전파 + supervisor tick | 0.15 s | 10 Hz supervisor 주기 + DDS 전파 |
| supervisor 밴드 유지 (hold_ticks 5) | 0.50 s | v1은 10 tick(1.0 s) — blip 억제를 CUSUM/k-미스가 흡수하므로 5 tick으로 단축 (민감도 배터리 §3-④에서 5 vs 10 검증) |
| 상태 전이 즉시 발행 | 0.05 s | §2.3 즉시 발행 — v1의 발행 지연 제거 |
| **합계** | **1.50 s** | |

### 2.2 감지 (cdr_monitor / mrm_supervisor)

| 항목 | 내용 | 겨냥하는 v1 결함 |
|------|------|-----------------|
| **CUSUM** | lidar/cam 품질 체커에 σ 초과분 순차 누적 검정 추가 (`cusum_k` 슬랙, `cusum_h` 결정 임계). 누적 초과 시 점수를 0.2로 강제 — 작지만 지속되는 열화를 EMA보다 빨리 확정 | ln 비단조·저심도 사각 |
| **k-미스** | 수신 간격 이동중앙값으로 공칭 주기 추정, **3주기 침묵**이면 FTTI 마감 전이라도 timeout_score ×0.5 조기 페널티 | cf/lf 감지 지연 (FTTI 상한까지 대기) |
| **IMU 원신호** | supervisor가 자이로 z 원신호 직접 구독 (`/sensing/imu/imu_data`). 3 s 창에서 (a) 1차 차분 표본분산이 부팅 기준선의 25배(`imu_var_ratio`) 초과 → imu_noise, (b) 이동평균이 부팅 기준선 대비 0.2 rad/s(`imu_bias_thresh`) 이상 오프셋 → imu_bias. imu_health = min(둘) → cap_loc에 min() 결합 | **imn 40/40 미감지** (잔차 기반 감시의 IMU 사각지대: 백색노이즈 흡수·바이어스 누적) + 신규 imb 대응 |
| **TTC 감쇠** | fail-conservative TTC 고정 5 s 홀드 → `ttc_hold(t) = ttc_last − elapsed` 선형 감쇠. 0 도달 시 최대 감점 1회 후 홀드 해제 | 영구/고정 홀드의 이중 처벌 (품질 점수가 이미 벌점) → DEFER 조기 붕괴 |
| **라이다 픽스 3종** | ① 중복 스탬프 가드 — 같은 header.stamp 프레임을 기준선/품질에 이중 학습하지 않음; ② finite(x,y,z) 유효 포인트만 카운트 — NaN/inf 패딩을 드롭아웃과 동일 취급; ③ 품질 추정 불가(데이터 없음/스키마 불량/유효 포인트 <100) 시 **`quality_valid=false`** 발행 (v1: 무조건 1.0 낙관 처리). 카메라 체커에도 동일 적용 | ln 비단조 (주입-측정 아티팩트), 낙관적 1.0 처리 |
| **quality_valid 소비** | supervisor는 `quality_valid=false` 센서를 UNKNOWN 취급 — 별도 카운트, health에는 보수적으로 0.5 상한 가중 | 낙관 1.0이 health를 오염 |

### 2.3 측정 (experiment_runner / judge / supervisor)

| 항목 | 내용 |
|------|------|
| **즉시 발행** | supervisor 상태 전이 시 1회 즉시 발행 + 10 Hz 주기 발행 병행. 관측 감지 지연 = 실제 전이 지연 (v1 0.93 s vs 2.2 s 오프셋 제거) |
| **로그 보관** | 케이스 종료 시 cdr/supervisor 로그를 truncate 대신 gzip 보관 (`results/case_logs/<idx>_<method>_<fault>/`, `--no-log-archive`로 끔) — 내부지연 P95 표(§4)의 원자료 |
| **velocity_factors 캡처** | STUCK/DEFER 확정 직전 `/api/planning/velocity_factors` 1회 캡처 → CSV `stall_factor` 열 — 정지 요인(신호/AEB/goal_planner) 직접 규명 |
| **충돌 직접 관측** | leaderboard 로그 실시간 스캔("collided"/차로이탈)으로 케이스 중 사건 시각 포착 + 종료 json 교차 검증, `t_hazard_s` 기록 (v1: 종료 json만) |

### 2.4 신규 고장 3종 (`EXTRA_FAULT_KINDS`, `--fault`로 명시 선택 시만 실행)

| 고장 | 주입 | 문헌 근거 |
|------|------|-----------|
| **imb** 자이로 바이어스 | `imb=<t>:<bias>` — t초부터 자이로 z에 상수 바이어스 (mid 0.3 / high 1.0 rad/s) | 자동차급 MEMS 자이로 정상 노이즈는 σ 0.04–0.09 °/s 수준(데이터시트 스펙) — 주입값(≈17–57 °/s)은 정상 대비 2–3자릿수 위의 명백한 고장이며, 감시 임계 0.2 rad/s(≈11.5 °/s)도 정상 바이어스 불안정성 대비 충분한 오탐 여유를 가짐. 백색노이즈(imn)와 달리 **누적형**이라 잔차 감시의 두 번째 사각지대를 검증 |
| **rain** 폭우 | `rain=<t>` — CARLA 날씨를 폭우 프리셋으로 전환 (물리 기반 환경 열화) | 우천 시 라이다 열화는 가우시안 거리 노이즈가 아니라 **포인트 소실형**(에코 손실·감쇠)이라는 실측 문헌 — ln(노이즈 주입)과 독립인 현실 고장 모드. 카메라도 동시 열화되는 **다중 센서 동시 열화** 시나리오 |
| **gb** GNSS 블랙아웃 | `gb=<t>[:<dur>]` — t초부터 dur초간 GNSS 발행 중단 (생략 시 영구) | 도심 협곡의 멀티패스/NLOS는 수 m–30 m 오차 및 완전 신호 두절을 유발 — gn(연속 노이즈)과 다른 **완전 상실** 모드. EKF dead-reckoning 유예시간 T_grace 도출용 (§4) |

---

## 3. 보강실험 배터리 (총 250케이스, ≈4.5분/케이스 → 약 19시간)

| # | 배터리 | 케이스 | 구성 | 목적 | 명령어 예시 |
|---|--------|--------|------|------|-------------|
| ① | **cf/lf v2 재검증** | 80 | cf·lf × ours/baseline × def/agg × 10반복 | 픽스 3종+quality_valid+k-미스 적용 후 HW 사망 고장 결론 확정 (v1 대비 감지 지연 단축 확인) | `python3 experiments/experiment_runner.py --fault cf --reps 10` / `--fault lf --reps 10` |
| ② | **MRM-off FTTI 도출** | 30 | {ln, gn, imb, cf, lf} high(해당 시) × agg × 5반복 + ln mid × 5 | 감시/MRM 개입 없이 고장→위험사건(t_hazard) 분포 관찰 → 실증 FTTI 상한 (예산 1.5 s의 타당성 근거) | `python3 experiments/experiment_runner.py --mrm-off --method ours --fault ln --sev high --npc agg --reps 5` |
| ③ | **동적 FTTI ablation** | 30 | {ln, cn, gn} mid × ours × agg × fixed/dynamic × 5반복 | 동적(TTC 조임) FTTI의 기여를 고정 데드라인 대비 분리 — 논문 핵심 주장 검증 | `... --variant fixed --fault ln --sev mid --npc agg --method ours --reps 5` vs `--variant dynamic ...` |
| ④ | **민감도** | 40 | τ_fresh {500, 850, 1000 ms} × {ln mid, cf} × ours × agg × 5반복 (30) + hold_ticks {5, 10} × ln mid × 5 (10) | FDTI 분해표 파라미터의 감지지연·오탐 트레이드오프 곡선 — 500 ms/5 tick 선택 정당화 | `... --fresh-ms 500 --fault ln --sev mid --method ours --npc agg --reps 5` |
| ⑤ | **현실 고장** | 30 | imb mid/high × 5 (10) + rain × 10 + gb × 10 (ours/baseline 혼합) | 신규 고장 3종의 감지·대응 실증 (imb: IMU 원신호 바이어스 검출, rain: 다중 센서 동시 열화, gb: T_grace) | `... --fault imb --sev high --reps 5` / `--fault rain --reps 10` / `--fault gb --reps 10` |
| ⑥ | **nf 재실험** | 20 | nf × ours/baseline × def/agg × 5반복 | v2 감지기(CUSUM·k-미스·IMU 원신호·quality_valid) 기준 오탐율 — 전략 A/B 게이트 1 (§5) | `python3 experiments/experiment_runner.py --fault nf --reps 5` |
| ⑦ | **구멍 메꾸기** | 20 | v1 결과 CSV의 인프라 실패(SETUP_FAIL/CARLA_DIED/TIMEOUT 등) 셀 재실행 | v1 매트릭스의 짝(ours/baseline) 결손 복구 — 짝 검정 표본 확보 | `python3 experiments/experiment_runner.py --retry-failed results/results_v1.csv` |

실행 순서 권장: **⑥ → ① → ⑤ → ②/③/④ → ⑦** (⑥·①이 §5 게이트 판정에 필요하므로 최우선).

---

## 4. 분석 계획

1. **짝 검정** — ours vs baseline은 TM_SEED·trigger_s를 공유하는 짝 설계.
   이항 결과(충돌 유무, 차로 점유 유무)는 McNemar 검정, 연속 결과
   (time_to_stop_s, min_gap_m, lane_block_pct, deg_dist_m)는 Wilcoxon
   부호순위 검정. 비율은 Wilson 95% CI로 보고. 인프라 실패 outcome
   (SETUP_FAIL/CARLA_DIED/TIMEOUT/STUCK_AT_JUNCTION/INTERRUPTED)은 분석 제외,
   ⑦로 짝 복구.
2. **내부지연 P95 표** — 보관된 case_logs에서 구간별 지연을 추출:
   주입(`fault ACTIVE` 로그) → 센서 토픽 이상 → cdr 점수 임계 통과 →
   supervisor 전이 로그 → 상태 토픽 관측(judge). 각 구간 P50/P95를
   FDTI 분해표(§2.1) 예산과 나란히 제시 — 예산 준수 검증이자
   v1 "0.93 vs 2.2 s" 해소의 직접 증거(전이 로그 시각 = 관측 시각).
3. **σ_crit 곡선** — 노이즈형 고장(ln/cn/imn/gn)별로 감지 성공(1/0) vs
   주입 σ를 로지스틱 회귀 → 50% 감지 임계 σ_crit과 95% CI.
   v1 대비 이동 보고: 특히 **imn은 v1 σ_crit = ∞(전건 미감지) → v2 유한값**이
   IMU 원신호 감시의 정량 기여.
4. **GNSS 블랙아웃 T_grace 곡선** — gb 지속시간 dur ∈ {5, 10, 20, 30, ∞} s
   스윕(⑤ 확장): 블랙아웃 길이 vs 위치오차(loc_meas 잔차)·위험사건율 곡선
   → EKF dead-reckoning이 버티는 허용 유예시간 T_grace 추정.
   "즉시 DEFER가 아니라 T_grace까지는 감속 지속 주행이 안전"의 근거.

---

## 5. 전략 결정: A(v1 본편 + 보강) vs B(전면 재실험)

| | 전략 A | 전략 B |
|---|--------|--------|
| 내용 | v1 880케이스를 본편으로 유지(측정 오프셋 사후 보정) + 보강 250을 부록/보완 장으로 | v2 스택으로 본편 매트릭스 전체 재실험 (~880케이스) |
| 비용 | ≈19 h (보강만) | ≈19 h + 66 h ≈ 3.5–4일 |

**게이트 검사** (⑥·① 결과로 판정, 순서대로):

1. **게이트 1 — 오탐 불변**: ⑥ nf 20케이스에서 오탐(무고장 MRM 발동) 0건이고
   v1 nf 오탐율과 방향 일치. 실패 시 → 감지기 거동 자체가 달라진 것이므로
   v1 결과와 비교 불가: **전략 B**.
2. **게이트 2 — 결론 재현**: ① cf/lf 80케이스의 ours−baseline 효과 방향이
   v1과 동일하고 효과 크기 CI가 v1 추정치와 겹침. 실패 시 → 라이다 픽스가
   결과를 뒤집은 것이므로 해당 고장군(ln/lnf/cf/lf)만 **부분 재실험**
   (부분 B, ≈250케이스 추가) 후 나머지는 A.
3. **게이트 3 — 측정 편향 상수성**: v1 로그에 supervisor 전이 timestamp가
   남아 있어 관측 지연(≈+1.3 s)을 케이스별로 사후 보정 가능함을 확인
   (전이 로그 없는 케이스 비율 <5%). 실패 시 → 지연 지표만 v2 수치로 대체
   보고하고 나머지는 A 유지.

**요약**: 세 게이트 모두 통과 → **전략 A** (본편 = v1 880 + 사후 보정,
보강 250은 v2 개선 효과 장으로). 게이트 1 실패 → **전략 B**.
게이트 2만 실패 → 부분 B. 게이트 3만 실패 → A + 지연 지표 교체.
판정은 보강 배터리 ⑥·① 종료 시점(≈캠페인 첫 8시간)에 내린다.
