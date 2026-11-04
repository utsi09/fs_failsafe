# 순정 Autoware(감시·대응 없음) 비교군 실험 설계

작성 2026-11-04. 목적: 센서 고장 시 **아무도 개입하지 않는 순정 Autoware**가 무엇을 하는지 캠페인과 같은 조건으로 전수 기록하고, 제안 감독자의 **정지 유보(MRM_DEFER)** 및 베이스라인(검출 + 스톡 정지 MRM)과 짝 비교한다. 핵심 질문: 순정은 "얼어붙는가 / 고장 난 채 계속 가는가", 그때 어떤 문제가 생기는가(차로 점유, 후속차 차단, 차로 이탈, 근접, 충돌), 제안의 정지와 무엇이 다른가.

## 1. 세 비교군의 정의

| 군 | 검출기 | 대응 | 데이터 |
|---|---|---|---|
| **stock (이번 실험)** | 없음 (cdr 은 점수만 기록, 개입 없음; 진단 그래프용 더미 OK) | 없음 — Autoware 자체 진단·플래너 반응만 | 새로 585회 |
| baseline | 제안과 동일 | 검출 시 스톡 정지 MRM 호출 | 캠페인 merged_final.csv |
| ours | 제안 | 대피(MOVE) 또는 정지 유보(DEFER) | 캠페인 merged_final.csv |

stock 은 러너의 `--mrm-off` 모드다. 원고 5.2절의 "종류당 1회 노출 관찰"을 캠페인 전체 셀로 확장하는 것.

## 2. 매트릭스 (캠페인과 동일 셀, 짝 비교)

- 고장 13조건: cf, cn-mid/high, gn-mid/high, lso-mid/high, lcl-mid/high, imn-mid/high, lf, nf
- 교통 3종: def, none, agg
- 반복 15회: TM_SEED = 100 + rep (캠페인과 같은 교통 시드)
- 주입 시점: 캠페인과 같은 셀 키 추첨 `random.Random(f"{fault}/{sev}/{npc}/{rep}").randint(10, 45)` → 같은 셀의 ours/baseline/stock 이 같은 자리에서 고장을 맞는다
- 총 585회. 케이스당 평균 7분(고장 뒤 60 s 정지 또는 완주 + CARLA 재시작) → 약 68시간, 3일
- **실행 순서**: 교통 def 블록(195회, 약 23 h) → none → agg. 블록 안에서는 rep 마다 13조건을 한 번씩 돌아가며(라운드로빈) 돌려 중간에 끊겨도 조건이 고르게 쌓이게 한다
- 디스크: 케이스당 bag 100~300 MB → 585회 최대 170 GB. 여유 84 GB 라 **포인트클라우드·궤적은 제외하고 객체·상태 토픽만** 담아 케이스당 ≤ 150 MB 로 맞추고, 러너가 15 GB 미만이면 멈춘다. 필요하면 분석 끝난 블록의 bag 을 지운다.

## 3. 케이스 종료 규칙 (관찰 모드 judge)

고장 뒤 60 s 이상 정지 → HAZARD_STOPPED, 충돌/차로이탈 → HAZARD_COLLISION/LANE_DEPART, 시나리오 종료 → SCENARIO_ENDED, 고장 뒤 360 s 무사고 → HAZARD_SURVIVED. "정지"가 신호 대기인지 얼어붙음인지는 GT 로거의 신호등 상태 열로 사후에 가른다(아래).

## 4. 기록 (케이스마다)

기존: results CSV 한 줄(outcome, 정지 좌표, 차로 점유, deg_dist, min_gap, 충돌 수·종류, 관측창 지표), leaderboard/autoware/cdr 로그, diag_dump(진단 유닛별 레벨).
추가:
- **GT csv (20 Hz)**: 자차 위치·속도 + 80 m 내 모든 차량·보행자 위치/속도 + **자차 앞 신호등 상태(tl)** 열 — 정지가 적신호 대기인지 구분
- **bag**: 객체 검출/추적, EKF, NDT 신뢰도, /monitor/scores(점수만), 차속, **Autoware MRM 상태(/system/fail_safe/mrm_state)**, **운행 모드(/system/operation_mode/state)**, **경로 상태(/planning/mission_planning/route_state)**, **제어 명령(/control/command/control_cmd: 가감속·조향)**, **진단 그래프(/diagnostics_graph/status)**, clock
- **method 열 = "stock"**, CASE_TAG 접두 `stock-`

## 5. 케이스당 파생 지표 (분석기 `stock_freeze.py`)

고장 시각 t_f 이후 구간에서:

| 지표 | 정의 | 비교 대상 |
|---|---|---|
| 결말 종류 | frozen(정지 ≥ 60 s, Autoware MRM 상태 NORMAL, 신호 대기 아님) / stock_mrm(Autoware 자체 MRM 작동) / drove_on(완주 또는 360 s 생존) / collision / lane_depart | ours 의 DEFER·MOVE, baseline 의 정지 |
| t_stop | 고장 → 정지 시작 | ours 의 time_to_stop |
| 정지 위치 | 교차로 안 / 차로 중간 / 목적지 30 m 내 / 갓길, 차로 점유 길이·비율 | ours DEFER 의 정지 위치 |
| deg_dist | 고장 뒤 주행 거리 | 캠페인 deg_dist (9~58 m) |
| v_max_after | 고장 뒤 최대 속도 | 저속 캡 3.0 m/s 와 대조 |
| lane_dep | 차로 이탈 횟수 (leaderboard) | |
| min_gap, min_TTC | 고장 뒤 NPC 와 최소 거리·TTC (GT) | 관측창 obs_* |
| blocked_s | 후속차(tail NPC)가 10 m 내에서 0 속도로 갇힌 시간 | ours 의 정지 vs 대피 |
| hard_brake | 감속 < −3 m/s² 횟수 (control_cmd) | |
| aw_diag | Autoware 진단이 ERROR 로 올린 유닛·시각 (diag_dump) | 제안 검출 지연과 대조: 순정이 "무엇을 언제 눈치채는가" |
| recov | 정지 뒤 다시 출발했는가(얼었다 풀림) | |

짝 비교: 같은 (fault, sev, npc, rep) 의 stock vs ours vs baseline 을 나란히. 분포는 조건별 막대(frozen/stock_mrm/drove_on/collision), 짝 차이는 deg_dist·t_stop·점유·blocked_s 의 쌍별 차이(부호검정, 15쌍).

## 6. 산출물

- Fig: (a) stock 결말 분포 13조건 (캠페인 그림 (b)(c) 와 같은 폭·행), (b) 짝 비교: deg_dist·t_stop·차로 점유 (stock vs ours DEFER), (c) 순정 진단이 ERROR 를 올린 비율·지연 vs 제안 검출률·지연
- Table: 조건별 frozen / stock_mrm / drove_on / collision / lane_depart 수, 정지 위치 분류

## 7. 실행

```
experiments/stock_matrix.sh            # def → none → agg, rep 마다 13조건 라운드로빈, 완료 셀은 results/stock_done.txt 에 기록(재시작 시 건너뜀)
```
러너 호출: `experiment_runner.py --mrm-off --record --method ours --fault F --sev S --npc N --rep R --reps 15 --no-log-archive` (method 열은 러너가 "stock" 으로 바꿔 쓴다)

## 8. 이후

이 매트릭스가 끝나면 Eagleye 블록(GNSS 오차 → 측위) 과 노이즈 스윕을 잇는다.
