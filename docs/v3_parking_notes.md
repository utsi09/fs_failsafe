# v3 주차장 MRM — 엔지니어링·실험 노트 (2026-08-10 ~ 08-11)

논문 집필용 1차 자료. 모든 수치·사건은 러너 CSV와 supervisor/autoware 로그 실측.

## 1. 요약

갓길(노측) 대피만 있던 MRM에 **주차장 대피**를 추가하고, Town07에 4개 주차 후보를
등록해 전부 PARKED_SUCCESS를 실측으로 검증했다. 이 과정에서 후진주차를 막던
버그 사슬(에이전트 제어 변환 → supervisor 타이머 → 맵 위상 → 시뮬레이터 판정
조항)을 총 15건+ 규명·수정했다.

핵심 지표 정의(사용자 확정):
- **주차 전환 후 성공률** = PARKED_SUCCESS / (MRM_MOVE:parking 진입 케이스) — 주 지표
- 전환 전 제자리 DEFER율 = 안전 동작 통계로 별도 보고 (버그 아님)
- 인프라 실패(CARLA_DIED/SETUP_FAIL/fault 미발화)는 분모 제외 후 재수거

## 2. 주차 후보 4곳 (lanelet2 `type=parking_lot` 폴리곤)

| id | 위치 | goal 오버라이드 (map_model.py) | 비고 |
|---|---|---|---|
| 1002367 | 남쪽 헛간 마당 | (-17.0, 14.0) 북향 | 후진주차 검증 |
| 1003001 | 북쪽 14칸 라인 | (-51.75, 68.3) 북향 | 레벨 구운 차량(BMW/Jeep/Leon) 사이 빈 칸 |
| 1003002 | 북서 로터리 섬 | (-147.5, 91.9) 북향 | 연석 필터 필요 |
| 1003003 | 북서 세로 패드 | (-140.9, 108.3) 남향 | 북측 덤불 회피 |

**goal 배치 3법칙** (A* "Invalid start or goal pose" 원인 전수):
1. footprint 전체(base_link 앞 3.83 / 뒤 1.05 / 폭 ±1.1 m)가 폴리곤 안 — 3 cm 초과도 실패
2. 레벨에 구워진 정적 차량·덤불과 이격 (`world.get_environment_objects()`로 실측 가능)
3. 폴리곤은 반드시 접근 도로를 물 것 — 정차 지점이 폴리곤 밖이면 `in_lot=0` → Parking 전환 불발

## 3. 수정된 결함 연대기 (원인 → 수정)

| # | 증상 | 근본 원인 | 수정 |
|---|---|---|---|
| 1 | 후진 계획·R기어 정상인데 영구 정지 | 에이전트가 후진 가속 부호 반전 (AW 규약: 후진에도 가속 +) | aw_converter 부호 반전 제거 |
| 2 | 주차 기동 90 s에 강제 종료 | creep guard(90 s/25 m)가 60~120 s 걸리는 주차 전환을 사살 | parking 후보 제외 |
| 3 | gn 온셋 즉시 DEFER | loc EMA 과도응답을 순간 샘플링 | 이동가능 게이트 5 s 유지, 하드플로어 0.5→2 s |
| 4 | 도착 후 route 재전송 스팸 | nudge가 출발 전 발사 | moved_since_goal 게이트 |
| 5 | 4.3 m 앞에서 조기 PARKED | mission 조기 ARRIVED | 주차장은 goal 3 m 이내만 인정 |
| 6 | 교차로 통과 거부 | 도로-갓길 재배분 때 도로 2.49 m 핀치 | 폭 재배분(3.2/1.29) |
| 7 | 회랑으로 진입 불가 | 입구 좌측 경계만 중복 노드(1284≠1275) | 노드 병합 |
| 8 | goal 스냅이 고립 차선(1271) | 서향 차선 그래프 고립 | road_shoulder 리태그 + 갓길 후보 제외 |
| 9 | 로터리 섬 진입 불가 | 연석 점군이 costmap 벽 | min_lidar_height 0→0.3 |
| 10 | 도착 직후 emergency | 시나리오 전환 궤적 점프 → trajectory_validation ERROR | 데모 그래프 제외 |
| 11 | A* 스톨마다 emergency | pointcloud rate 모니터만 미완화(1 s) | 60 s 통일 |
| 12 | 장기 케이스만 실패 | leaderboard AgentBlocked 180 s / InRouteTest 30 m | 600 s / 200 m (타이머 위계: 재시도 540 < judge 560 < blocked 600 < case 900) |
| 13 | 리루트 직후 재출발 거부 | isEgoOutOfRoute 무마진 폴리곤 검사 × gn 노이즈 래치 | 1.5 m 마진 (C++ 리빌드) + 20 s 웨지 재전송 |
| 14 | 특정 지점 fault 시 전 후보 unreachable | map_model center 방향 오추정 → 싱크 lanelet 9개 | 싱크 자동 감지·center 뒤집기·재빌드 |
| 15 | 교차로 초록불 프리징 | 카메라 TL 인식 미스 | fault-조건부 V2X (카메라 fault 케이스는 카메라 인식 유지 — 실험 변인 보존) |

## 4. 고장 종류별 MRM 등급 (Arm B 실측, gn/ln 완료분)

| 고장 | supervisor 판단 | 실측 |
|---|---|---|
| gn-mid | 주차장 대피 시도 | 전환 후 성공률 ~100% (외부 판정 컷 제거 후) |
| gn-high | loc 붕괴 → 이동 금지, 제자리 DEFER | 15/15 설계대로 |
| ln-mid/high | 주차 시도하나 freespace가 라이다 기반 goal 승인 거부 → 정차 | "라이다 없인 freespace 불가" — 유효 결과로 채택 |
| cn | (TL 카메라 유지 조건) 진행 예정 | costmap 무관 → gn-mid급 예상 |
| baseline 전군 | 즉시 stock MRM (차로 내 정지, 점유 ~54%) | 대조군 일관 |

노측 대피 용어: "갓길"이 아닌 **노측 정차(road-edge pull-over)** + 차로 점유율 지표
(주차 4~15% vs 노측 ~30% vs 차로 내 ~54%).

## 5. 실험 설계 (v3 배터리)

- **Arm B** 주차 arm(`--no-shoulder`, "갓길 없는 구간"): 5고장 × npc{none,def,agg} × 5reps × ours/baseline = 270
- **Arm A** 본 매트릭스: 11고장셀 × npc3 × 15reps × 페어 = 990
- **Arm C** 민감도(OAT): min_loc_to_move / lower_threshold / shoulder_mrc_penalty 스윕 (+CUSUM h·k는 오프라인 리플레이 24,012행)
- 확장: imb(계단 바이어스)·imd(램프 드리프트 — 감지지연 9.1 s 실측, CUSUM 지연∝1/기울기 검증용)
- MRC 질 서열을 비용에 인코딩: `shoulder_mrc_penalty=175` (≈저하주행 25 s 상당, gap +80~270 실측 보정)
- 게이트 운용: gn-mid 15케이스에서 주 지표 ≥90% 확인 후 무중단 진행

## 6. 재현 도구

- `experiments/full_stop.sh` — 유일하게 허용되는 중단 방법 (부분 정지 사고 3종 예방)
- `experiments/v3_check.sh` — 진행/판정/타깃 분포 헬스체크
- `experiments/cusum_replay.py` — cdr 로그에서 CUSUM h·k 그리드 오프라인 재계산
- `patches/` — autoware_launch / autoware_universe(behavior 마진) / leaderboard(판정 조항) diff
- RViz HUD: `/mrm_supervisor/hud` (상태·health·고장 센서), ADAPI mrm_state 매핑

## 7. 데모 후 원복 목록 (논문 방법론 명기 대상)

- costmap `minimum_lidar_height_thres` 0.3 (연석 등판 가정)
- 진단 그래프에서 localization·trajectory_validation 링크 제외 (supervisor가 저하 구간 MRM 소유)
- `CDR_DEMO_NO_ERROR=1`, AgentBlocked 600 s, InRouteTest 200 m
- TL fault-조건부 V2X
