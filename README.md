# fs_failsafe — BT 기반 동적 FTTI 센서 모니터링 + 점수 기반 MRM Supervisor

Autoware + CARLA(AWCL) 환경에서 센서 고장을 실시간 감지하고,
고장 심각도에 따라 **주차장 대피 → 노측(road-edge) 대피 → 제자리 정지(DEFER)** 의
MRC 질 서열로 최소위험기동을 수행하는 failsafe 시스템과 자동 반복실험 프레임워크.

## v4.6 (2026-08-12): 현실형 라이다 고장 lso / lcl

가우시안 반경 노이즈(ln)는 현실 대표성이 낮아 본 매트릭스에서 빼고, 문헌상 빈도 1·2위인
라이다 열화 두 가지를 물리 모델로 넣었다. 둘 다 CARLA 브리지(`patches/carla_cpp_bridge.patch`)
파라미터로 구현되고, 에이전트(`agent/aw_e2e.py`)가 고장 시각에 파라미터를 켠다.

| 코드 | 뜻 | 모델 | 스펙 (`<t>:<a>:<b>`) | mid / high |
|---|---|---|---|---|
| `lso` | 전방 라이다 **오염 섹터** (커버에 흙·벌레 등이 붙어 일부 시야가 흐려짐) | 섹터 안 강도 × T² (Trierweiler 2019 투과율 실측 + Goodin 2019 검출 문턱) → 멀고 어두운 표적부터 사라짐 | `t:투과율T:섹터폭deg` | `0.33:90` / `0.07:150` |
| `lcl` | 양쪽 라이다 **강수 클러터** (비·물보라) | Beer-Lambert 감쇠 α=0.01·Rr^0.6 + 거리 지터 + 근거리 부유점 (LISA/Hahner) | `t:강우량mm/h:클러터점수` | `5:1500` / `25:4000` |

모니터(`ros/cdr_monitor`)도 같이 바뀌었다: 근거리 클러터·원거리 결손·8-섹터 히스토그램 통계와
카운트 샤프 커브(lso 소실을 포인트 수 채널로 감지). 감독자(`ros/mrm_supervisor`)에는 라이다
이동 하한(장애물 인지 능력이 바닥이면 이동 MRC 대신 제자리 DEFER), η_max 노출 시간 상한,
MRM 이동 중 속도 캡(`MRM_MOVE_SPEED`), 후방 점유 폴백이 들어갔다.

## v3 (2026-08-11): 주차장 MRM

- Town07 주차 후보 4곳(남쪽 헛간·북쪽 14칸 라인·로터리 섬·북서 패드) 등록,
  **후진주차 포함 전 지점 PARKED_SUCCESS 실측** — 성공 영상: [`docs/videos/`](docs/videos/)
- 후진주차를 막던 결함 사슬 15건 규명·수정 (에이전트 후진 가속 부호,
  supervisor 타이머 위계, lanelet 그래프 위상, 시뮬레이터 판정 조항 등)
  — 전체 목록·실측 근거: [`docs/v3_parking_notes.md`](docs/v3_parking_notes.md)
- 주 지표: **주차 전환 후 성공률** (전환 전 제자리 DEFER는 안전 동작으로 별도 집계)
- 트래픽 밀도 차원(none/def/agg + low/high), 자이로 램프 드리프트(imd) 고장,
  RViz 상태·건강 HUD, 민감도 스윕(Arm C) + CUSUM 오프라인 리플레이 추가

## 구조

```
ros/                      # colcon 패키지 (ROS 2 Humble)
├── cdr_monitor/          # C++ BehaviorTree 센서 모니터
│   │                     #   센서별 신선도(FTTI 타임아웃) x 품질(노이즈) 점수
│   │                     #   → 8방향 시야 → 능력(perception/localization) 3층 집계
│   │                     #   → /monitor/scores + /diagnostics(래치/워밍업/이진화)
│   ├── behavior_tree/monitor.xml
│   ├── config/sensors_awcl.yaml
│   └── scripts/awcl_adapter.py   # AWCL 토픽 → 모니터 입력 변환 + 시공간 TTC
├── mrm_supervisor/       # Python 상태기계
│   │                     #   health = min(perception, localization)
│   │                     #   band(0.3~0.7) → 비용 최소 갓길/주차장 재라우팅
│   │                     #   floor(≤0.3)  → DEFER (순정 comfortable/emergency stop)
│   │                     #   + 주차불가 후보 blacklist 폴백, GNSS 입출력 잔차 감시
│   └── mrm_supervisor/map_model.py  # lanelet2 파싱, 후보 추출, 경로 비용
└── hesai_zed_fusion/     # ObstacleArray 메시지 의존성

experiments/
├── experiment_runner.py  # 자동 반복실험: 스택 재기동, 고장 매트릭스,
│                         #   심판(판정), CSV 기록, CARLA 자동복구
└── carla_unstick.py      # sync 모드 잔류 해제

agent/aw_e2e.py           # 리더보드 에이전트 (고장 주입: lso/lcl/cn/imn/gn/cf/lf (+ln 등 EXTRA),
                          #   tail NPC, npc=agg, TM_SEED, dg 직행 목적지)

patches/                  # 서드파티 저장소 수정분 (git diff)
├── autoware_launch.patch        # 진단 그래프(carla), AEB 언링크,
│                                #   engage_on_driving, goal_planner 완화 등
├── carla_cpp_bridge.patch       # 브리지 고장 주입 파라미터 (lso 오염 섹터, lcl 강수, 노이즈)
└── leaderboard_and_agent.patch  # client timeout, TM_SEED, aw_e2e 전체

scripts/
├── run_leaderboard.sh    # KEY=VALUE 환경변수로 고장 지정 실행
├── cyclone_localhost.xml # CycloneDDS 설정 (호스트/컨테이너 공통 필수!)
└── setup.sh              # 배포 절차 안내
```

## 핵심 주의사항

- **DDS 통일**: 호스트 노드도 반드시 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` +
  `CYCLONEDDS_URI=cyclone_localhost.xml`. 벤더가 다르면 토픽은 통해도
  **서비스(경로 변경)가 무응답**으로 증발함.
- Autoware 실행 시 `diagnostic_graph_type:=carla` 필수 (awsim 그래프는 타임아웃 1초라
  CARLA에서 MRM 플래핑 발생).
- 실험은 `python3 experiments/experiment_runner.py --smoke`로 검증 후 풀 매트릭스.

## 실험 매트릭스 (v4.6)

| 차원 | 값 |
|---|---|
| method | ours / baseline (동일 시드 짝비교) |
| fault | lso, lcl, cn, imn, gn (mid/high) + cf, lf (파괴) + nf (무고장 대조). EXTRA(`--fault` 명시 시): ln, gm, imb, imd, rain, gb … |
| t | 20, 40 s (엔게이지 기준) |
| npc | def / agg (+꼬리 NPC 상시) |

= 셀 84개, `--reps`로 반복 지정. 권장: 파일럿 2회 → 헤드라인(agg,t20) 20회 → 커버리지 5회.
