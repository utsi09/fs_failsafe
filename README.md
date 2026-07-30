# fs_failsafe — BT 기반 동적 FTTI 센서 모니터링 + 점수 기반 MRM Supervisor

Autoware + CARLA(AWCL) 환경에서 센서 고장을 실시간 감지하고,
고장 심각도에 따라 **갓길 대피(MRM_MOVE)** 또는 **순정 Autoware MRM 위임(MRM_DEFER)** 을
수행하는 failsafe 시스템과 자동 반복실험 프레임워크.

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

agent/aw_e2e.py           # 리더보드 에이전트 (고장 주입: ln/cn/imn/gn/cf/lf,
                          #   tail NPC, npc=agg, TM_SEED, dg 직행 목적지)

patches/                  # 서드파티 저장소 수정분 (git diff)
├── autoware_launch.patch        # 진단 그래프(carla), AEB 언링크,
│                                #   engage_on_driving, goal_planner 완화 등
├── carla_cpp_bridge.patch       # 브리지 노이즈 주입 파라미터
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

## 실험 매트릭스 (v2)

| 차원 | 값 |
|---|---|
| method | ours / baseline (동일 시드 짝비교) |
| fault | ln, cn, imn, gn (mid/high) + cf, lf (파괴) + nf (무고장 대조) |
| t | 20, 40 s (엔게이지 기준) |
| npc | def / agg (+꼬리 NPC 상시) |

= 셀 84개, `--reps`로 반복 지정. 권장: 파일럿 2회 → 헤드라인(agg,t20) 20회 → 커버리지 5회.
