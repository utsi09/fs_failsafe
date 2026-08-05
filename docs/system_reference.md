# 시스템 구현 사실 문서 (System Ground-Truth Reference)

> 목적: 논문·발표에서 말하는 모든 것의 "실제 구현 여부"를 코드 기준으로
> 기록한다. 과장 없이 — 구현됨 / Autoware 위임 / 비활성화됨 / 제안만 됨을
> 명시한다. (2026-08-05, v2.1 기준)

## 0. 한눈 요약: 우리가 만든 것 vs 빌려 쓴 것

| 구분 | 항목 |
|---|---|
| **우리 구현** | 센서 점수화 전 채널, 섹터/능력 합성, health 밴드 상태기계, 대피 후보 선택(비용함수·자체 다익스트라·신호등 카운트·관측성 게이트), 위치 3채널(잔차/바이어스/침묵), 진단 발행, 고장 주입 13종, 실험 러너·판정기, MRM 후보 지도 주석 |
| **Autoware 위임** | 실제 경로 계획(mission planner), 갓길 정차 기동 실행(goal_planner), 차로 내 정차(mrm_handler), 신호등 실시간 인식, EKF 위치추정, AEB/장애물 정지, 차량 제어 전부 |
| **비활성화(사유 있음)** | IMU 진동 분산 채널, 카메라 CUSUM, cdr gps_check(토픽 미설정) |
| **제안만(미구현)** | FRTI 차감형 동적 마감, 속도 구간별 원신호 기준선, 장면 조건부 기준선, 개루프 자극 시험 |

---

## 1. cdr_monitor (C++/BehaviorTree, 10Hz)

### 1.1 신선도(타임아웃) 채널 — 구현됨
- `timeout_score = max(0, 1 − age/τ_detect)` 선형 감쇠.
- **τ_detect 구현 = min(τ_fresh, TTC)** (calculate_ftti.cpp).
  τ_fresh 기본 850ms(파라미터 `fresh_ms`), 동적 조임 on/off `ttc_dynamic`.
  ⚠️ 원고 초기판의 "TTC − FRTI − margin" 차감형은 **미구현 제안**
  (원고 v5에서 정정 완료).
- k-미스: 수신 간격 이동중앙값의 3배 침묵 시 timeout_score ×0.5 조기 감점
  — 구현됨(lidar_check/cam_check, v2).
- TTC 소스: awcl_adapter가 발행하는 `/monitor/nearest_obstacle_m` +
  계획경로×예측경로 교차 시간(spatio-temporal, adapter 계산). TTC 소스
  센서 불량 시 fail-conservative 홀드 — v2에서 **선형 감쇠형**(ttc_hold =
  ttc_last − elapsed)으로 구현됨.

### 1.2 라이다 품질 — 구현됨 (명칭: 상대 거리불연속 점수)
- σ_est = mean(|Δr|<1m) × √π/2, 부팅 30프레임 기준선(이후 1%/frame 적응,
  1.5× 급등 불추종), 점수 = exp(−max(0,σ−base)/0.1) × 유효포인트비.
- v2 픽스: 동일 스탬프 중복 학습 방지, finite 포인트만 카운트,
  `quality_valid` 플래그.
- **CUSUM: 라이다만 활성** (k 기본 0.5×base, h 0.5m). 실측 3×3 격자 전
  영역 감지 유지.
- ⚠️ 이웃 = 메모리상 인접 포인트(ring/azimuth 공간 이웃 아님) — 기준선
  상대 방식이라 실험상 유효했으나 한계로 문서화. ring 기반은 미구현.

### 1.3 카메라 품질 — 구현됨, CUSUM은 비활성
- 프레임 화소 차분 기반 동일 구조.
- **CUSUM 비활성 사유**: 주행 중 optical flow로 차분이 자연 상승(정지 3.8
  → 주행 6.4 실측) → 누적 오적산으로 무고장 DEFER 유발(S=1257 실측).
  코드에 주석과 함께 클램프만 제거(값은 기록용 계산 유지).

### 1.4 IMU/GPS 물리 검사 — 부분 구현
- imu_check/gps_check: 물리 범위 검사 노드 존재.
- ⚠️ **gps_check는 AWCL 구성에서 사실상 비활성**: sensors_awcl.yaml의
  gps.topic="" (구독 대상 없음). GNSS 감시는 supervisor의 위치 3채널이
  담당(아래).
- ⚠️ IMU **진동 분산 채널(supervisor, v2)은 비활성**: 기준선이 속도
  종속(정지 학습 ≪ 주행 진동)이라 가속 구간 오탐 2회 실측 → 코드 주석과
  함께 스코어 무효화. 속도 구간별 기준선은 향후 과제.

### 1.5 섹터/능력 합성 — 구현됨
- sector = 1 − Π(1−s_i); perception = Σw·sector (F .30/FL FR .15/L R B .10/
  RL RR .05), 장애물:신호등 = 0.6:0.4; health = min(perception, cap_loc).
- 진단 발행: /monitor/scores + /diagnostics (perception_score 유닛,
  워밍업 게이트·이력 래치 포함). Autoware 진단 그래프(autoware-carla.yaml,
  우리가 유닛 추가)를 통해 mrm_handler 가용성에 연결.

## 2. mrm_supervisor (Python, 10Hz)

### 2.1 상태기계 — 구현됨 (실제 상수)
- NORMAL→MOVE: 0.3<h≤0.7 (hold_ticks=10 = 1s 유지) + armed(건강 3초) 이후.
- →DEFER: h≤0.3 5틱(0.5s) 지속, 또는 loc<min_loc_to_move(0.5).
- MOVE→PARKED: RouteState.ARRIVED 또는 종/횡 분해(경로방향 4m·횡 1.5m).
- 가드: 경로변경 무응답 5s×3→DEFER, 후보 정체 45s 블랙리스트, 크립
  90s/25m, 상태 즉시 발행+10Hz 주기 발행(v2).

### 2.2 위치 3채널 — 구현됨
- 잔차: r=|GNSS pose − EKF pose|, exp(−max(0,r−base−0.2)/0.55), 부팅
  오프셋 캘리브레이션(~1.4m, 60표본).
- 바이어스(v2): 3초 창 IMU wz 평균 vs 같은 창 EKF wz 평균 차 >0.2rad/s가
  3초 지속 시 감점. imb 실측: 주행 중 주입 14/14 감지, 1.8s.
- 침묵(v2.1): **시뮬 스탬프 기준**(EKF 스탬프 − GNSS 스탬프) 1초 초과부터
  exp(−(stale−1)/0.8) 감쇠. ⚠️ 초기 벽시계 구현은 시뮬 감속 스터터에서
  오탐 4건 → 수정 후 0/50 재검증. gb 감지는 이 채널이 담당(1.9~2.0s).

### 2.3 대피 후보 선택 — 구현됨, 전부 자체 계산
- 후보: 수동 주석 lanelet (갓길 16, 주차 7). 갓길 최소 7m(실측 경계
  6.3/7.8m 도출), 접근거리 25m 미만 탈락.
- **η(노출시간): 자체 다익스트라** — map_model이 lanelet 그래프(후행+차선
  변경 간선)에서 length/speed로 경로시간 산출. **Autoware 경로/ETA 미사용**
  (후보 다수를 매 틱 비교해야 하므로 서비스 호출 불가).
- **신호등 개수 n_tl: 자체 카운트** — lanelet 속성 `has_tl`을 자체 경로
  위에서 합산. **Autoware 미사용**(Autoware는 주행 중 신호 "인식"만 담당).
- 관측성 게이트: 경로상 신호등 방위 vs 건강 카메라(장착 yaw·FOV 파라미터)
  기하 판정 — 불가 시 J=∞. 구현됨.
- 비용: J = 7·η + 200·n_tl·g + 500·Σ(1−sector_req) + 14·η·(1−loc).
  가중치 ±50% 800회 섭동에 최상위 후보 100% 불변(변별은 게이트·거리 지배).
- 실행: 선택 후 AD API SetRoutePoints로 목적지 전달 → 이후 경로·기동은
  Autoware(mission planner→goal_planner)가 수행. 추정-실행 불일치는 경로
  워치독이 흡수.

## 3. 고장 주입 (aw_e2e.py + cpp bridge)

| 코드 | 구현 방식 | 상태 |
|---|---|---|
| ln | 브리지 noise_sigma 파라미터(포인트 거리 가우시안) | ✓ |
| cn | 브리지 카메라 화소 노이즈 | ✓ |
| imn/imb | 에이전트가 IMU 메시지에 백색/바이어스 가산 | ✓ |
| gn | 에이전트 GNSS 발행에 노이즈 | ✓ |
| cf/lf | CARLA 센서 액터 파괴 | ✓ |
| gb | 에이전트 발행 게이트 + **브리지 gnss_mute 원격 설정**(동일 토픽 이중 발행자 문제로 v2.1에서 브리지 측 추가) | ✓ |
| rain | world.set_weather 폭우 프리셋 — ⚠️ CARLA 0.9.16 센서 데이터 열화는 미재현(시각만). 오경보 강건성 실험으로 전용 | ✓(한계 명시) |
| mix | ln+cn 동시 | ✓ |
| cfl/cfl2/lfl2 | 선행차 급제동(+TTC 트리거 센서 파괴). ⚠️ 폐루프에서 TTC<1.8s 미도달(스택 선제 감속) — "위험 창 자기 소멸" 관찰로 종결 | ✓(관찰) |

## 4. 실험 인프라 사실
- 러너: 케이스별 CARLA 재기동, 컨테이너/호스트 프로세스 청소, 로그 gzip
  전수 보관(v2), 짝 설계(TM 시드·고장시점 공유), 판정기는 토픽 구독 기반.
- 판정 규칙: SHOULDER_SUCCESS(PARKED)/AW_*_DONE/DEFER_STOPPED/STUCK(정지
  100s·MOVE 150s)/TIMEOUT(6분)/CARLA_DIED/SETUP_FAIL. 관측 감지지연 해상도
  ±1s — 내부 지연은 케이스 로그 스탬프로 별도 산출.
- 진단 그래프 결합: cdr perception_score + supervisor defer_request 유닛이
  STALE이면 autonomous 잠금(실측 사고 2회 — dummy_defer로 MRM-off 해결).

## 5. 제안만 되고 미구현인 것 (정직 목록)
1. FRTI·여유 선차감형 동적 마감 (min(τ,TTC−FRTI−m))
2. 속도 구간별 IMU 진동 기준선 / 카메라 CUSUM 재활성화
3. ring/azimuth 공간 이웃 기반 라이다 지표
4. 장면 조건부 기준선(도심/수풀)
5. 공장 기준값 교차 검증(부팅-고장 감지)
6. 신호등 없는 우회 경로 탐색(관측성 게이트 완화)
7. 개루프 자극 시험(동적 예산 이득 정량화)
8. HD맵 후보 자동 추출(수동 주석 대체)
9. Autoware 자체 pull_over MRM과의 직접 비교(활성화 가능성 미조사)

## 6. 자주 헷갈리는 Q 3개
- **"경로시간·신호등 개수 Autoware 꺼?"** → 아니오, 둘 다 자체 map_model
  (다익스트라 + has_tl 카운트). Autoware는 선택 이후의 실행만.
- **"850ms 안에 다 감지?"** → 아니오, 850ms는 신선도 감쇠 상수(t0→t1
  구간). 종단 감지는 t0→t3 = 0.75~2.8s이며 고장별 위험 예산과 비교.
- **"동적 조임이 감지를 빨리 했나?"** → 발동은 실증(0 vs 15~3,373틱),
  이득은 폐루프 측정 불가(위험 창 자기 소멸) — 심층 방어로 규정.
