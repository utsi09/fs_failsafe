# 추가 실험 설계: 노이즈 강도 ↔ Autoware 인지·측위 저하 ↔ 건강 점수 상관관계

작성 2026-11-04. 대상: 석사논문(Continuous Sensor Health Monitoring and Risk/Capability-aware Graded MRM Supervision) 보강 실험.
장비: 연구실 RTX 5080 PC 한 대. 기간: 3~4일. 실행 예산은 **하루치(무인 약 16시간, 146회 + 재실행 여유)**. 대규모 캠페인 아님.

## 0. 왜 하는가 (논문의 어느 구멍을 메우나)

논문 5.1절에 이렇게 써 있다: *"'설계상 허용'과 '품질 채널의 둔감'을 아직 분리할 수 없으므로 Table 5.1의 검출률은 개입 적격성으로 읽어야 한다 (분리는 향후 과제)."* 그리고 Table 5.2의 lso-mid "deliberate continued driving, deg_dist 255 m"는 **"lso-mid에서는 Autoware가 아직 볼 수 있다"**는 전제를 깔고 있는데, 그 전제를 측정한 적이 없다.

이번 실험은 그 전제를 직접 잰다. 질문은 두 개.

| | 질문 | 논문에서 걸린 결정 |
|---|---|---|
| A. 인지 | 노이즈를 올리면 Autoware(centerpoint) 검출률이 어떻게 떨어지고, 내 LiDAR 점수·perception 점수는 그걸 얼마나 따라가나? 0.7 문턱이 "실제로 못 보기 시작하는 지점"과 맞나? | lso-mid 허용(비개입), lso-high·lcl 개입 |
| B. 측위 | 노이즈를 올리면 Autoware 최종 EKF(/localization/kinematic_state) 오차가 얼마나 커지고, 내 cap_loc 점수·0.5 이동 최저선이 그 오차와 맞나? | gn-mid 대피 / gn-high 정지 분기, imn 정지 |

결과물은 논문 5장에 들어갈 새 절 하나(가칭 **5.8 Score-versus-capability correlation**)와 Table 2개, Figure 2개다.

## 1. 사전 조사에서 나온 사실 (설계를 바꾸는 것들)

이 PC의 코드와 로그를 뒤져서 확인한 내용. 실험 전에 ①만 라이브로 재확인하면 된다.

1. **gn(GNSS 위치 노이즈)은 Autoware EKF에 들어가지 않는다.**
   - 측위는 스톡 Autoware 설정 그대로: `pose_source`=ndt, twist=gyro_odom (`autoware_launch/.../tier4_localization_component.launch.xml:4`). EKF 입력은 NDT가 내는 `/localization/pose_estimator/pose_with_covariance`.
   - NDT의 GNSS 정규화는 꺼져 있음 (`ndt_scan_matcher.param.yaml:40-41`, `enable: false`).
   - gn은 `/sensing/gnss/pose_with_covariance`만 더럽힌다 (`aw_e2e.py:977-991`). 에이전트의 `_ndt_pose_pub`(l.106)은 선언만 있고 발행 안 함.
   - 기존 데이터도 같은 얘기: `results/ego_path_gnmid.csv`에서 GNSS-EKF 잔차는 0.7~2.8 m로 흔들리는데 EKF 자체의 샘플 간 최대 이동은 0.21 m. EKF가 1 m 노이즈를 따라가지 않는다. mrm-off에서 gn-high로도 251 m를 멀쩡히 달린 것(5.2절)이 그 결과다.
   - **의미**: 논문의 "gn: Gaussian position noise (EKF input)"라는 표현은 정확히는 "GNSS 입력 노이즈"다. 내 loc_meas 채널(GNSS vs EKF 잔차)은 **입력 무결성**을 재는 것이지 EKF 저하를 재는 게 아니다. 이건 약점이 아니라 정리해서 써야 할 사실이다. gn-high에서 정지로 분기하는 건 "EKF가 망가져서"가 아니라 "GNSS를 믿을 수 없는 상태에서 이동을 보수적으로 포기"한 것이다. 실험 B에 gn 3회를 **영(null) 대조**로 넣어 EKF 오차가 안 변함을 수치로 보이고, 이 문장을 논문 3.2절 C와 6.4절에 추가한다.
   - 라이브 확인 명령 (Autoware 떠 있을 때):
     ```
     ros2 topic info -v /localization/pose_estimator/pose_with_covariance   # 발행자가 ndt_scan_matcher 인지
     ros2 param get /localization/pose_twist_fusion_filter/ekf_localizer pose_frame_id
     ```
2. **EKF에 실제로 영향을 주는 고장은 LiDAR(lso/lcl → NDT)와 IMU(imn/imb → gyro_odometer)다.** 그래서 실험 B의 독립변수는 gn이 아니라 **imn σ와 lso/lcl 강도**다. lso/lcl은 실험 A와 **같은 실행**에서 EKF 오차도 같이 기록하면 되므로 실행 수가 늘지 않는다.
3. **인지는 LiDAR 전용 centerpoint(tiny)다.** `perception_mode=lidar` (`autoware.launch.xml:47`), 카메라는 신호등 인식에만 쓰인다. 따라서 cn/cf는 객체 검출률을 바꾸지 않는다 → 실험 A에서 카메라 고장은 뺀다 (논문에 한 줄: "camera faults affect traffic-light recognition only in this configuration").
4. **정답(ground truth)이 없다.** 캠페인 rosbag 없음, CARLA 액터 위치 기록 없음. `hazard_injector`의 ego_x/y는 ROS 추정치. 리더보드 컨테이너 밖 CARLA 클라이언트는 `get_actors()`가 빈 리스트를 돌려준다(`results/actor_poll.log`). **GT 기록은 에이전트 프로세스 안(`aw_e2e.py`)에서 해야 한다.** 이미 `CarlaDataProvider.get_world().get_actors().filter("vehicle.*")`를 쓰는 코드가 l.465, 504에 있으니 거기에 붙이면 된다. 좌표계: Autoware y = −CARLA y. 고장 전 GNSS pose는 CARLA GT에서 온 값이라 base_link 대비 약 1.4 m 고정 오프셋이 있다.
5. 러너는 `mid/high`만 안다 (`experiment_runner.py:93-115 SEVERITY`). 스윕하려면 값을 직접 넘기는 옵션 하나 추가.
6. 실행 시간: 고장 케이스 wall 105~155 s, nf(끝까지 주행) 264 s, 여기에 CARLA 재시작·Autoware 기동 1~3분. **mrm-off로 끝까지 달리게 하면 1회 ≈ 6분, 시간당 10회.** GPU는 centerpoint+CARLA 8.6~10 GB / 16 GB.

## 2. 공통 설계 원칙

- **관찰 전용(mrm-off)**으로 돌린다. 감시기(cdr)는 켜서 점수를 기록하되 감독자는 `dummy_defer`로 바꿔 개입하지 않는다(러너 `--mrm-off`가 이미 이 구성). 그래야 고장 상태에서 계속 달리며 검출률·EKF 오차를 길게 잰다.
- **고장은 t=10 s에 켠다** (TRIGGER_RANGE 무작위 아님). 주행 구간 대부분이 고장 상태가 되게. 처음 10 s는 버린다.
- **교통은 def(방어형)**, `TM_SEED = 100+rep`로 고정. 같은 rep은 노이즈 수준이 달라도 같은 차들이 같은 자리에 나오므로 **수준 간 짝(paired) 비교**가 된다.
- **한 실행에서 A와 B를 동시에 기록**한다. 추가 비용 없음.
- 프레임 단위 수치는 같은 실행 안에서 서로 독립이 아니므로, 통계는 **실행(run) 단위 요약값**으로 한다. 셀당 n=5 (보조 블록은 3).

## 3. 실행 매트릭스 (총 146회, 하루치 ≈ 16시간)

시간당 10회 기준. 재실행 여유(CARLA_DIED 등 약 10 %)를 포함하면 17~18시간이라 하루 안에 끝난다.
**우선순위 순서대로 돌린다.** 중간에 끊겨도 앞쪽 블록만으로 논문 절이 성립하게.

| 순서 | 블록 | 고장 | 수준 (주입 값) | rep | 실행 수 | 목적 |
|---|---|---|---|---|---|---|
| 1 | N | nf | – | 5 | 5 | 기준선 (A·B 공통) |
| 2 | A1 | lso (T:폭) | 폭 90° 고정, T = 0.8, 0.6, 0.45, **0.33**(mid), 0.25, 0.2, 0.15, 0.12, **0.07** | 5 | 45 | 투과율 스윕 9단계. mid·high 값 포함. 0.33 근처(0.45/0.25)를 촘촘히 해서 0.7 문턱 위치를 잡는다 |
| 3 | A3 | lcl (mm/h:클러터) | 1:1000, 2:1100, **5:1500**(mid), 10:2100, 15:2750, **25:4000**(high), 40:5900 | 5 | 35 | 강우 스윕 7단계. 클러터 수는 mid–high 선형보간(125/mm·h) |
| 4 | B1 | imn (gyro σ) | 0.05, 0.1, 0.25, **0.5**(mid), 1.0, **1.5**(high) | 5 | 30 | EKF 오차 스윕 6단계 |
| 5 | B0 | gn (σ m) | 0.5, **1.0**, **2.5**, 5.0 | 3 | 12 | 영 대조에 기울기까지. EKF가 안 변하면 "입력 무결성" 확정, 변하면 그대로 스윕 데이터 |
| 6 | A2 | lso | T 0.33:150, T 0.07:150 (90°는 A1에 있음) | 5 | 10 | 폭 효과 분리 (high는 T와 폭을 동시에 바꿨으므로) |
| 7 | B2 | imb (gyro bias rad/s) | 0.1, **0.3**(mid), **1.0**(high) | 3 | 9 | 바이어스 채널(3 s 평균 차) 대 EKF 요 오차 |
| 합계 | | | | | **146** | ≈ 15 h + 재실행 ≈ 16~18 h |

밤 하나로 1~4(115회 ≈ 12 h)가 끝나고, 낮에 5~7(31회 ≈ 3.5 h)이 이어진다. 밤 시작을 저녁 6시로 잡으면 다음 날 오후 2시쯤 전부 끝난다.

**주입 모델이 예측하는 최대 검출 거리** (lidar_bridge.cpp 의 문턱 P_min = 0.9/(π·100²), CARLA 강도 ≈ e^(−0.004 r), lcl 은 e^(−2αr) 왕복 감쇠, lso 는 섹터 안 강도 × T²). 검출기(CenterPoint)의 D_det 는 이 값 **이하**로 나와야 정상이다(점 몇 개로는 객체를 못 만드니까). 두 스윕 모두 26 m 판단 창을 가로지르도록 짜여 있다.

| lcl mm/h | 0 | 1 | 2 | 5 (mid) | 10 | 15 | 25 (high) | 40 |
|---|---|---|---|---|---|---|---|---|
| α [1/m] | 0 | 0.010 | 0.015 | 0.026 | 0.040 | 0.051 | 0.069 | 0.092 |
| 가시거리 환산 ≈ 3/α [m] | ∞ | 300 | 200 | 114 | 75 | 59 | 43 | 33 |
| 점이 남는 최대 거리 [m] | 100 | 76 | 63 | 48 | 38 | 33 | **27** | 23 |

| lso T | 1.0 | 0.8 | 0.6 | 0.45 | 0.33 (mid) | 0.25 | 0.2 | 0.15 | 0.12 | 0.07 (high) |
|---|---|---|---|---|---|---|---|---|---|---|
| 섹터 안 최대 거리 [m] | 100 | 100 | 93 | 73 | **55** | 43 | 35 | 27 | 21 | **13** |

lcl-high(25 mm/h)가 "근거리만 보였다"는 기억이 맞다: 모델상 27 m 밖의 점은 전부 사라진다. 이는 26 m 판단 창의 바로 끝이라 lcl-high 개입은 "여유 0"으로 정당화된다. Kim 2023 실측(40 mm/h 에서 20~30 m 부터 비관측)과도 맞는다. lso 의 55 m / 13 m 는 논문 본문의 "mid ≈ 55 m, high ≈ 12 m" 그대로.

줄여야 하면 뒤에서부터 자른다: B2(−9) → A2(−10) → A1의 0.8·0.15(−10) → lcl 1·15(−10) = 107회.
결과가 경계에 걸린 수준(예: 점수가 0.7 바로 위/아래)은 그 수준만 rep 8로 보충(+3회/수준)한다.

### 3-1. GNSS 노이즈(gn)가 실제로 건드리는 것 — 이 Autoware 설정 기준

`/sensing/gnss/pose_with_covariance` 를 읽는 곳은 셋뿐이다 (launch 파일 grep 결과):
1. **pose_initializer** (`gnss_enabled`): 시동 때 NDT 초기 정렬 위치를 GNSS 로 잡는다. 주행 중 NDT 가 놓쳐서 **재초기화**할 때도 이걸 쓴다.
2. **ndt_scan_matcher 의 GNSS 정규화**: 특징 없는 구간에서 NDT 를 GNSS 쪽으로 당기는 기능. **이 설정에서는 꺼져 있음**.
3. eagleye 설정 파일: 사용 안 함.

그러므로 주행 중 gn 은 Autoware 의 EKF·계획·제어에 **아무 영향이 없다**. 영향이 있는 경우는 (a) NDT 가 무너져 재초기화할 때 → 2.5 m 틀린 자리에서 NDT 를 다시 맞추기 시작해 엉뚱한 곳에 수렴할 수 있음, (b) GNSS 정규화를 켠 설정, (c) GNSS/INS 가 pose source 인 차량(PCD 맵 없이 RTK 로 측위하는 경우) → 그때는 EKF 에 바로 들어가 위치가 튀고 경로·제어가 흔들린다. 실차(아이오닉6, RTK GPS)가 (c) 구성이면 gn 은 가장 직접적인 고장이다.

논문에서 gn 의 자리: "GNSS 입력 무결성 고장". 감시기의 loc_meas 는 EKF 가 아니라 **GNSS 가 믿을 만한가**를 재고, gn-high 에서 정지로 분기하는 것은 "재초기화·정규화에 쓸 GNSS 를 잃었으니 이동을 포기"하는 보수적 정책이다. Autoware 자체 감시기(localization_error_monitor: EKF 공분산 타원 횡 0.25 m 경고 / 0.3 m 오류, pose_instability_detector)는 GNSS 를 보지 않으므로 이 고장을 못 잡는다 — 5.2절 "gn-high 로 251 m 주행"의 기전.

**선택 블록 B3 (gn 이 EKF 에 들어가면 어떻게 되나, 6회)**: pose_twist_fusion_filter.launch.xml 의 `input_pose_with_cov_name` 을 `/sensing/gnss/pose_with_covariance` 로 바꾼 launch 로 Autoware 를 띄우면 GNSS 가 EKF 의 pose source 가 된다(NDT 출력은 무시됨). gn 1.0·2.5 × 3 회. 결과: e_lat P95 가 σ 에 비례해 커지고 cap_loc 교차와의 관계를 직접 볼 수 있다. 주의: 에이전트 GNSS pose 는 base_link 대비 1.4 m 오프셋이 있어 nf 에서도 차가 차로 안에서 치우칠 수 있다 → 먼저 nf 1회로 정상 주행이 되는지 확인하고, 안 되면 B3 은 뺀다. 이 블록은 "구성 (c) 차량이라면"이라는 가정 아래의 보조 결과로만 쓴다.

## 4. 기록할 것 (추가할 코드 4개)

### 4-1. GT 로거 (aw_e2e.py 안, env `GT_LOG=1`일 때만)
매 `run_step` 틱마다 `experiments/results/gt_<CASE_TAG>.csv` 한 줄씩:
```
sim_t, ego_x, ego_y, ego_yaw, ego_v,            # CARLA GT, Autoware 좌표계로 변환(y 부호 반전)
actor_id, type, x, y, yaw, len, wid, dist, bearing_deg   # 반경 80 m 안 vehicle.* / walker.* 전부, 액터마다 한 줄
```
`CarlaDataProvider.get_hero_actor().get_transform()`와 `get_world().get_actors()`로 충분하다. 기존 l.465/504의 루프를 복사하면 된다. 동기 모드 0.05 s 스텝이라 20 Hz. 파일 크기: 액터 20대 × 20 Hz × 300 s ≈ 12만 줄, 10 MB 수준.

### 4-2. ROS 기록 (호스트에서 `ros2 bag record`, 포인트클라우드는 제외)
```
/perception/object_recognition/detection/objects     # 검출기 출력(트래킹 전) ← 검출률은 이걸로
/perception/object_recognition/objects               # 트래킹 후(계획이 쓰는 것)
/localization/kinematic_state                        # EKF 최종
/localization/pose_estimator/pose_with_covariance    # NDT 출력
/localization/pose_estimator/transform_probability
/localization/pose_estimator/nearest_voxel_transformation_likelihood
/localization/pose_estimator/exe_time_ms
/sensing/gnss/pose_with_covariance                   # 고장 전엔 GT, 고장 후엔 오염 입력
/sensing/imu/imu_data
/monitor/scores                                      # cdr 점수 (LIDAR_front/rear quality, perception, obs, tl)
/fault_injection/active                              # 주입 시각 원장
/vehicle/status/velocity_status
```
cdr이 호스트에서 라이다 토픽을 구독하고 있으니 호스트에 같은 도메인으로 이미 보인다. 실행당 100~300 MB, 146회 ≈ 30~40 GB (여유 84 GB). 부족하면 bag 대신 CSV 기록기로 바꾼다(`record_ego_path2.py` 확장).
러너의 케이스 시작/종료 훅에 `ros2 bag record -o results/bag_<CASE_TAG> ...` / SIGINT를 넣는다 (`record_ego_path2.py`를 띄우는 자리와 같은 곳).

### 4-2b. 디스크 감시 (실행 내내)
146회를 무인으로 돌리는 동안 디스크가 차면 bag이 깨지고 CARLA가 죽는다. 배치 스크립트에 세 겹으로 넣는다.
- **실행 전 검사**: 케이스마다 시작 전에 `df --output=avail -BG /home`을 읽어 **여유 15 GB 미만이면 배치를 멈추고** `results/DISK_FULL` 파일을 남긴다(러너가 다음 케이스로 넘어가지 않게).
- **bag 상한**: `ros2 bag record --max-bag-size 400000000`(400 MB)로 한 실행이 폭주해도 400 MB에서 끊는다. 정상 실행은 100~300 MB.
- **백그라운드 기록**: `watch -n 300 'date; df -h /home | tail -1' >> results/disk_log.txt` 를 배치와 같이 띄워 5분마다 남긴다. 아침에 이 파일만 보면 밤새 추세가 보인다.
- 시작 전 정리: 지금 여유 84 GB. 146회 × 최대 300 MB = 44 GB가 상한이므로 시작 전에 **여유 60 GB 이상**을 확보한다(`~/autoware_carla_leaderboard/experiments/results/case_logs`의 오래된 gz, docker 이름 없는 이미지 `<none>` 5개 ≈ 70 GB가 후보. 지우기 전에 사용자 확인).

### 4-3. 러너 옵션 2개
- `--sev-value "0.2:90"`: SEVERITY 표 대신 이 문자열을 그대로 `fault=t:값`에 넣는다 (`fault_args()` l.189-221).
- `--t 10` 고정은 이미 있음. `--mrm-off --npc def --reps 3` 조합 사용.
- 배치 스크립트는 `experiments/speed_dose_batch.sh`를 복사해서 `noise_sweep_batch.sh`로. 한 줄 예:
  ```
  python3 experiments/experiment_runner.py --mrm-off --method ours --fault lso --sev-value 0.2:90 --t 10 --npc def --reps 3
  ```

### 4-4. 분석기 `experiments/analysis/noise_capability.py`
입력: gt_*.csv + bag → 출력: `noise_capability_runs.csv` (실행당 한 줄) + 그림.
매칭은 `~/carla_workspace/.../scripts/evaluate_bag.py`(중심거리 탐욕 매칭, 기본 4 m, 50 m 범위)를 가져다 쓰되 임계값을 2.0 m(주)·4.0 m(민감도)로 돌린다. IoU가 아니라 중심거리로 매칭하는 근거는 nuScenes 공식 지표(13절).

## 5. 지표

### 실험 A (인지) — 실행당 요약값
전방 섹터(ego 기준 ±45°, lso 폭 90°와 일치)·고장 후 구간만.
참고: 내 cdr의 공간-시간 TTC 창은 6 s(`awcl_adapter.py` TTC_HORIZON_S)로 Autoware의 5 s보다 1 s 길다. 감시기가 계획기보다 한 박자 먼저 본다는 근거로 3.4절에 한 줄 추가.

| 지표 | 정의 | 왜 |
|---|---|---|
| **R_h** (horizon recall) | GT 차량이 0~**26 m**에 있는 프레임 중 검출 객체 중심이 **2.0 m** 안에 있는 비율 (nuScenes TP 매칭 임계값. 4 m로도 재서 민감도 표기) | 1차 종점. 26 m = 제한속도 4.17 m/s × **Autoware 충돌 검사 창 5 s** + 정지 여유 5 m. 5 s는 이 설정의 obstacle_cruise `dense_time_horizon`(5.0), dynamic_obstacle_stop `time_horizon`(5.0), MPC 예측 궤적(50 step × 0.1 s)이 공통으로 쓰는 값이고, 5 m는 obstacle_stop `stop_margin`. 즉 "Autoware가 충돌 판단에 쓰는 구간 안에서 보느냐". 속도에 따라 창을 바꾸지 않는 이유: 고장 상태에서 느려지면 창이 줄어 검출률이 좋아 보이는 착시가 생기므로 제한속도 기준 고정. (8 m 돌발 대응은 거리가 아니라 감지 지연 문제라 5.4~5.6절이 다룬다) |
| R_26-50 | 26~50 m 구간 같은 정의 | 55 m 유효거리 주장 검증. 정상에서도 원거리는 놓치기 시작하므로 보조 |
| **D_det** | 각 GT 차량에 접근하면서 "3프레임 연속 검출"이 처음 성립한 거리의 중앙값 | 유효 검출거리. 논문의 "lso-mid ≈ 55 m, high ≈ 12 m" 직접 검증. 기준 40 m ≈ 1.5 × 26 m: 판단 창에 들어오기 전에 3프레임 확정(0.3 s)과 트래킹 지연을 치를 여유 |
| FP/s | GT 4 m 안에 없는 검출 객체 수 / 초 (nuScenes 최대 매칭 반경) | lcl 유령 객체 (mech_objects.log의 그 현상) |
| e_pos | 매칭된 객체 중심 오차 중앙값 | 보조 |
| **S_lidar** | 고장 후 `/monitor/scores` LIDAR_front quality_score 중앙값 | 내 점수 |
| **S_perc** | 고장 후 perception 점수 중앙값 | 결정 변수(0.7/0.3) |
| RR | R_h(수준) / R_h(nf 중앙값) | Robo3D의 resilience rate. 수준 간·문헌 간 비교용 정규화 값 |
| RCE | (R_h(nf) − R_h(수준)) / R_h(nf) | Dong 2023의 relative corruption error. RR = 1 − RCE |

### 실험 B (측위) — 실행당 요약값
EKF(`kinematic_state`) vs GT. 고정 오프셋(≈1.4 m)은 각 실행의 고장 전 10 s에서 평균으로 빼고 시작.

| 지표 | 정의 | 왜 |
|---|---|---|
| **e_lat P95** | 경로 중심선 기준 횡방향 EKF 오차 95퍼센타일 | 차로 유지에 직접 관련. 차로 3.5 m, 차폭 1.9 m → 여유 0.8 m. **0.5 m를 "이동 가능" 기준선**으로 둔다. 문헌 기준선도 같이 표시: Reid 2019 의 로컬도로 횡방향 경보한계 0.29 m(95 % 0.10 m), 고속도로 0.57 m(0.20 m). 0.5 m 는 둘 사이이고 저속(3 m/s) 대피라는 점으로 정당화. Autoware 자체 localization_error_monitor 의 횡방향 타원 기준(경고 0.25 m / 오류 0.3 m, 공분산 기준이라 실제 오차와는 다른 양)도 각주로 |
| e_pos RMSE / P95 | 위치 오차 | 보조 |
| e_yaw P95 | 요 오차 | imn에 민감 |
| NDT TP / NVTL 중앙값 | NDT 자체 신뢰도 | lso/lcl이 NDT에 주는 타격 |
| t_selfstop | 스톡이 스스로 멈춘 시각(있으면) | imn에서 Autoware 자체 정지와 비교 |
| NDT 발행률 | `/localization/pose_estimator/pose_with_covariance` 의 초당 메시지 수 (정상 10 Hz) | NVTL이 2.3 아래면 NDT가 pose 발행을 멈추고 EKF는 추측항법만 남는다(Autoware 문서). 이게 lso-high 오차 급증의 기전인지 확인 |
| **S_loc** | supervisor 로그의 cap_loc 중앙값 (mrm-off에서도 dummy 대신 `SUP_SHADOW`로 점수만 계산하게 하거나, 사후에 GNSS·EKF 토픽으로 `loc_meas` 식을 재계산) | 0.5 이동 최저선과 대조 |

`loc_meas = exp(−max(0, r − base − 0.2)/0.55)`, r = |GNSS − EKF| 는 bag의 두 토픽으로 오프라인 재계산이 되므로 supervisor를 꼭 띄울 필요는 없다.

## 6. 사전 등록 가설 (데이터 보기 전에 적어 두는 것)

- **H1 (단조성)** lso T↓, lcl 강우↑에 따라 R_h·D_det 비증가, S_lidar·S_perc 비증가. 실행 단위 Spearman ρ(수준, R_h) < 0, ρ(S_perc, R_h) > 0.
- **H2 (lso-mid 허용의 정당성)** T=0.33에서 R_h이 nf 대비 10 %p 이내(동등성 여유 10 %p, 짝 비교 5쌍 모두). D_det ≥ 40 m. 이 둘이 성립하면 "허용은 설계 의도대로"이고, 깨지면 "품질 채널이 둔감했다"로 논문을 고친다.
- **H3 (개입 문턱의 위치)** S_perc가 0.7 아래로 내려가는 첫 수준에서 R_h이 nf 대비 유의하게(짝 5쌍 모두 하락, 중앙값 −20 %p 이상) 떨어진다. 즉 0.7은 "못 보기 시작하는 지점"과 같은 수준에 있거나 그보다 앞선다(보수적).
- **H4 (lcl은 FP)** lcl에서 R_h 저하보다 FP/s 증가가 먼저 온다. 점수의 clutter 채널이 FP/s와 ρ < 0.
- **H5 (gn 영 대조)** gn 0.5~5.0 전 구간에서 e_lat P95, e_pos RMSE가 nf와 구별되지 않는다(차이 < 0.1 m). 반면 GNSS 잔차 기반 S_loc은 0.5 아래로 내려간다. → "loc 채널은 입력 무결성 지표"라는 문장을 확정.
- **H6 (imn)** σ↑에 따라 e_yaw P95·e_lat P95 단조 증가. S_loc(IMU bias 채널 포함 cap_loc)이 0.5를 지나는 σ가, e_lat P95가 0.5 m를 넘는 σ **이하**다(점수가 먼저 반응 = 보수적). 아니면 "점수가 늦다"로 기록.
- **H7 (lso/lcl → NDT)** lso T≤0.12 또는 lcl ≥25에서 NDT NVTL 저하·발행률 저하·e_pos 증가. 이것이 5.3절 "empty-point-cloud freeze"의 측위 측면 설명이 된다. 사전 기대: Burnett 2022(Boreas)는 실측 눈·비에서 LiDAR 측위가 생각보다 튼튼했다고 보고하므로 lcl-mid 까지는 NDT 가 버틸 가능성이 높다. 반면 lso 는 전방 섹터를 통째로 지우므로 NDT 에 더 아플 수 있다.

## 7. 통계 (작게, 정직하게)

- 셀당 n=5(실행), 보조 블록 n=3. 모든 수치는 실행당 점 + 중앙값으로 그리고, 구간은 부트스트랩(실행 단위) 95 %.
- 단조성: 실행 단위 Spearman ρ(수준 9단계 × 5 = 45점이면 CI가 좁아진다), 부트스트랩 CI. 짝 비교는 **방향 일치 횟수**(짝 5쌍 중 몇 쌍이 같은 방향; 5:0이면 부호검정 양측 p=0.0625, 논문 4.7절의 소표본 주의 문장 그대로).
- 동등성(H2): 짝 5쌍 차이의 최대값이 여유(10 %p) 안에 들어오면 "동등"으로. 검정력 주장은 안 한다.
- 프레임을 표본으로 세지 않는다(한 실행 안의 프레임은 독립이 아님). 논문 4.7절 문장 그대로 유지.
- **감시기를 "성능 저하 예측기"로 평가** (run-time monitoring 문헌 방식, 13절): 실행마다 R_h < R_h(nf) − 10 %p 이면 "저하"로 라벨링하고, S_perc 를 점수로 한 AUROC·AUPRC 와 0.7 문턱에서의 true-warning rate(저하 실행 중 점수가 0.7 아래인 비율)·false-warning rate 를 보고한다. 측위도 같은 틀: e_lat P95 > 0.5 m(또는 Reid 로컬도로 0.29 m) 를 "저하"로, S_loc 로 AUROC. 실행 수가 146이라 AUROC 의 부트스트랩 CI 가 나온다.

## 8. 그림·표 (논문 5.8절)

- **Fig 5.5** 2×2: (a) R_h·D_det vs T (lso), (b) R_h·FP/s vs 강우(lcl), (c) S_perc vs R_h 산점 (실행 단위, 0.7/0.3 선), (d) 실행당 e_lat P95 vs imn σ와 S_loc (0.5 선).
- **Table 5.6** 수준별 R_h / D_det / FP/s / S_lidar / S_perc / 결정 밴드(NORMAL·DEGRADED·CRITICAL).
- **Table 5.7** 수준별 e_lat P95 / e_pos RMSE / NVTL / S_loc / 이동 가능 여부(0.5 m 기준) — gn 두 줄은 "EKF 불변" 행.
- 본문: "점수가 0.7을 넘는 수준에서는 R_h이 nf의 ±x %p, D_det ≥ y m였고, 0.7 아래로 내려간 첫 수준부터 R_h이 z %p 떨어졌다" 식으로 숫자 채움. 한계: 단일 맵·단일 검출기(centerpoint tiny)·셀당 n=3.

## 9. 일정 (3~4일)

| 날 | 할 일 | 산출 |
|---|---|---|
| 1일 (반나절) | ① 라이브 확인(EKF 입력 토픽) ② GT 로거·bag 훅·`--sev-value` 구현 ③ nf 1회, lso 0.07:150 1회 스모크 → gt csv·bag·scores 다 찍히는지, 매칭기가 돌아가는지 | 코드 + 스모크 2회 |
| 1일 밤 | 순서 1~4 배치(115회 ≈ 12 h) 무인 실행 | bag 115개 |
| 2일 | 오전: 순서 5~7(31회) 이어서 실행 + 분석기 완성. 오후: 밤 데이터 1차 그림 | 표 5.6 초안 |
| 3일 | 전체 분석, 그림 2장, 표 2개, 5.8절·3.2절 C 보강·6.4절 한계 문장 | 논문 패치 |
| 4일 (예비) | 깨진 실행·경계 수준 보충(rep 8), 리뷰 | |

## 10. 위험과 대비

| 위험 | 대비 |
|---|---|
| lso T≤0.12 / lcl 40에서 NDT가 깨져 Autoware가 스스로 서 버림(5.3절 freeze) | 그것도 데이터다(t_selfstop, R_h은 멈추기 전 구간). 다만 전방 차량 표본이 줄어드니 D_det는 "측정 불가"로 표기 |
| def 교통이 전방 30 m 안에 잘 안 들어옴 → R_h 표본 부족 | 스모크에서 GT 로그로 30 m 내 프레임 수 확인. 부족하면 `npcadd`로 선행차 추가 또는 `cfl` 계열 선행차 스폰 코드 재사용(정차 차량 2대를 경로 옆에 고정 스폰) |
| bag 기록이 Autoware 성능에 영향 | 포인트클라우드 제외, 객체·pose 토픽만(수 KB/s). 영향 없음 |
| 에이전트 안 GT 로깅이 틱을 느리게 함 | 20 Hz × 액터 20대 CSV 쓰기는 ms 단위. 버퍼링 후 100틱마다 flush |
| rep 5로도 경계 수준 결론이 흔들림 | 사전 등록 가설·동등성 여유 명시. 경계 수준만 rep 8로 보충(+3회/수준) |
| gn 영 대조가 "영"이 아님(EKF가 GNSS를 어떤 경로로든 받고 있음) | 그러면 논문 표현이 맞는 것이고, B0의 4단계가 그대로 스윕 데이터가 된다. rep만 5로 올림(+8회) |

## 11. 논문에 들어갈 문장 (결과에 따라 고르는 두 갈래)

- **lso-mid가 R_h·D_det를 지키면**: "The tolerance of lso-mid is consistent with measured detector capability: recall within 30 m remained within x %p of fault-free and the effective detection range was y m, whereas the first level at which the perception score crossed θ_H coincided with a z %p recall drop. The score threshold therefore sits at or before the onset of capability loss."
- **못 지키면**: "The quality channel under-reacts to sparsification: at T=0.33 recall within 30 m had already fallen by x %p while the score stayed above θ_H. The continued driving under lso-mid in Table 5.2 is thus a detector blind spot, not a design choice; the count/far channels should be re-weighted (future work)."
- **gn에 대해 (어느 경우든)**: "GNSS position noise does not propagate to the EKF in the evaluated configuration (NDT pose source, GNSS regularization disabled): EKF lateral error under gn-high was indistinguishable from fault-free. The localization residual channel therefore monitors input integrity rather than estimator degradation, and the gn-high deferral is a conservative policy on untrusted input, not a response to measured estimator error."

## 12. 참고 위치 (이 PC)

- 주입 코드: `~/autoware_carla_leaderboard/src/tum_agents/autoware_agent/aw_e2e.py` (lso l.675-690, lcl l.694-710, imn l.714/1058-1080, gn l.977-991), 브리지 물리: `src/autoware_carla_cpp_bridge/.../lidar_bridge.cpp:161-251`
- 점수: `~/fs_ws/src/cdr_monitor/include/cdr_monitor/lidar_quality_check.hpp` (σ l.125-142, count l.153, clutter l.187, CUSUM l.222), `publish_scores.hpp` (섹터 l.88-120); loc 채널: `~/fs_ws/src/mrm_supervisor/mrm_supervisor/supervisor.py:472-516, 592-603`
- 러너: `~/autoware_carla_leaderboard/experiments/experiment_runner.py` (SEVERITY l.93, fault_args l.189, --mrm-off l.710/1004-1035)
- 기존 결과: `~/autoware_carla_leaderboard/experiments/results/` (results_*.csv, case_logs/, scoretrace_*.csv, ego_path_gnmid.csv)
- 매칭기 재사용: `~/carla_workspace/src/monodetr_dgwa/scripts/evaluate_bag.py`
- Autoware 설정: `~/autoware_awcl/autoware/src/launcher/autoware_launch/autoware_launch/` (launch/autoware.launch.xml, config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml)

## 13. 참고 문헌과 적용 방식 (인식률·측위 평가 방법을 어디서 빌려오나)

2026-11-04 조사. 논문 참고문헌 [63] 이후로 추가한다. 세 갈래로 묶고, 각 갈래에서 **무엇을 빌리는지**를 명시한다.

### 13-1. "노이즈 강도 스윕 + 상대 저하율" — 손상 강건성 벤치마크 계열

| 문헌 | 방법 | 우리가 빌리는 것 |
|---|---|---|
| Hendrycks & Dietterich, ICLR 2019 (ImageNet-C) | 손상 유형 × 심각도 1~5 단계, 기준 모델 대비 상대 오류 | "심각도 단계 스윕"이라는 실험 형식 자체 |
| **Dong et al., CVPR 2023** "Benchmarking Robustness of 3D Object Detection to Common Corruptions" | LiDAR·카메라 27종 손상 × 5단계. LiDAR fog α ∈ {0.005, 0.01, 0.02, 0.03, 0.06}, rain/snow 강도 {0.2…7.29}, density decrease {6…30 %}. 지표 AP_cor = 손상·단계 평균 AP, **RCE = (AP_clean − AP_cor)/AP_clean** | ① RCE 정의 그대로 가져와 R_h·D_det 에 적용. ② 우리 lcl 의 Beer-Lambert α = 0.01·R^0.6 는 5 mm/h → 0.026, 25 mm/h → 0.069, 40 mm/h → 0.091 로 Dong 의 fog 단계 3~5 와 같은 α 범위다. 표에 "Dong 심각도 환산" 열을 넣으면 우리 mid/high 가 문헌에서 어느 단계인지 한눈에 보인다. Dong/Hahner 의 fog 도 같은 e^(−2αr) 감쇠라 α=0.06(5단계, 가시거리 50 m)에서는 그쪽 검출기도 원거리를 잃는다 — 우리 lcl-high 가 27 m 밖을 잃는 것과 같은 체제. ③ lso 의 sector dropout 은 Dong 의 "FOV lost"(각도 범위 축소)·"local density decrease" 와 같은 종류 — 관련 문장에 인용 |
| **Kong et al., ICCV 2023** "Robo3D" | 8종 손상 × 3단계(fog, wet ground, snow, motion blur, beam missing, crosstalk, incomplete echo, cross-sensor). **기준 모델이 CenterPoint**. mCE(CenterPoint 대비 오류 비), **mRR = 손상 평균 정확도 / 정상 정확도** | ① RR(resilience rate) 를 수준별 정규화 지표로 채택: RR = R_h(수준)/R_h(nf). ② 우리 검출기가 바로 CenterPoint 라서 Robo3D 의 CenterPoint fog/snow/beam-missing 수치를 "외부 참조값"으로 옆에 둘 수 있다 (데이터셋이 달라 절대 비교는 아니고 저하 폭의 크기 감 비교). |
| **Hahner et al., ICCV 2021** (fog) / CVPR 2022 (snow) — 이미 [58][59] | 물리 기반 fog 시뮬레이션, α ↔ MOR(가시거리) 대응: 0.005→600 m, 0.01→300, 0.02→150, 0.03→100, 0.06→50 m. AP@0.5 IoU(car), 40 recall 포인트, 날씨 분할(clear/light/dense fog)로 보고 | α→가시거리 환산표를 그대로 써서 lcl 각 수준에 "MOR 환산 ≈ xx m" 를 붙인다 (독자가 mm/h 보다 가시거리를 직관적으로 안다). 거리별 분해는 Hahner 도 안 했으므로 우리의 R_h / R_26-50 분해는 추가 기여로 쓸 수 있다 |

### 13-2. "센서 저하 ↔ 검출 거리" — 실측 열화 계열 (이미 일부 인용)

| 문헌 | 방법 | 우리가 빌리는 것 |
|---|---|---|
| **Bijelic, Gruber, Ritter, IV 2018** — [20] | 안개 챔버, 가시거리 V = 45/36/24/13 m, 반사율 5/50/90 % 표적. 지표 **d_max = 표적 위 마지막 가시 점까지 거리**, 깊이 구간별 점 수. 결론 "가시거리 40 m 아래면 인지 25 m 미만" | D_det 는 d_max 의 "검출기 수준" 판 — 점이 아니라 **검출기가 객체로 인식한 첫 거리**. 이 대응을 지표 정의문에 한 줄로 쓴다. 저하 수준 대 유효거리 곡선(우리 Fig 5.5a)의 형식 선례 |
| **Kim, Park, Kim, Sensors 2023** — [23] | 실도로(KICT 기상재현도로), 비 10/20/30/40 mm/h, 안개 가시거리 50/100/150 m, 거리 10~50 m, 재질 5종, **셀당 5회 반복**, 지표 NPC(표적 점 수)·intensity, **유효 관측 5회 미만이면 "non-observation"**, 일원 ANOVA + Tukey HSD. 결과: 40 mm/h·안개 50 m 에서 20~30 m 부터 비관측, NPC 최대 −56 %, intensity 최대 −73 % | ① 셀당 5회 반복이 선례가 있다 → 우리 rep 5 의 근거. ② "non-observation" 정의를 빌려 D_det 산출 불가 셀을 같은 용어로 표기. ③ 우리 lcl 40 mm/h 결과를 이 논문의 "40 mm/h 에서 20~30 m 비관측"과 대조하는 문장 하나 (시뮬레이션 클러터 모델의 현실성 점검). ④ 통계는 ANOVA 대신 실행 단위 Spearman·부트스트랩을 쓰되, 그 이유(프레임 비독립, 단조성 질문)를 4.7절에 한 줄 |
| Goodin 2019 [53], Kilic LISA 2021 [54], Trierweiler 2019 [55] | 이미 lcl/lso 모델의 출처 | 변경 없음. 스윕 값(mm/h, T)이 이 모델들의 입력이라는 점만 재확인 |

### 13-3. "감시 신호가 실제 검출 성능을 맞히는가" — 런타임 모니터링 계열 (논문 2.5절에 추가할 갈래)

| 문헌 | 방법 | 우리가 빌리는 것 |
|---|---|---|
| **Rahman, Sünderhauf, Dayoub, WACV Workshops 2021** "Per-frame mAP prediction for continuous performance monitoring of object detection during deployment" | 정답 없이 검출기 내부 특징으로 **프레임별 mAP 가 임계값 λ(k 퍼센타일 또는 0.5) 아래인지** 예측. 평가는 AUROC/AUPRC, **true warning rate**(저하 프레임 중 경보 비율), 위험회피 점수 | **우리 7절의 AUROC 분석이 이 틀이다.** 다른 점: 그들은 검출기 내부 활성값, 우리는 센서 물리 채널(σ, 점 수, 클러터)로 예측 → "검출기와 독립적인 신호로 같은 일을 한다"고 자리매김. 저하 라벨 정의(R_h 가 nf − 10 %p 아래)를 그들의 λ 와 같은 역할로 설명 |
| Rahman, Corke, Dayoub, IEEE Access 2021 "Run-time monitoring of machine learning for robotic perception: a survey of emerging trends" | 런타임 모니터링 분류 체계 | 2.5절에 "performance monitoring" 갈래를 소개하는 인용. 우리 감시기가 그 분류의 어디(입력 수준 모니터)인지 한 줄 |
| Yatbaz et al., CVPR Workshops 2024 "Run-time monitoring of 3D object detection in automated driving systems" | KITTI/nuScenes, **PointPillars·CenterPoint** 의 활성 패턴으로 3D 검출 오류 예측 | 3D·CenterPoint 로 같은 질문을 다룬 가장 가까운 선행. 우리와의 차이(폐루프 스택, 물리 채널)만 적고 수치 비교는 안 함 |

### 13-4. 측위

| 문헌 | 방법 | 우리가 빌리는 것 |
|---|---|---|
| **Reid et al., SAE Int. J. CAV 2019** — [29] | 차량 측위 요구치: 로컬도로 횡 0.29 m(95 % 0.10 m)·종 0.29 m·자세 0.50°, 고속도로 횡 0.57 m(0.20 m) | 표 5.7 에 0.29 m / 0.57 m 두 선을 긋고 우리 0.5 m 가 그 사이임을 보인다. cap_loc 0.5 교차점이 0.29 m 교차점보다도 앞서면 "엄격 기준으로도 보수적", 0.57 m 보다만 앞서면 "고속도로 기준으로 보수적" 으로 등급을 나눠 서술 |
| **Sturm et al., IROS 2012** (TUM RGB-D benchmark) | ATE(정렬 후 절대 궤적 오차 RMSE), RPE(구간 상대 오차 = 드리프트) | EKF 대 GT 를 ATE(오프셋 정렬 후 RMSE·P95)로, imn/imb 드리프트는 RPE(1 s 구간)로 보고. `evo` 도구 그대로 사용 가능 |
| Autoware `ndt_scan_matcher` 문서 / Autoware Discussion #5628, #5135 | NVTL 임계 2.3(converged_param). **점수가 낮으면 NDT 가 pose 발행을 멈추고 EKF 는 속도로 추측항법**. 실도로 사례에서 NVTL 이 측위 붕괴를 못 잡은 보고(#5135) | ① "NDT 발행률" 지표의 근거. ② NVTL 만으로는 부족하니 GT 오차로 직접 재야 한다는 논거(4.9절 설계 동기). ③ lso-high 에서 EKF 오차가 커지는 기전 설명 |
| **Burnett et al., RA-L 2022** "Are we ready for radar to replace lidar in all-weather mapping and localization?" (Boreas) | 1년 반복 주행, 눈보라에서도 LiDAR 측위가 최고 정확도. "중간 강수에 대한 LiDAR 측위 민감도는 과장돼 왔다" | H7 의 사전 기대. lcl-mid 에서 NDT 가 안 흔들리면 이 보고와 일치한다고 쓴다 |
| **Matos, Durães, Cunha, Informatics 2025** "Simulating the effects of sensor failures on autonomous vehicles for safety evaluation" | CARLA + Autoware, ROS 브리지에서 LiDAR/GNSS/IMU 고장 주입, 주행 결과(충돌·이상 거동)로 평가 | 2.6절의 가장 가까운 동시대 연구. 차이: 그들은 주행 결과, 우리는 **인지·측위 능력 자체와 감시 점수의 대응**을 잰다 |

### 13-5. 지표 정의의 출처 (한 줄씩)
- 중심거리 매칭(IoU 대신), 임계 {0.5, 1, 2, 4} m, TP 지표는 2 m: **Caesar et al., CVPR 2020 (nuScenes)**. 우리 2.0 m(주)·4.0 m(민감도)의 근거.
- 거리 구간별 분해 [0, 30), [30, 50), [50, ∞): **Sun et al., CVPR 2020 (Waymo Open Dataset)** 의 관례(공식은 35 m 경계, 논문들은 30 m). 우리는 26/50 m 로 자르되 "Waymo 식 거리 구간 분해"라고 소개.

### 13-6. 논문에 넣는 위치
- 2.3절 끝에 한 단락: 손상 강건성 벤치마크(Dong, Kong)와 실측 열화(Bijelic, Kim)는 **검출기 성능**을 재고, 2.5절의 런타임 모니터링(Rahman, Yatbaz)은 **그 성능을 정답 없이 예측**한다. 본 연구의 추가 실험은 둘을 잇는다: 물리 채널 점수가 폐루프 스택의 실제 검출·측위 능력을 얼마나 추적하는지.
- 4.9절 "Capability-correlation experiment design": 매트릭스, 지표(R_h, D_det, FP/s, RR/RCE, ATE, NDT 발행률), 라벨 정의, AUROC.
- 5.8절 결과 + Table 5.6/5.7 + Fig 5.5.
- 6.4절 한계: 단일 검출기(CenterPoint tiny)·단일 맵·시뮬레이션 클러터 모델, Kim 2023 과의 정성 대조만 가능.
