# 추가 실험 설계: 노이즈 강도 ↔ Autoware 인지·측위 저하 ↔ 건강 점수 상관관계

작성 2026-11-04. 대상: 석사논문(Continuous Sensor Health Monitoring and Risk/Capability-aware Graded MRM Supervision) 보강 실험.
장비: 연구실 RTX 5080 PC 한 대. 기간: 3~4일. 총 실행 수 60회 이내(대규모 캠페인 아님).

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
- 프레임 단위 수치는 같은 실행 안에서 서로 독립이 아니므로, 통계는 **실행(run) 단위 요약값**으로 한다. 셀당 n=3.

## 3. 실행 매트릭스 (총 55회, 약 6시간)

| 블록 | 고장 | 수준 (주입 값) | rep | 실행 수 | 목적 |
|---|---|---|---|---|---|
| N | nf | – | 3 | 3 | 기준선 (A·B 공통) |
| A1 | lso (T:폭) | 폭 90° 고정, T = 0.6, 0.45, **0.33**(mid), 0.2, 0.12, **0.07** | 3 | 18 | 투과율 스윕. mid·high 값 포함 |
| A2 | lso | T 0.07 고정, 폭 = 90, **150**(high) | 3 | 6 | 폭 효과 분리 (high는 T와 폭을 동시에 바꿨으므로) |
| A3 | lcl (mm/h:클러터) | 2:1100, **5:1500**(mid), 10:2100, **25:4000**(high), 40:5900 | 3 | 15 | 강우 스윕. 클러터 수는 mid–high 사이 선형보간 (5→1500, 25→4000: 125/mm·h) |
| B1 | imn (gyro σ) | 0.1, 0.25, **0.5**(mid), 1.0, **1.5**(high) | 2 | 10 | EKF 오차 스윕 |
| B0 | gn (σ m) | **1.0**, **2.5** | 2 | 4 | 영 대조: EKF 오차 불변 확인 |
| 합계 | | | | **56** | ≈ 6 h (시간당 10회) |

밤 1회로 N+A1+A2+A3(42회 ≈ 4.5 h), 다음 날 낮에 B1+B0(14회 ≈ 1.5 h). imn은 스톡이 스스로 멈추는 경우(5.2절: 71 m)가 있어 더 짧다.

줄여야 하면: A2 삭제(−6), A3의 2:1100·40:5900 삭제(−6) → 44회. 늘릴 여유가 있으면 A1·A3 rep 5로.

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
cdr이 호스트에서 라이다 토픽을 구독하고 있으니 호스트에 같은 도메인으로 이미 보인다. 실행당 100~300 MB, 56회 ≈ 15 GB (여유 84 GB).
러너의 케이스 시작/종료 훅에 `ros2 bag record -o results/bag_<CASE_TAG> ...` / SIGINT를 넣는다 (`record_ego_path2.py`를 띄우는 자리와 같은 곳).

### 4-3. 러너 옵션 2개
- `--sev-value "0.2:90"`: SEVERITY 표 대신 이 문자열을 그대로 `fault=t:값`에 넣는다 (`fault_args()` l.189-221).
- `--t 10` 고정은 이미 있음. `--mrm-off --npc def --reps 3` 조합 사용.
- 배치 스크립트는 `experiments/speed_dose_batch.sh`를 복사해서 `noise_sweep_batch.sh`로. 한 줄 예:
  ```
  python3 experiments/experiment_runner.py --mrm-off --method ours --fault lso --sev-value 0.2:90 --t 10 --npc def --reps 3
  ```

### 4-4. 분석기 `experiments/analysis/noise_capability.py`
입력: gt_*.csv + bag → 출력: `noise_capability_runs.csv` (실행당 한 줄) + 그림.
매칭은 `~/carla_workspace/.../scripts/evaluate_bag.py`(중심거리 탐욕 매칭, 기본 4 m, 50 m 범위)를 가져다 쓴다.

## 5. 지표

### 실험 A (인지) — 실행당 요약값
전방 섹터(ego 기준 ±45°, lso 폭 90°와 일치)·고장 후 구간만.

| 지표 | 정의 | 왜 |
|---|---|---|
| **R30** | GT 차량이 0~30 m에 있는 프레임 중 검출 객체가 2.5 m 안에 있는 비율 | 1차 종점. 8 m 돌발 대응에 필요한 거리 |
| R50 | 30~50 m 구간 같은 정의 | 55 m 유효거리 주장 검증 |
| **D_det** | 각 GT 차량에 접근하면서 "3프레임 연속 검출"이 처음 성립한 거리의 중앙값 | 유효 검출거리. 논문의 "lso-mid ≈ 55 m, high ≈ 12 m" 직접 검증 |
| FP/s | GT 4 m 안에 없는 검출 객체 수 / 초 | lcl 유령 객체 (mech_objects.log의 그 현상) |
| e_pos | 매칭된 객체 중심 오차 중앙값 | 보조 |
| **S_lidar** | 고장 후 `/monitor/scores` LIDAR_front quality_score 중앙값 | 내 점수 |
| **S_perc** | 고장 후 perception 점수 중앙값 | 결정 변수(0.7/0.3) |

### 실험 B (측위) — 실행당 요약값
EKF(`kinematic_state`) vs GT. 고정 오프셋(≈1.4 m)은 각 실행의 고장 전 10 s에서 평균으로 빼고 시작.

| 지표 | 정의 | 왜 |
|---|---|---|
| **e_lat P95** | 경로 중심선 기준 횡방향 EKF 오차 95퍼센타일 | 차로 유지에 직접 관련. 차로 3.5 m, 차폭 1.9 m → 여유 0.8 m. **0.5 m를 "이동 가능" 기준선**으로 둔다 |
| e_pos RMSE / P95 | 위치 오차 | 보조 |
| e_yaw P95 | 요 오차 | imn에 민감 |
| NDT TP / NVTL 중앙값 | NDT 자체 신뢰도 | lso/lcl이 NDT에 주는 타격 |
| t_selfstop | 스톡이 스스로 멈춘 시각(있으면) | imn에서 Autoware 자체 정지와 비교 |
| **S_loc** | supervisor 로그의 cap_loc 중앙값 (mrm-off에서도 dummy 대신 `SUP_SHADOW`로 점수만 계산하게 하거나, 사후에 GNSS·EKF 토픽으로 `loc_meas` 식을 재계산) | 0.5 이동 최저선과 대조 |

`loc_meas = exp(−max(0, r − base − 0.2)/0.55)`, r = |GNSS − EKF| 는 bag의 두 토픽으로 오프라인 재계산이 되므로 supervisor를 꼭 띄울 필요는 없다.

## 6. 사전 등록 가설 (데이터 보기 전에 적어 두는 것)

- **H1 (단조성)** lso T↓, lcl 강우↑에 따라 R30·D_det 비증가, S_lidar·S_perc 비증가. 실행 단위 Spearman ρ(수준, R30) < 0, ρ(S_perc, R30) > 0.
- **H2 (lso-mid 허용의 정당성)** T=0.33에서 R30이 nf 대비 10 %p 이내(동등성 여유 10 %p, 짝 비교 3쌍 모두). D_det ≥ 40 m. 이 둘이 성립하면 "허용은 설계 의도대로"이고, 깨지면 "품질 채널이 둔감했다"로 논문을 고친다.
- **H3 (개입 문턱의 위치)** S_perc가 0.7 아래로 내려가는 첫 수준에서 R30이 nf 대비 유의하게(짝 3쌍 모두 하락, 중앙값 −20 %p 이상) 떨어진다. 즉 0.7은 "못 보기 시작하는 지점"과 같은 수준에 있거나 그보다 앞선다(보수적).
- **H4 (lcl은 FP)** lcl에서 R30 저하보다 FP/s 증가가 먼저 온다. 점수의 clutter 채널이 FP/s와 ρ < 0.
- **H5 (gn 영 대조)** gn 1.0·2.5에서 e_lat P95, e_pos RMSE가 nf와 구별되지 않는다(차이 < 0.1 m). 반면 GNSS 잔차 기반 S_loc은 0.5 아래로 내려간다. → "loc 채널은 입력 무결성 지표"라는 문장을 확정.
- **H6 (imn)** σ↑에 따라 e_yaw P95·e_lat P95 단조 증가. S_loc(IMU bias 채널 포함 cap_loc)이 0.5를 지나는 σ가, e_lat P95가 0.5 m를 넘는 σ **이하**다(점수가 먼저 반응 = 보수적). 아니면 "점수가 늦다"로 기록.
- **H7 (lso/lcl → NDT)** lso T≤0.12 또는 lcl ≥25에서 NDT NVTL 저하·e_pos 증가. 이것이 5.3절 "empty-point-cloud freeze"의 측위 측면 설명이 된다.

## 7. 통계 (작게, 정직하게)

- 셀당 n=3(실행). 모든 수치는 실행당 점 + 중앙값으로 그리고, 구간은 부트스트랩(실행 단위) 95 %.
- 단조성: 실행 단위 Spearman ρ, 부트스트랩 CI. n이 작으니 p값보다 **방향 일치 횟수**(짝 3쌍 중 몇 쌍이 같은 방향)를 같이 보고.
- 동등성(H2): 짝 차이의 최대값이 여유 안에 들어오면 "동등"으로. 검정력 주장은 안 한다.
- 프레임을 표본으로 세지 않는다(한 실행 안의 프레임은 독립이 아님). 논문 4.7절 문장 그대로 유지.

## 8. 그림·표 (논문 5.8절)

- **Fig 5.5** 2×2: (a) R30·D_det vs T (lso), (b) R30·FP/s vs 강우(lcl), (c) S_perc vs R30 산점 (실행 단위, 0.7/0.3 선), (d) 실행당 e_lat P95 vs imn σ와 S_loc (0.5 선).
- **Table 5.6** 수준별 R30 / D_det / FP/s / S_lidar / S_perc / 결정 밴드(NORMAL·DEGRADED·CRITICAL).
- **Table 5.7** 수준별 e_lat P95 / e_pos RMSE / NVTL / S_loc / 이동 가능 여부(0.5 m 기준) — gn 두 줄은 "EKF 불변" 행.
- 본문: "점수가 0.7을 넘는 수준에서는 R30이 nf의 ±x %p, D_det ≥ y m였고, 0.7 아래로 내려간 첫 수준부터 R30이 z %p 떨어졌다" 식으로 숫자 채움. 한계: 단일 맵·단일 검출기(centerpoint tiny)·셀당 n=3.

## 9. 일정 (3~4일)

| 날 | 할 일 | 산출 |
|---|---|---|
| 1일 (반나절) | ① 라이브 확인(EKF 입력 토픽) ② GT 로거·bag 훅·`--sev-value` 구현 ③ nf 1회, lso 0.07:150 1회 스모크 → gt csv·bag·scores 다 찍히는지, 매칭기가 돌아가는지 | 코드 + 스모크 2회 |
| 1일 밤 | N+A1+A2+A3 배치(42회) 무인 실행 | bag 42개 |
| 2일 | 분석기 완성, 밤 데이터 1차 그림. 오후에 B1+B0(14회) | 표 5.6 초안 |
| 3일 | 전체 분석, 그림 2장, 표 2개, 5.8절·3.2절 C 보강·6.4절 한계 문장 | 논문 패치 |
| 4일 (예비) | 깨진 실행 재실행, 리뷰 | |

## 10. 위험과 대비

| 위험 | 대비 |
|---|---|
| lso T≤0.12 / lcl 40에서 NDT가 깨져 Autoware가 스스로 서 버림(5.3절 freeze) | 그것도 데이터다(t_selfstop, R30은 멈추기 전 구간). 다만 전방 차량 표본이 줄어드니 D_det는 "측정 불가"로 표기 |
| def 교통이 전방 30 m 안에 잘 안 들어옴 → R30 표본 부족 | 스모크에서 GT 로그로 30 m 내 프레임 수 확인. 부족하면 `npcadd`로 선행차 추가 또는 `cfl` 계열 선행차 스폰 코드 재사용(정차 차량 2대를 경로 옆에 고정 스폰) |
| bag 기록이 Autoware 성능에 영향 | 포인트클라우드 제외, 객체·pose 토픽만(수 KB/s). 영향 없음 |
| 에이전트 안 GT 로깅이 틱을 느리게 함 | 20 Hz × 액터 20대 CSV 쓰기는 ms 단위. 버퍼링 후 100틱마다 flush |
| rep 3으로 결론이 흔들림 | 사전 등록 가설·동등성 여유 명시. 결과가 경계에 걸리면 그 수준만 rep 5로 보충(+4회) |
| gn 영 대조가 "영"이 아님(EKF가 GNSS를 어떤 경로로든 받고 있음) | 그러면 논문 표현이 맞는 것이고, 실험 B에 gn 스윕(0.5, 1.0, 1.5, 2.5 × 2)을 8회 추가 |

## 11. 논문에 들어갈 문장 (결과에 따라 고르는 두 갈래)

- **lso-mid가 R30·D_det를 지키면**: "The tolerance of lso-mid is consistent with measured detector capability: recall within 30 m remained within x %p of fault-free and the effective detection range was y m, whereas the first level at which the perception score crossed θ_H coincided with a z %p recall drop. The score threshold therefore sits at or before the onset of capability loss."
- **못 지키면**: "The quality channel under-reacts to sparsification: at T=0.33 recall within 30 m had already fallen by x %p while the score stayed above θ_H. The continued driving under lso-mid in Table 5.2 is thus a detector blind spot, not a design choice; the count/far channels should be re-weighted (future work)."
- **gn에 대해 (어느 경우든)**: "GNSS position noise does not propagate to the EKF in the evaluated configuration (NDT pose source, GNSS regularization disabled): EKF lateral error under gn-high was indistinguishable from fault-free. The localization residual channel therefore monitors input integrity rather than estimator degradation, and the gn-high deferral is a conservative policy on untrusted input, not a response to measured estimator error."

## 12. 참고 위치 (이 PC)

- 주입 코드: `~/autoware_carla_leaderboard/src/tum_agents/autoware_agent/aw_e2e.py` (lso l.675-690, lcl l.694-710, imn l.714/1058-1080, gn l.977-991), 브리지 물리: `src/autoware_carla_cpp_bridge/.../lidar_bridge.cpp:161-251`
- 점수: `~/fs_ws/src/cdr_monitor/include/cdr_monitor/lidar_quality_check.hpp` (σ l.125-142, count l.153, clutter l.187, CUSUM l.222), `publish_scores.hpp` (섹터 l.88-120); loc 채널: `~/fs_ws/src/mrm_supervisor/mrm_supervisor/supervisor.py:472-516, 592-603`
- 러너: `~/autoware_carla_leaderboard/experiments/experiment_runner.py` (SEVERITY l.93, fault_args l.189, --mrm-off l.710/1004-1035)
- 기존 결과: `~/autoware_carla_leaderboard/experiments/results/` (results_*.csv, case_logs/, scoretrace_*.csv, ego_path_gnmid.csv)
- 매칭기 재사용: `~/carla_workspace/src/monodetr_dgwa/scripts/evaluate_bag.py`
- Autoware 설정: `~/autoware_awcl/autoware/src/launcher/autoware_launch/autoware_launch/` (launch/autoware.launch.xml, config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml)
