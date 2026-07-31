# fs_failsafe 초보자 안내서

> 대상 독자: 알고리즘은 알지만 ROS2 / BehaviorTree / C++ / CARLA는 처음인 대학원생.
> 이 문서만 읽으면 코드를 열었을 때 "이게 왜 여기 있지?"가 줄어들도록 만들었다.

---

## 1. 이 레포 전체 지도

한 줄 요약: **센서가 고장나면 점수가 떨어지고, 점수가 떨어지면 차가 안전한 곳에 세워지는 시스템.**

```
[센서: 라이다/카메라/IMU/GPS]  (CARLA 시뮬레이터 또는 실차)
        │  토픽 (포인트클라우드, 이미지, ...)
        ▼
[cdr_monitor]  ─ C++ BehaviorTree. 센서마다 "신선한가(FTTI)" x "품질 좋은가" 점수를 매김
        │  /monitor/scores (점수 전체) + /diagnostics (고장 알림)
        ▼
[mrm_supervisor]  ─ Python 상태기계. 점수를 보고 "정상 / 갓길로 피하자 / 그냥 세우자" 판단
        │  경로 변경 요청 (서비스 호출) 또는 DEFER (Autoware에게 위임)
        ▼
[Autoware]  ─ 자율주행 스택. 실제로 핸들을 꺾고 브레이크를 밟는 실행자
        │
        ▼
[CARLA]  ─ 시뮬레이터. 가짜 도시에서 차를 굴리고 센서 데이터를 만들어 줌
```

폴더별 역할:

| 폴더 | 역할 |
|---|---|
| `ros/cdr_monitor/` | 센서 감시자 (C++, BehaviorTree). 점수 계산의 본체 |
| `ros/mrm_supervisor/` | 판단자 (Python). 점수 → 상태기계 → 갓길 대피 or 위임 |
| `ros/hesai_zed_fusion/` | 실차 포팅용 라이다+카메라 융합 (시뮬레이션에선 안 씀) |
| `agent/aw_e2e.py` | CARLA 리더보드 에이전트. 고장 주입(노이즈, 센서 차단)도 담당 |
| `experiments/` | 자동 반복실험 프레임워크 (실행 → 판정 → CSV 기록) |
| `patches/` | Autoware, CARLA 브리지 등 서드파티 저장소에 가한 수정분 |
| `scripts/` | 실행 스크립트 + CycloneDDS 설정 |

---

## 2. 5분 ROS2 입문

ROS2는 "로봇 프로그램들이 서로 대화하는 규칙"이다.

- **노드(node)** = 프로그램 하나. cdr_monitor도 노드, mrm_supervisor도 노드.
- **토픽(topic)** = 방송 채널. 라디오 주파수라고 생각하면 된다.
  - **발행(publish)** = 채널에 방송을 내보내는 것.
  - **구독(subscribe)** = 채널을 틀어놓고 듣는 것. 방송이 올 때마다 콜백 함수가 실행된다.
  - 누가 듣는지 몰라도 방송할 수 있다. 이게 ROS의 핵심 장점 (프로그램끼리 직접 연결 안 함).
- **서비스(service)** = 방송이 아니라 전화. "경로 바꿔줘" → "알겠어/실패했어" 응답이 온다.
  mrm_supervisor가 Autoware에 경로 변경을 요청할 때 쓴다.
- **파라미터(parameter)** = 노드의 설정값. 코드 안의 `declare_parameter("conf", 0.3)` 같은 줄이
  "conf라는 설정값을 만들고 기본값은 0.3"이라는 뜻. 실행할 때 밖에서 바꿀 수 있다.
- **런치파일(launch file)** = 노드 여러 개를 설정값과 함께 한 번에 켜는 시작 스크립트 (Python으로 씀).
- **QoS** = 통신 품질 옵션. "유실돼도 빨리(BEST_EFFORT)" vs "느려도 확실히(RELIABLE)".
  발행자와 구독자의 QoS가 안 맞으면 **에러도 없이 조용히 데이터가 안 온다.** 자주 겪는 함정.
- **DDS** = 토픽을 실제로 실어 나르는 밑바닥 통신 계층. 여러 회사 구현체가 있는데,
  **서로 다른 구현체끼리는 방언이 달라서 특히 서비스 호출이 말없이 증발한다.**
  그래서 이 프로젝트는 모든 노드를 CycloneDDS로 통일한다
  (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `scripts/cyclone_localhost.xml`). 반드시 지킬 것.

터미널에서 직접 확인해 보기 (시스템이 켜져 있을 때):

```bash
ros2 node list                        # 지금 살아있는 노드(프로그램) 목록
ros2 topic list                       # 지금 존재하는 방송 채널 목록
ros2 topic echo /monitor/scores       # 점수 방송을 실시간으로 훔쳐 듣기
ros2 topic hz /lidar_points           # 라이다가 초당 몇 번 방송하는지 (죽었는지 확인)
ros2 param get /hesai_zed_fusion conf # 노드의 설정값 읽기
```

---

## 3. 5분 BehaviorTree 입문

BehaviorTree(행동트리)는 "무엇을 어떤 순서로 할지"를 트리 모양으로 그린 것이다.
게임 AI에서 온 개념인데, cdr_monitor가 센서 검사 순서를 이걸로 짠다.

- **tick(똑딱)**: 루트 노드를 주기적으로 "똑딱" 하고 두드리면, 신호가 트리를 타고 내려가
  자식 노드들이 차례로 실행된다. cdr_monitor는 이 tick을 일정 주기로 반복한다.
  즉 "매 주기마다 검사 목록을 위에서 아래로 훑는 while문"을 트리로 그린 것.
- **SyncActionNode**: "tick을 받으면 일을 즉시 끝내고 성공/실패를 돌려주는" 가장 단순한
  노드 타입. 우리 검사 노드들(lidar_check 등)이 전부 이 타입이다.
- **blackboard(블랙보드)**: 트리 전체가 공유하는 화이트보드(키-값 저장소).
  lidar 검사 노드가 `lidar_quality_score = 0.7`을 적어두면,
  나중에 실행되는 PublishScores 노드가 그 값을 읽어서 합산한다.
  노드끼리 직접 대화하지 않고 이 보드를 통해서만 값을 주고받는다.
- **XML로 구조 정의**: 트리의 모양(누가 누구의 자식인지, 실행 순서)은 C++ 코드가 아니라
  `ros/cdr_monitor/behavior_tree/monitor.xml`에 적혀 있다.
  C++ 파일들은 "노드 부품"을 만들고, XML이 부품을 조립한다.
  검사 순서를 바꾸고 싶으면 컴파일 없이 XML만 고치면 된다. 느낌만 보면:

```xml
<Sequence>                          <!-- 자식들을 순서대로 실행 (하나라도 실패하면 중단) -->
    <LidarCheck .../>               <!-- 라이다 신선도 검사 → 블랙보드에 점수 기록 -->
    <LidarQualityCheck .../>        <!-- 라이다 품질 검사 → 블랙보드에 점수 기록 -->
    <PublishScores .../>            <!-- 블랙보드의 점수를 모아 토픽으로 발행 -->
</Sequence>
```

  태그 이름(`LidarCheck` 등)이 곧 C++ 클래스 이름이다. XML의 한 줄 = 부품 하나.

---

## 4. 5분 C++ 관용구 (코드에서 마주칠 것들만)

- **포인터 vs 참조**: `T* p`는 주소를 담는 변수(화살표), `T& r`은 별명이다.
  `p->x`는 "주소를 따라가서 x를 봐라", `r.x`는 그냥 `r`이 가리키는 원본의 x.
- **`std::shared_ptr<T>`**: 자동 관리 포인터. 몇 명이 쓰고 있는지 세다가
  아무도 안 쓰면 스스로 메모리를 해제한다. `make_shared<T>(...)`로 만든다.
  Python처럼 "해제 걱정 안 해도 되는 포인터"라고 보면 된다.
- **`std::weak_ptr<T>`와 `.lock()`**: shared_ptr을 "세지 않고" 지켜보는 포인터.
  쓰고 싶을 때 `.lock()`을 부르면 → 대상이 아직 살아있으면 shared_ptr을 주고,
  이미 죽었으면 빈 포인터를 준다. "아직 살아있니?" 확인 후 쓰는 안전장치.
- **람다 `[this](){ ... }`**: 이름 없는 즉석 함수. 대괄호는 "바깥에서 뭘 가져올지" 목록이고,
  `[this]`는 "지금 이 객체의 멤버 변수/함수를 안에서 쓰겠다"는 뜻.
  콜백 등록할 때 자주 보인다 (`std::bind(&클래스::함수, this, ...)`도 같은 목적).
- **`mutex`와 `lock_guard`**: 스레드 여러 개가 같은 변수를 동시에 만지면 데이터가 깨진다.
  mutex는 자물쇠, `std::lock_guard<std::mutex> lk(m);`은 "이 중괄호 블록 동안 잠그고,
  블록을 나가면 자동으로 풀어라". ROS 콜백은 다른 스레드에서 올 수 있어서 필요하다.
- **템플릿 꺾쇠 `<>`**: `vector<double>`은 "double을 담는 vector"라는 뜻.
  Python의 `List[float]` 타입 힌트와 같은 역할인데, C++에선 강제된다.
- **`auto`**: "타입은 컴파일러가 알아서 적어라". `auto msg = ...`처럼 긴 타입 이름을 줄일 때 쓴다.
- **헤더(.hpp) vs 소스(.cpp)**: C++은 파일이 둘로 나뉜다. 헤더는 "이런 클래스/함수가 있다"는
  선언(차례), 소스는 실제 구현(본문). 우리 검사 노드들은 짧아서 헤더 안에 구현까지 다 넣었다.
  그래서 `include/cdr_monitor/`의 .hpp 파일만 읽어도 로직 전체가 보인다.
- **콜백(callback)**: "이 채널에 방송이 오면 이 함수를 불러줘"라고 등록해 두는 함수.
  내가 함수를 부르는 게 아니라 ROS가 대신 불러준다. 코드의 실행 순서가 위→아래가 아니라
  "데이터가 도착한 순서"라는 점이 처음엔 헷갈리는 부분.

---

## 5. 코드 읽는 순서 추천

이 순서대로 읽으면 "점수가 만들어지고 → 판단되고 → 실험되는" 흐름을 그대로 따라가게 된다.

1. `ros/cdr_monitor/include/cdr_monitor/lidar_quality_check.hpp`
   — 검사 노드 하나의 전형. 이거 하나 이해하면 나머지 `*_check.hpp`는 다 같은 패턴이다.
2. `ros/cdr_monitor/src/calculate_ftti.cpp`
   — "센서가 얼마나 오래 죽어도 되는가(FTTI)"를 속도/TTC로 동적으로 계산하는 논문의 핵심 수식.
3. `ros/cdr_monitor/include/cdr_monitor/publish_scores.hpp`
   — 개별 점수들을 센서→섹터→능력 3층으로 합산해 /monitor/scores와 /diagnostics로 내보내는 출구.
4. `ros/mrm_supervisor/mrm_supervisor/supervisor.py`
   — 점수를 받아 NORMAL / MRM_MOVE / MRM_DEFER를 오가는 상태기계. 판단 로직의 전부가 여기 있다.
5. `ros/mrm_supervisor/mrm_supervisor/map_model.py`
   — 지도(lanelet2)에서 갓길/주차장 후보를 뽑고 어디로 피할지 비용을 계산하는 지도 담당.
6. `agent/aw_e2e.py`
   — CARLA와 Autoware를 잇는 에이전트 + 고장(노이즈/센서 차단)을 일부러 주입하는 실험 도구.
7. `experiments/experiment_runner.py`
   — 위 전부를 자동으로 수백 번 돌리고 결과를 판정해 CSV로 쌓는 실험 총감독.

---

## 6. 프로젝트 용어 사전

| 용어 | 뜻 |
|---|---|
| **FTTI** | Fault Tolerant Time Interval. 고장이 나고 "이 시간 안에만 대처하면 안전"한 한계 시간. 이 프로젝트에선 고정값이 아니라 속도/장애물 거리에 따라 실시간으로 늘었다 줄었다 함 |
| **TTC** | Time To Collision. 지금 속도로 가면 앞 장애물과 몇 초 뒤에 부딪히는가 |
| **MRM** | Minimum Risk Maneuver. 최소 위험 기동. "더는 정상 주행이 어려우니 안전하게 세우는 동작" 전반 |
| **MRM_MOVE** | 우리 방식의 MRM: 점수가 애매하게 나쁠 때, 지도에서 갓길/주차장을 골라 그리로 우회해 정차 |
| **DEFER (MRM_DEFER)** | 점수가 너무 나쁠 때 "우리가 판단 안 하고 순정 Autoware의 비상정지에 맡김(위임)". 되돌릴 수 없는 종착 상태 |
| **band (밴드)** | 점수 0.3 < s ≤ 0.7 구간. "나쁘긴 한데 아직 움직일 수는 있다" → MRM_MOVE 대상 |
| **floor (플로어)** | 점수 ≤ 0.3인 바닥 구간. "이 눈으로는 못 달린다" → DEFER 대상. 한 번의 튐이 아니라 연속 5틱 이상이어야 발동 |
| **health** | min(perception 점수, localization 점수). 시스템 전체 건강도는 가장 아픈 쪽이 결정한다는 뜻 |
| **armed (무장)** | 부팅 직후엔 점수가 요동치므로, "3초간 건강한 점수가 유지된 뒤에야" 감시를 실전 가동하는 안전핀 |
| **섹터(sector)** | 차 주변을 8방향(전, 전좌, 좌, ...)으로 나눈 부채꼴. 센서 점수를 "어느 방향을 볼 수 있는가"로 합칠 때 씀 |
| **quality score** | 데이터가 오긴 오는데 내용이 이상한 정도 (노이즈 증가, 포인트 수 감소 등). 0~1 |
| **timeout score** | 데이터가 제때 오는가. FTTI 한계 대비 얼마나 늦었는지로 계산. 0~1. 최종 센서 점수 = timeout x quality |
| **blacklist** | MRM_MOVE로 보낸 갓길/주차 후보에 주차를 실패했을 때 그 후보를 다시 안 고르도록 적어두는 명단 |
| **diagnostics** | ROS 표준 고장 알림 토픽(/diagnostics). Autoware의 진단 그래프가 이걸 읽고 자체 MRM을 발동함 |
| **lanelet2** | Autoware가 쓰는 HD 지도 포맷. 차선 하나하나가 polygon으로 정의됨. map_model.py가 파싱 |
| **고장 주입 (fault injection)** | 실험을 위해 일부러 센서를 망가뜨리는 것. ln=라이다 노이즈, cn=카메라 노이즈, gn=GPS 노이즈, cf/lf=카메라/라이다 완전 차단, nf=무고장 대조군 |
| **ego (에고)** | 자율주행 차 자신. "ego vehicle" = 우리가 조종하는 그 차 |
| **리더보드 (leaderboard)** | CARLA 공식 평가 프레임워크. 정해진 경로/시나리오로 에이전트를 돌리고 점수를 매김. 우리 실험의 실행 틀 |

---

## 다음 단계

- 전체 설계와 실험 매트릭스는 루트의 `README.md` 참고.
- 실차 포팅 쪽이 궁금하면 `ros/hesai_zed_fusion/`의 두 노드(한글 주석 포함)를 열어볼 것.
- 실행해 보고 싶으면: `scripts/run_leaderboard.sh` → `python3 experiments/experiment_runner.py --smoke`.
