/**
 * [calculate_ftti.cpp] "동적 FTTI" 계산 노드 — 이 시스템의 핵심 아이디어.
 *
 * FTTI(Fault Tolerant Time Interval) = 센서 고장을 견딜 수 있는 최대 시간.
 * 즉 "이 시간 안에 새 데이터가 안 오면 고장으로 판정한다"는 데드라인이다.
 *
 * 데이터 흐름:
 *   블랙보드에서 속도/장애물 거리/외부 TTC/중요 센서 목록을 읽고
 *   → TTC(충돌까지 남은 시간) 계산
 *   → 동적 FTTI = min(850ms, TTC): "위험이 가까울수록 데드라인을 조인다"
 *   → 센서별 FTTI("Hesai_AT128_ftti" 등)를 블랙보드에 써서
 *     다음 단계의 타임아웃 검사 노드(LidarChecker 등)가 쓰게 한다.
 *
 * 추가 안전장치(fail-conservative TTC hold):
 *   TTC를 재는 데 쓰인 센서 자신이 아플 때는, "커지는 TTC"를 믿지 않고
 *   마지막으로 신뢰했던 TTC 값을 최대 5초간 유지한다. (아래 tick() 참고)
 */
#include "cdr_monitor/calculate_ftti.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <string.h>
using namespace std;

// 속도 벡터 (vel_x, vel_y) 의 크기(속력)를 구한다. 피타고라스 정리.
double CalculateFtti::calculate_vel(double vel_x, double vel_y) {
    return sqrt((vel_x * vel_x) + (vel_y * vel_y));
}

// 단순(직선 가정) TTC 계산: 거리[m] / 속력[m/s] = 남은 시간[s] → *1000 해서 ms.
// 속력이 0이면 나눗셈이 불가능하므로 안전하게 1000ms(1초)를 돌려준다.
double CalculateFtti::calculate_ttc(double vel, double dist){
    if (vel != 0)
        return (dist/vel) * 1000;
    else
        return 1000.0;
}

// "a,b,c" 처럼 쉼표로 이어진 문자열을 ["a","b","c"] 목록으로 쪼개는 함수.
// (블랙보드에는 목록을 문자열 하나로 넘겨받기 때문에 직접 파싱한다)
vector<string> CalculateFtti::parsing_port(string raw){
    vector<string> result = {};
    size_t start = 0;                    // 지금 자르는 조각의 시작 위치
    size_t end = raw.find(',');          // 다음 쉼표 위치
    while (end != string::npos){         // npos = "못 찾음" 이라는 특수값
        result.push_back(raw.substr(start,end-start));   // 쉼표 앞까지 잘라 담고
        start = end+1;                   // 쉼표 다음 칸부터
        end = raw.find(',', start);      // 또 다음 쉼표를 찾는다
    }
    result.push_back(raw.substr(start)); // 마지막 조각(쉼표 뒤 남은 부분)도 담기
    return result;
}

// 생성자. SyncActionNode = "tick 한 번에 일을 끝내는 동기식 행동트리 노드" 베이스 클래스.
// : SyncActionNode(name,config) 는 부모 클래스 생성자에 이름/설정을 그대로 전달한다는 뜻.
CalculateFtti::CalculateFtti(const string& name, const NodeConfig& config)
    : SyncActionNode(name,config)
{}

// 이 노드가 블랙보드와 주고받는 "포트" 목록 선언.
// InputPort = 블랙보드에서 읽어오는 입력 칸. (XML에서 {@vel_x} 식으로 연결됨)
PortsList CalculateFtti::providedPorts() {
    return {
        InputPort<vector<string>>("lidar_names"),      // 라이다 센서 이름 목록
        InputPort<vector<string>>("camera_names"),     // 카메라 센서 이름 목록
        InputPort<double>("vel_x"),                    // 차 속도 x [m/s]
        InputPort<double>("vel_y"),                    // 차 속도 y [m/s]
        InputPort<double>("nearest_obstacle"),         // 가장 가까운 장애물 거리 [m]
        InputPort<double>("ttc_ext_ms"),               // 외부(경로 기반) TTC [ms]
        InputPort<string>("important_sensors"),        // 중요 센서 목록 (쉼표 문자열)
        InputPort<string>("ttc_source_sensors"),       // TTC의 근거가 된 센서 목록 (쉼표 문자열)
    };
}

// tick() = 행동트리가 이 노드 차례가 될 때마다(10Hz) 호출하는 본체 함수.
NodeStatus CalculateFtti::tick() {
    // 입력값 담을 변수들 (getInput이 실패해도 안전하도록 기본값을 미리 준다)
    double vel_x = 0.0;
    double vel_y = 0.0;
    double nearest_obstacle = 1000.0;
    string important_sensors = "";
    vector<string> lidar_names;
    vector<string> camera_names;

    // 블랙보드에서 속도를 읽어와 속력 계산
    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    double vel = calculate_vel(vel_x, vel_y);

    // "지금 위험에 중요한 센서" 목록 (UpdateSystemState가 적어둔 값)
    getInput("important_sensors", important_sensors);

    // 가장 가까운 장애물까지의 거리
    getInput("nearest_obstacle", nearest_obstacle);

    // 거리가 0 이하 = 말이 안 되는 값(데이터 없음 등) → 100m(멀다)로 취급
    if (nearest_obstacle <= 0.0){
        nearest_obstacle = 100.0;
    }

    // 1차 TTC: "지금 속력으로 직진하면 장애물까지 몇 ms 걸리나"
    double ttc = calculate_ttc(vel, nearest_obstacle);

    // 2차 TTC(외부): 계획된 주행 경로 x 물체들의 예측 경로를 시공간에서 겹쳐 본 TTC.
    // 직선 가정으로는 놓치는 "교차로에서 옆에서 오는 차, 마주 오는 차" 상황을 잡는다.
    // 두 TTC 중 더 작은(더 위험한) 쪽을 채택 → 항상 보수적으로.
    double ttc_ext = -1.0;
    getInput("ttc_ext_ms", ttc_ext);
    if (ttc_ext > 0.0 && ttc_ext < ttc)
        ttc = ttc_ext;

    // 쉼표 문자열로 온 중요 센서 목록을 진짜 목록으로 변환
    vector<string> important_sensors_names;
    if (!important_sensors.empty())
        important_sensors_names = parsing_port(important_sensors);

    // 디버그 출력 (터미널에서 상태 확인용)
    cout << "dist " << nearest_obstacle << "m" << endl;
    cout << "ttc " << ttc << "ms (ext " << ttc_ext << "ms)" << endl;

    // 센서 이름 → 그 센서에 줄 FTTI[ms] 를 담을 표
    map<string,double> ftti_map;

    getInput("lidar_names", lidar_names);
    getInput("camera_names", camera_names);

    // 블랙보드를 직접 만질 수 있는 핸들 (포트로 선언 안 한 키도 읽고 쓸 수 있음)
    auto bb = config().blackboard;

    // ── fail-conservative TTC hold ──
    // 장애물 목록은 인지(perception) 파이프라인에서 나오는데, 그 근원 센서가
    // 아프면 "장애물을 못 봐서" TTC가 커 보일 수 있다. 즉 TTC가 커지는 게
    // 실제로 안전해진 게 아니라 탐지를 놓친 것일 수 있다.
    // 그래서: 근원 센서가 아플 땐 커지는 TTC를 믿지 않고, 마지막으로 신뢰했던
    // 값을 유지한다. 현재 측정값이 더 "작을"(더 위험할) 때만 받아들인다.
    // 근원 센서를 따로 지정하지 않으면 라이다 전체로 본다
    // (AWCL의 물체 인식이 라이다 파이프라인 기반이라서).
    string ttc_sources = "";
    getInput("ttc_source_sensors", ttc_sources);
    vector<string> source_names =
        ttc_sources.empty() ? lidar_names : parsing_port(ttc_sources);
    // 근원 센서들이 전부 건강한지 확인
    bool source_healthy = true;
    for (const auto& n : source_names) {
        bool failed = false;
        double q = 1.0;
        // 다른 검사 노드들이 블랙보드에 적어둔 고장 여부/품질 점수를 읽는다.
        // (void) 는 "반환값(성공 여부)은 일부러 안 쓴다"는 표시 — 키가 없으면 기본값 유지.
        (void)bb->get(n + "_failed", failed);
        (void)bb->get(n + "_quality_score", q);
        // 완전 고장이거나 품질 점수가 0.7 미만이면 → 근원이 아픈 것
        if (failed || q < 0.7) { source_healthy = false; break; }
    }
    // hold에는 5초 시간제한이 있다: 순간적인 탐지 끊김을 메꾸는 게 목적이고,
    // 지속적인 고장은 이미 품질 점수 쪽에서 벌점을 받고 있다. 얼어붙은 작은 TTC를
    // 영원히 붙잡고 있으면 이중 처벌이 되어 점수가 DEFER 하한선 아래로 무너지고,
    // 단계별 대응 설계가 깨진다(갓길 정차 단계가 실행될 기회가 없어짐).
    if (source_healthy) {
        // 건강함: 지금 TTC를 "신뢰값"으로 갱신하고 hold 상태 해제
        trusted_ttc_ = ttc;
        was_unhealthy_ = false;
    } else {
        // 방금 아프기 시작했다면 아픈 시각을 기록 (5초 타이머 시작점)
        if (!was_unhealthy_) {
            was_unhealthy_ = true;
            unhealthy_since_ = std::chrono::steady_clock::now();
        }
        // 아픈 지 몇 초 됐는지 계산 (steady_clock = 뒤로 안 가는 스톱워치 시계)
        const double held_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - unhealthy_since_).count();
        // 5초 이내이고, 신뢰값이 측정값보다 작다면(측정 TTC가 커졌다면) → 신뢰값 유지
        if (held_s < 5.0 && trusted_ttc_ < ttc) {
            cout << "ttc source unhealthy: hold " << trusted_ttc_
                 << "ms (measured " << ttc << "ms, " << held_s << "s)" << endl;
            ttc = trusted_ttc_;
        }
    }

    // ── 센서별 FTTI 배정 ──
    // 기본값: 모든 라이다/카메라는 850ms (평상시 데드라인)
    for (const auto& name : lidar_names) {
        ftti_map[name] = 850.0;
    }
    for (const auto& name : camera_names) {
        ftti_map[name] = 850.0;
    }
    // 핵심 규칙: "중요 센서"만, TTC가 850ms보다 작을 때, FTTI = TTC 로 조인다.
    // 즉 동적 FTTI = min(850ms, TTC): 충돌이 가까울수록 그 위험을 보는 센서에게
    // 더 빡빡한 데드라인을 요구한다.
    for (const auto& name : important_sensors_names) {
        if (ttc < 850.0)
            ftti_map[name] = ttc;
    }

    // 계산된 FTTI를 블랙보드에 기록 (예: "Hesai_AT128_ftti" = 620.0).
    // 다음 단계의 LidarChecker 등이 이 값을 데드라인으로 사용한다.
    // auto& [name,value] 는 "구조 분해": 표의 (키,값) 쌍을 두 변수로 바로 받는 문법.
    for (auto& [name,value] : ftti_map){
        bb->set(name + "_ftti", value);
        RCLCPP_INFO(rclcpp::get_logger("ftti_names"), "%s : %.2f", name.c_str(), value);   // ROS 로그 출력
    }

    // SUCCESS 반환 → 행동트리가 다음 형제 노드(SensorChecks)로 진행
    return NodeStatus::SUCCESS;
}
