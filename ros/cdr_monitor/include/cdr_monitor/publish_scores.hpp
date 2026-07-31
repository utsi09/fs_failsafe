// ============================================================================
// [이 파일의 역할] — 감시 파이프라인의 "출구"
// 전체 시스템 흐름: 각 센서 검사 노드(타임아웃 검사 *_check + 품질 검사
// *_quality_check)가 블랙보드(트리 공유 칠판)에 점수를 적어두면,
// 이 노드가 매 tick 그 점수들을 모아 3층 모델로 합산한다:
//   1층: 센서별 점수 = timeout_score x quality_score
//   2층: 8방향 섹터 커버리지 (여러 라이다의 겹침을 확률적으로 융합)
//   3층: 능력(capability) 점수 — 신호등 인식 / 장애물 감지 / 자기위치
// 그리고 두 토픽으로 발행한다:
//   /monitor/scores : 전체 점수 세트 → MRM 슈퍼바이저가 구독해 대응 판단
//   /diagnostics    : 센서별 상태 → Autoware의 diagnostic_graph_aggregator로 전달
// ============================================================================
#pragma once                                   // 헤더 중복 include 방지
#include "behaviortree_ros2/bt_topic_sub_node.hpp"   // BT + ROS2 연동 기반 클래스
#include <diagnostic_msgs/msg/diagnostic_array.hpp>  // 진단 메시지 타입 (이름+레벨+키값 목록)
#include <algorithm>                           // std::max, std::clamp 등
#include <chrono>                              // 시계/시간 라이브러리 (워밍업 타이머용)
#include <cmath>                               // 수학 함수
#include <cstdint>                             // uint8_t, int64_t 같은 고정 크기 정수
#include <map>                                 // 키-값 트리 맵 (래치 저장용)
#include <sstream>                             // 숫자→문자열 변환용 문자열 스트림
#include <vector>                              // 동적 배열
using namespace std;                           // std:: 접두사 생략

// [합산 규칙 요약]
// sensor_score = timeout_score(FTTI 검사 노드가 기록) * quality_score(품질 검사 노드가 기록).
// 섹터 융합: 1 - prod(1 - s_i) — "그 방향을 덮는 라이다들이 전부 놓칠 확률"의 여집합.
//   예: 점수 0.9 라이다 2개가 겹치면 1 - 0.1*0.1 = 0.99 (중복 센서일수록 든든함)
// SyncActionNode = "행동트리가 '똑딱(tick)' 부르면 한 번 실행되는 작업 단위"
class PublishScores : public BT::SyncActionNode
{
public:
    // 8개 방향 섹터 라벨: 앞(F)부터 반시계 방향으로 45도씩.
    // F=앞, FL=앞왼쪽, L=왼쪽, RL=뒤왼쪽, B=뒤, RR=뒤오른쪽, R=오른쪽, FR=앞오른쪽
    // constexpr = 컴파일 시점에 확정되는 상수
    static constexpr const char* SECTOR_LABELS[8] =
        {"F", "FL", "L", "RL", "B", "RR", "R", "FR"};  // 각 45도, +x축(차량 앞)부터 반시계

    // 센서 1개의 등록 정보 (설정 파일에서 읽어옴)
    struct SensorEntry {
        std::string name;                      // 센서 이름 (블랙보드 키의 접두어로 쓰임)
        std::string type;      // 종류: "lidar" 또는 "camera"
        double yaw_deg = 0.0;                  // 센서가 향한 방향 (차량 앞 기준 각도)
        double fov_deg = 360.0;                // 수평 시야각 (360이면 전방위)
    };

    // 생성자: 트리 구성 시 한 번 호출. ROS 파라미터에서 센서 목록을 읽고 발행자를 만든다
    PublishScores(const std::string& name, const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
                  : SyncActionNode(name, conf) // 부모(BT 노드) 초기화
    {
        // weak_ptr(약한 참조)를 shared_ptr(진짜 소유권)로 승격. 노드가 죽었으면 nullptr
        node_ = params.nh.lock();
        // "lidar", "camera" 두 종류에 대해 설정 파일의 센서 목록을 순회
        for (const std::string& type : {std::string("lidar"), std::string("camera")}) {
            std::vector<std::string> names;
            // ROS 파라미터(설정 파일 값)에서 센서 이름 목록을 읽음. 예: sensors.lidar.names
            node_->get_parameter("sensors." + type + ".names", names);
            for (const auto& sensor_name : names) {   // 센서 하나씩 등록
                const std::string prefix = "sensors." + type + "." + sensor_name + ".";
                SensorEntry e;
                e.name = sensor_name;
                e.type = type;
                // 파라미터가 아직 선언 안 됐으면 기본값 0.0으로 선언부터 함 (ROS2 규칙:
                // declare 없이 get하면 값을 못 받음)
                if (!node_->has_parameter(prefix + "yaw_deg"))
                    node_->declare_parameter<double>(prefix + "yaw_deg", 0.0);
                node_->get_parameter(prefix + "yaw_deg", e.yaw_deg);     // 센서 장착 방향
                node_->get_parameter(prefix + "h_fov_deg", e.fov_deg);   // 센서 시야각
                sensors_.push_back(e);         // 목록에 추가
            }
        }
        // create_publisher = "이 토픽(방송 채널)으로 메시지를 내보낼 송신기"를 만듦. 10 = 대기열 크기
        scores_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/monitor/scores", 10);            // MRM 슈퍼바이저가 구독하는 전체 점수 토픽
        diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);               // Autoware 진단 체계로 들어가는 표준 토픽
        // 래치(상태 유지) 관련 파라미터들: 선언 후 읽기
        if (!node_->has_parameter("diag_recovery_hold_s"))
            node_->declare_parameter<double>("diag_recovery_hold_s", 3.0);
        node_->get_parameter("diag_recovery_hold_s", recovery_hold_s_);      // 회복 인정까지 버텨야 하는 시간
        if (!node_->has_parameter("diag_recovery_margin"))
            node_->declare_parameter<double>("diag_recovery_margin", 0.2);
        node_->get_parameter("diag_recovery_margin", recovery_margin_);      // 회복 인정에 필요한 여유 점수
        if (!node_->has_parameter("warmup_timeout_s"))
            node_->declare_parameter<double>("warmup_timeout_s", 120.0);
        node_->get_parameter("warmup_timeout_s", warmup_timeout_s_);         // 워밍업 최대 대기 시간
        warmup_start_ = std::chrono::steady_clock::now();   // 워밍업 시작 시각 기록 (steady_clock = 되돌아가지 않는 시계)
    }

    // 이 노드는 XML 입력 구멍이 없음 (모든 입력을 블랙보드에서 직접 읽음)
    static BT::PortsList providedPorts() { return {}; }

    // tick() = 트리가 주기적으로 부르는 함수. 점수 수집 → 합산 → 발행을 한 번에 수행
    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;         // blackboard = 트리 노드들이 공유하는 칠판(키-값 저장소)
        // get_d = "칠판에서 double 값을 읽되 없으면 기본값" 하는 람다(즉석 함수).
        // [&] = 바깥 변수(bb)를 참조로 빌려 씀. (void)는 "반환값 무시" 표시
        auto get_d = [&](const std::string& key, double def) {
            double v = def;
            (void)bb->get(key, v);
            return v;
        };

        // --- 1층: 센서별 점수 ---
        std::vector<double> scores(sensors_.size());                    // 최종 센서 점수
        std::vector<double> tscores(sensors_.size()), qscores(sensors_.size());   // 타임아웃/품질 점수
        for (size_t i = 0; i < sensors_.size(); i++) {
            // FTTI 검사 노드가 적어둔 타임아웃 점수 (없으면 1.0 = 문제 없음 가정)
            tscores[i] = get_d(sensors_[i].name + "_timeout_score", 1.0);
            // 품질 검사 노드가 적어둔 품질 점수
            qscores[i] = get_d(sensors_[i].name + "_quality_score", 1.0);
            // 센서 점수 = 두 점수의 곱 (하나만 나빠도 깎임), 0~1로 자름
            scores[i] = std::clamp(tscores[i] * qscores[i], 0.0, 1.0);
        }

        // --- 2층: 라이다들의 8방향 섹터 커버리지 (중복 센서 융합) ---
        double sectors[8];
        for (int k = 0; k < 8; k++) {          // 각 45도 섹터마다
            double miss = 1.0;                 // "이 방향을 전부 놓칠 확률"에서 시작
            for (size_t i = 0; i < sensors_.size(); i++) {
                if (sensors_[i].type != "lidar") continue;   // 섹터 커버리지는 라이다만 사용
                // 이 라이다가 k번째 섹터(중심각 k*45도)를 덮으면, 놓칠 확률에 (1-점수)를 곱함
                if (covers(sensors_[i], k * 45.0)) miss *= (1.0 - scores[i]);
            }
            // 섹터 점수 = 1 - (전부 놓칠 확률). 덮는 라이다가 없으면 miss=1 → 0점
            sectors[k] = 1.0 - miss;
        }

        // --- 3층: 능력(capability) 점수 ---
        double cap_tl = 0.0;   // 신호등 인식 능력: 전방을 보는 카메라만 기여
        bool has_cam = false;
        for (size_t i = 0; i < sensors_.size(); i++) {
            // 카메라이면서 전방(0도)을 덮는 센서 중 최고 점수를 채택
            if (sensors_[i].type == "camera" && covers(sensors_[i], 0.0)) {
                cap_tl = std::max(cap_tl, scores[i]);
                has_cam = true;
            }
        }
        if (!has_cam) cap_tl = 0.0;            // 전방 카메라가 아예 없으면 신호등 인식 불가 → 0점

        // 장애물 감지 능력: 섹터 점수의 가중 평균. 주행 중엔 앞쪽(F, FL, FR)이 제일
        // 중요하므로 전방에 큰 가중치. W 합 = 1.0 (F 0.30 + FL/FR 각 0.15 + ...)
        static const double W[8] = {0.30, 0.15, 0.10, 0.05, 0.10, 0.05, 0.10, 0.15};
        double cap_obs = 0.0;
        for (int k = 0; k < 8; k++) cap_obs += W[k] * sectors[k];   // 내적: sum(W_k * sector_k)

        // 자기위치(localization) 능력: IMU와 GPS 검사 노드가 칠판에 적어둔 고장 여부 사용
        bool imu_failed = false, gps_failed = false;
        (void)bb->get("IMU_failed", imu_failed);
        (void)bb->get("GPS_failed", gps_failed);
        // 둘 중 하나라도 고장이면 0점 (0 또는 1의 곱)
        const double cap_loc = (imu_failed ? 0.0 : 1.0) * (gps_failed ? 0.0 : 1.0);

        // 종합 인지(perception) 점수 = 장애물 60% + 신호등 40% 가중합
        const double perception = 0.6 * cap_obs + 0.4 * cap_tl;

        // --- /monitor/scores 발행 (모든 층의 점수를 하나의 배열 메시지로) ---
        diagnostic_msgs::msg::DiagnosticArray out;
        out.header.stamp = node_->now();       // 메시지에 현재 시각 도장
        for (size_t i = 0; i < sensors_.size(); i++) {
            // 센서별 상태 항목 추가 (부가 정보로 타임아웃/품질 점수도 첨부)
            out.status.push_back(make_status(
                "sensor/" + sensors_[i].name, scores[i],
                {{"timeout_score", tscores[i]}, {"quality_score", qscores[i]}}));
        }
        for (int k = 0; k < 8; k++)            // 섹터 8개 점수 추가
            out.status.push_back(make_status(std::string("sector/") + SECTOR_LABELS[k],
                                             sectors[k], {}));
        // 능력 점수 4개 추가
        out.status.push_back(make_status("capability/traffic_light", cap_tl, {}));
        out.status.push_back(make_status("capability/obstacle", cap_obs, {}));
        out.status.push_back(make_status("capability/localization", cap_loc, {}));
        out.status.push_back(make_status("capability/perception", perception, {}));
        scores_pub_->publish(out);             // 실제 발행 (MRM 슈퍼바이저가 받음)

        // [워밍업 처리] 시뮬레이션 시작 직후엔 리더보드가 센서를 아직 스폰하는 중이라
        // "고장처럼 보이는" 상태가 정상이다. 모든 센서가 첫 메시지를 보낼 때까지는
        // Autoware 쪽에 OK로 보고해서 자율주행 시작(engage)이 막히거나 출렁이지 않게 하고,
        // 회복 래치 로직에도 이 구간을 포함시키지 않는다.
        if (!warmup_done_) {
            bool all_seen = true;              // 모든 센서가 한 번 이상 수신됐는가
            for (const auto& s : sensors_) {
                bool failed = true;
                // 칠판에 기록이 없으면(아직 검사 노드가 안 돌았으면) 실패로 간주
                if (!bb->get(s.name + "_failed", failed)) failed = true;
                if (failed) { all_seen = false; break; }   // 하나라도 실패면 워밍업 계속
            }
            // 워밍업 시작 후 경과 시간(초)
            const double waited = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - warmup_start_).count();
            // 전부 수신됐거나 최대 대기 시간을 넘기면 워밍업 종료
            if (all_seen || waited > warmup_timeout_s_) {
                warmup_done_ = true;
                cout << "[Scores] warmup done (" << waited << "s, all_seen="
                     << all_seen << ")" << endl;
            }
        }

        // --- /diagnostics 발행 (Autoware의 diagnostic_graph_aggregator가 소비) ---
        // 레벨은 히스테리시스(이력 효과) 래치로 고정:
        //   나빠지는 방향은 즉시 반영, 좋아지는 방향은 "임계값+여유를 recovery_hold_s초
        //   동안 유지"해야 인정. 이게 없으면 점수가 톱니처럼 오르내릴 때(정지 → TTC 완화
        //   → 재출발 반복 루프) ERROR<->OK가 깜빡거려서 MRM이 완료 전에 계속 취소됨.
        const int64_t now_ns = rclcpp::Time(out.header.stamp).nanoseconds();   // 현재 시각(ns)
        diagnostic_msgs::msg::DiagnosticArray diag;
        diag.header.stamp = out.header.stamp;
        for (size_t i = 0; i < sensors_.size(); i++) {
            // Autoware 진단 규칙에 맞는 이름 형식: "노드이름: 경로"
            auto st = make_status("cdr_monitor: sensing/ftti/" + sensors_[i].name,
                                  scores[i],
                                  {{"timeout_score", tscores[i]},
                                   {"quality_score", qscores[i]}});
            st.hardware_id = "cdr_monitor";    // 어느 하드웨어/노드의 진단인지 표시
            if (warmup_done_) apply_latch(st, scores[i], now_ns);   // 래치 적용 (깜빡임 방지)
            else { st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;   // 워밍업 중엔 무조건 OK
                   st.message = "warmup"; }
            diag.status.push_back(st);
        }
        // 종합 인지 점수도 하나의 진단 항목으로 발행
        auto agg = make_status("cdr_monitor: sensing/perception_score", perception, {});
        agg.hardware_id = "cdr_monitor";
        // [중요] Autoware 진단 그래프에 연결된 이 유닛은 반드시 2단계(OK/ERROR)여야 함:
        // Autoware의 operation-mode availability는 "OK가 아니면 사용 불가"로 취급하므로
        // (level()==OK 검사), 여기서 WARN을 내면 슈퍼바이저가 직접 다뤄야 할 중간 점수
        // 구간에서 Autoware 기본 MRM이 먼저 발동해버린다. 그래서 바닥값 위면 OK, 아래면 ERROR
        agg.level = perception > 0.3 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                     : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        if (warmup_done_) apply_latch(agg, perception, now_ns, true);   // binary=true 래치
        else { agg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;  // 워밍업 중엔 OK
               agg.message = "warmup"; }
        diag.status.push_back(agg);
        diag_pub_->publish(diag);              // 실제 발행 (Autoware 진단 체계로)

        // 10tick마다 한 번씩만 요약을 콘솔에 출력 (로그 홍수 방지)
        if (++tick_count_ % 10 == 1)
            cout << "[Scores] perception " << perception << " | tl " << cap_tl
                 << " obs " << cap_obs << " loc " << cap_loc << endl;
        return BT::NodeStatus::SUCCESS;        // 이 노드는 항상 SUCCESS
    }

private:
    // 센서 s가 특정 방향(섹터 중심각)을 시야에 담는지 판정하는 도우미
    static bool covers(const SensorEntry& s, double sector_center_deg)
    {
        if (s.fov_deg >= 360.0) return true;   // 전방위 센서는 모든 방향을 덮음
        // 센서 방향과 섹터 중심의 각도 차이를 0~180도로 정규화
        // (fmod = 실수 나머지 연산. 예: 350도 차이 → 실제로는 10도 차이)
        double d = std::fmod(std::fabs(sector_center_deg - s.yaw_deg), 360.0);
        if (d > 180.0) d = 360.0 - d;          // 둘레 반대편으로 재는 게 더 가까우면 그쪽으로
        // 시야 반각 + 섹터 반폭(22.5도) 이내면 "이 섹터에 걸친다"고 판정
        return d <= s.fov_deg / 2.0 + 22.5;   // 섹터 반폭만큼 겹침 허용
    }

    // 점수 하나를 DiagnosticStatus 메시지로 포장하는 도우미.
    // initializer_list = {{"키",값},...} 형태의 임시 목록을 받는 타입
    diagnostic_msgs::msg::DiagnosticStatus make_status(
        const std::string& name, double score,
        std::initializer_list<std::pair<const char*, double>> extras)
    {
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = name;
        // 점수 → 3단계 레벨 변환: 0.7 초과 OK / 0.3 초과 WARN / 그 이하 ERROR
        st.level = score > 0.7 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : score > 0.3 ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                               : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        // ostringstream = 숫자를 문자열로 바꾸는 도구 (진단 메시지 필드는 문자열이라서)
        std::ostringstream os;
        os.precision(3);                       // 유효숫자 3자리
        os << score;
        st.message = os.str();                 // 메시지 본문 = 점수 문자열
        diagnostic_msgs::msg::KeyValue kv;     // 진단 메시지의 부가 정보는 키-값 쌍 목록
        kv.key = "score";
        kv.value = st.message;
        st.values.push_back(kv);               // "score: 0.87" 형태로 첨부
        // 추가로 넘어온 부가 점수들(타임아웃/품질 등)도 키-값으로 첨부
        for (const auto& [k, v] : extras) {    // 구조 분해: pair를 k, v 두 변수로 풀어냄
            std::ostringstream vs;
            vs.precision(3);
            vs << v;
            kv.key = k;
            kv.value = vs.str();
            st.values.push_back(kv);
        }
        return st;
    }

    // 래치 1개의 상태: 현재 고정된 레벨 + 회복 타이머 시작 시각
    struct Latch {
        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;   // 현재 유지 중인 레벨
        int64_t better_since_ns = -1;   // 점수가 좋아지기 시작한 시각. -1 = 회복 타이머 미작동
    };

    // 히스테리시스 래치 적용: 악화는 즉시, 회복은 "여유 있게 + 오래" 확인 후에만 반영.
    // binary=true면 OK/ERROR 2단계만 사용 (그래프 연결 유닛용)
    void apply_latch(diagnostic_msgs::msg::DiagnosticStatus& st, double score,
                     int64_t now_ns, bool binary = false)
    {
        using DS = diagnostic_msgs::msg::DiagnosticStatus;   // 긴 타입 이름의 별칭
        auto& l = latches_[st.name];           // 이 진단 항목 전용 래치 (map: 없으면 자동 생성)
        const uint8_t raw = st.level;          // 래치 적용 전의 "생(raw)" 레벨
        // 참고: 레벨 숫자는 OK(0) < WARN(1) < ERROR(2), 클수록 나쁨
        if (raw >= l.level) {                    // 지금이 같거나 더 나쁨 → 즉시 따라감 (악화는 지체 없이)
            l.level = raw;
            l.better_since_ns = -1;            // 회복 타이머 리셋
        } else {
            // 좋아지는 방향: "임계값 + 여유(margin)"를 recovery_hold_s초 동안 유지해야 인정
            // rec = 여유를 반영해 다시 계산한 회복 목표 레벨
            const uint8_t rec = binary
                ? (score > 0.3 + recovery_margin_ ? DS::OK : DS::ERROR)      // 2단계 모드
                : score > 0.7 + recovery_margin_ ? DS::OK                    // 3단계 모드
                : score > 0.3 + recovery_margin_ ? DS::WARN
                                                 : DS::ERROR;
            if (rec >= l.level) {
                l.better_since_ns = -1;          // 여유 구간(히스테리시스 밴드) 안 → 회복으로 안 침, 타이머 리셋
            } else if (l.better_since_ns < 0) {
                l.better_since_ns = now_ns;    // 회복 조건 첫 충족 → 타이머 시작
            } else if (now_ns - l.better_since_ns >=
                       static_cast<int64_t>(recovery_hold_s_ * 1e9)) {
                l.level = rec;                 // 충분히 오래 유지됨 → 드디어 회복 인정
                l.better_since_ns = -1;
            }
        }
        st.level = l.level;                    // 최종 레벨은 래치가 결정
        if (st.level != raw) st.message += " [latched]";   // 래치가 개입했음을 메시지에 표시
    }

    rclcpp::Node::SharedPtr node_;             // ROS 노드 핸들 (스마트포인터)
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr scores_pub_;   // /monitor/scores 송신기
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;     // /diagnostics 송신기
    std::vector<SensorEntry> sensors_;         // 설정에서 읽은 센서 목록
    std::map<std::string, Latch> latches_;     // 진단 항목 이름 → 래치 상태
    double recovery_hold_s_ = 3.0;             // 회복 인정까지 유지해야 하는 시간(초)
    double recovery_margin_ = 0.2;             // 회복 인정에 필요한 점수 여유
    bool warmup_done_ = false;                 // 워밍업 완료 여부
    double warmup_timeout_s_ = 120.0;          // 워밍업 최대 대기 시간(초)
    std::chrono::steady_clock::time_point warmup_start_;   // 워밍업 시작 시각
    uint64_t tick_count_ = 0;                  // tick 횟수 (10회마다 요약 출력용)
};
