#!/bin/bash
# fs_failsafe 배포 절차 (참고용 — 경로는 환경에 맞게 조정)
# 전제: AWCL 4-터미널 도커 구성 (CARLA 호스트 / bridge+leaderboard, autoware 컨테이너)
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"

echo "1) ROS 패키지 → colcon 워크스페이스"
echo "   cp -r $REPO/ros/* ~/fs_ws/src/ && cd ~/fs_ws && colcon build --symlink-install"

echo "2) 서드파티 패치 적용"
echo "   cd ~/autoware_awcl/autoware/src/launcher/autoware_launch && git apply $REPO/patches/autoware_launch.patch"
echo "   cd ~/autoware_carla_leaderboard && git apply $REPO/patches/carla_cpp_bridge.patch"
echo "   cd ~/autoware_carla_leaderboard && git apply $REPO/patches/leaderboard_and_agent.patch"
echo "   (bridge는 컨테이너 안에서 colcon build 필요)"

echo "3) 실험 스크립트"
echo "   cp $REPO/experiments/* ~/autoware_carla_leaderboard/experiments/"
echo "   cp $REPO/scripts/run_leaderboard.sh ~/autoware_carla_leaderboard/"

echo "4) DDS 환경 (호스트 ~/.bashrc)"
echo "   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "   export CYCLONEDDS_URI=\$HOME/cyclone_localhost.xml   # scripts/에서 복사"
echo "   export ROS_DOMAIN_ID=0"

echo "5) 검증"
echo "   python3 experiments/experiment_runner.py --smoke --fault lnf --t 20"
