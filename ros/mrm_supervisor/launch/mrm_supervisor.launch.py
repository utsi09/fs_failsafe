"""mrm_supervisor 노드를 띄우는 런치 파일.

[런치 파일이란]
  ROS2에서 노드를 실행하는 방법은 두 가지다:
    1) ros2 run 패키지 실행파일        — 노드 하나를 맨손으로 실행
    2) ros2 launch 패키지 런치파일     — 이 파일처럼 "무엇을, 어떤 설정으로"
       실행할지를 파이썬 코드로 적어 둔 실행 각본(launch description)
  런치 파일을 쓰면 파라미터 값, 노드 이름, 출력 방식 등을 한곳에 모아두고
  명령행에서 인자만 바꿔 재사용할 수 있다.

사용 예:
  ros2 launch mrm_supervisor mrm_supervisor.launch.py
  ros2 launch mrm_supervisor mrm_supervisor.launch.py baseline:=true
"""
from launch import LaunchDescription                      # 런치 각본의 최상위 컨테이너
from launch.actions import DeclareLaunchArgument          # "명령행 인자"를 선언하는 액션
from launch.substitutions import LaunchConfiguration      # 인자 값을 나중에(실행 시점에) 꺼내오는 자리표시자
from launch_ros.actions import Node                       # "노드 하나를 실행해라"는 액션


def generate_launch_description():
    """ros2 launch가 이 함수를 찾아 호출한다 (이름이 정확히 이래야 한다)."""
    return LaunchDescription([
        # DeclareLaunchArgument: 명령행에서 이름:=값 형태로 덮어쓸 수 있는 인자.
        # 예: map_path:=/다른/지도.osm
        DeclareLaunchArgument(
            "map_path", default_value="/home/taewook/awcl_maps/Town07/lanelet2_map.osm"),
        # baseline:=true 면 supervisor가 갓길 이동 없이 곧바로 Autoware 기본
        # 정지에 위임하는 비교실험(대조군) 모드로 동작한다 (supervisor.py 참고).
        DeclareLaunchArgument("baseline", default_value="false"),
        Node(
            package="mrm_supervisor",     # 어느 패키지의
            executable="supervisor",      # 어느 실행파일을 (setup.py의 entry_points 이름)
            name="mrm_supervisor",        # 노드 이름은 이걸로 해서
            output="screen",              # 로그를 터미널 화면에 그대로 출력하며 실행
            parameters=[{
                # use_sim_time=True: 벽시계 대신 시뮬레이터(CARLA)의 /clock
                # 시간을 쓴다. 시뮬레이션이 일시정지/감속되면 노드의 타이머와
                # 타임아웃(45초 정체 판정 등)도 같이 멈추거나 늘어난다.
                "use_sim_time": True,
                # LaunchConfiguration("이름"): 위에서 선언한 인자의 실제 값이
                # 실행 시점에 여기로 채워진다 (자리표시자).
                "map_path": LaunchConfiguration("map_path"),
                "baseline": LaunchConfiguration("baseline"),
            }],
        ),
    ])
