#!/usr/bin/env python3
"""CARLA를 비동기(asynchronous) 모드로 되돌리는 응급 처치 스크립트
(LB 컨테이너 안에서 실행; experiment_runner의 carla_unstick()이 호출).

왜 필요한가:
  leaderboard는 서버를 동기(synchronous) 모드로 바꿔 놓는다 — 이 모드에선
  클라이언트가 world.tick()을 쳐줘야만 시뮬레이션 시간이 한 걸음씩 간다.
  그런데 러너가 케이스를 pkill -9로 끝내버리면 tick을 쳐줄 사람이
  사라지고, 게임 스레드는 오지 않는 tick을 기다리며 얼어붙는다(고아 상태).
  그 뒤의 모든 load_world()는 영원히 타임아웃("vehicle never spawns").
  정상 종료라면 leaderboard의 _cleanup()이 원상복구를 해주지만,
  SIGKILL(-9)은 정리 코드를 건너뛰므로 여기서 강제로 되돌린다.
"""
import carla

# 서버(localhost:2000)에 접속해 현재 월드 설정을 읽는다
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
settings = world.get_settings()
was_sync = settings.synchronous_mode
if was_sync:
    # 핵심 처치: 동기 모드 해제 + 고정 시간 간격 해제 → 서버가
    # tick 없이도 스스로 시간이 흐르는 비동기 모드로 복귀
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
try:
    # Traffic Manager(NPC 교통 담당)도 따로 동기 모드를 가진다 — 같이 해제
    client.get_trafficmanager().set_synchronous_mode(False)
except Exception:
    pass
print(f"carla async ok (was_sync={was_sync}):", world.get_map().name)
