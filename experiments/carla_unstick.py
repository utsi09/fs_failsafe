#!/usr/bin/env python3
"""Reset CARLA to asynchronous mode (run inside the LB container).

The leaderboard puts the server in synchronous mode; when the runner
pkill -9's a case nobody calls world.tick() anymore, the game thread
freezes, and every later load_world() times out ("vehicle never spawns").
Normal leaderboard shutdown undoes this in _cleanup(); SIGKILL skips it.
"""
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
settings = world.get_settings()
was_sync = settings.synchronous_mode
if was_sync:
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
try:
    client.get_trafficmanager().set_synchronous_mode(False)
except Exception:
    pass
print(f"carla async ok (was_sync={was_sync}):", world.get_map().name)
