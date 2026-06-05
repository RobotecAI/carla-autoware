#!/usr/bin/env python3
"""Connect to a PIE session and reproduce the traffic-light freeze defect.

Run while the editor is in PIE on NishishinjukuMap or Odaiba (after EUW Full Run):
    python3 t4_signal_smoke.py
Output: grep-able key=value lines. freeze_all_held_30s=False reproduces the
defect this branch fixes (dynamically spawned groups ignored freeze).
"""
import time

import carla


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    # NOTE: world.get_map() is intentionally avoided -- lanelet2-driven maps may
    # carry no parseable OpenDRIVE and get_map() then raises (observed 2026-06-04).
    print(f"smoke connect=OK traffic_lights={len(lights)}")
    if not lights:
        print("smoke verdict=NG reason=no_traffic_lights (run EUW Full Run first)")
        return
    tl = lights[0]
    world.freeze_all_traffic_lights(True)
    tl.set_state(carla.TrafficLightState.Red)
    t0 = time.time()
    held = True
    while time.time() - t0 < 30.0:
        if tl.get_state() != carla.TrafficLightState.Red:
            held = False
            break
        time.sleep(1.0)
    print(f"smoke freeze_all_held_30s={held} per_light_frozen={tl.is_frozen()}")
    world.freeze_all_traffic_lights(False)
    print("smoke verdict=DONE (expected after fix: held=True frozen=True)")


if __name__ == "__main__":
    main()
