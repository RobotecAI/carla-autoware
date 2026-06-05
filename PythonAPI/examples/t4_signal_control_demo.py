#!/usr/bin/env python3
"""End-to-end demo of lanelet2 traffic light control (spec 2026-06-04 section 6).

Run while the editor is in PIE on NishishinjukuMap after EUW Full Run
(+ Extract Native for pedestrians) and hide native:
    python3 t4_signal_control_demo.py [--vehicle-id 1412] [--ped-id 1550]
Output: grep-able key=value lines; visual checks are noted inline.
"""
import argparse
import time

import carla

import t4_signal_utils as t4


def check(name, ok):
    print(f"demo {name}={'OK' if ok else 'NG'}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle-id", default="1412")
    ap.add_argument("--ped-id", default="1550")
    ap.add_argument("--no-face-id", default="1478")
    args = ap.parse_args()

    world = carla.Client("localhost", 2000).get_world()
    world.freeze_all_traffic_lights(True)
    print("demo freeze_all=ON")

    tl = t4.get_traffic_light_by_lanelet2_id(world, args.vehicle_id)
    ped = t4.get_traffic_light_by_lanelet2_id(world, args.ped_id)
    check("found_vehicle", tl is not None
          and tl.attributes.get("signal_kind") == "vehicle")
    if ped is not None and ped.attributes.get("signal_kind") != "pedestrian":
        print(f"demo found_pedestrian=NG reason=kind_mismatch "
              f"(id {args.ped_id} is {ped.attributes.get('signal_kind')}; "
              f"pass --ped-id with a real pedestrian id)")
        peds = [a.attributes.get("lanelet2_id")
                for a in world.get_actors().filter("traffic.traffic_light*")
                if a.attributes.get("signal_kind") == "pedestrian"]
        print(f"demo pedestrian_ids_sample={peds[:8]}")
        ped = None
    else:
        check("found_pedestrian", ped is not None)
    if tl is None:
        print("demo verdict=ABORT (attributes missing? rebuild engine / rerun Full Run)")
        return

    tl.set_state(carla.TrafficLightState.Red)
    tl.set_arrow_state(int(carla.TrafficLightArrow.GreenRight))
    time.sleep(1.0)
    check("red_held", tl.get_state() == carla.TrafficLightState.Red)
    check("arrow_echo", tl.get_arrow_state() == int(carla.TrafficLightArrow.GreenRight))
    print(f"demo caps={tl.get_arrow_capabilities():#x} (1412 expects 0x7) "
          f"-- visual: red + right arrow only")
    time.sleep(3.0)
    tl.set_arrow_state(int(carla.TrafficLightArrow.NONE))
    print("demo arrows=NONE -- visual: all arrows off")
    time.sleep(3.0)
    tl.set_arrow_state(0x7)
    print("demo arrows=ALL -- visual: three arrows lit")

    nf = t4.get_traffic_light_by_lanelet2_id(world, args.no_face_id)
    if nf is not None:
        req = int(carla.TrafficLightArrow.GreenRight)
        nf.set_arrow_state(req)
        print(f"demo no_face_unlit={t4.unlit_arrows(nf, req):#x} (1478 expects 0x4)")

    if ped is not None:
        for state in ("Red", "Green", "Yellow", "Off"):
            ped.set_state(getattr(carla.TrafficLightState, state))
            print(f"demo ped_state={state} -- visual: standing red / walking green / "
                  f"blinking green / off")
            time.sleep(2.0)

    world.freeze_all_traffic_lights(False)
    print("demo freeze_all=OFF -- visual: cycle resumes")


if __name__ == "__main__":
    main()
