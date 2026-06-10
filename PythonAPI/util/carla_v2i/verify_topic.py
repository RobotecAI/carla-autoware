"""Topic-level gate: subscribe N seconds and cross-check against CARLA.

Run while CARLA + carla_v2i publisher are both up. Output: key=value lines,
last line verdict=PASS|FAIL.
"""
import argparse
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from autoware_perception_msgs.msg import TrafficLightGroupArray

from carla_v2i import conversion


def main():
    parser = argparse.ArgumentParser(
        description="Topic gate for carla_v2i. Freeze CARLA traffic lights "
                    "first (world.freeze_all_traffic_lights(True)) -- with "
                    "cycling lights the snapshot comparison can flake.")
    parser.add_argument("--osm-path", required=True)
    parser.add_argument("--topic", default="/v2x/traffic_signals")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--carla-host", default="localhost")
    parser.add_argument("--carla-port", type=int, default=2000)
    args = parser.parse_args()

    rclpy.init()
    node = Node("v2i_verify")
    received = []
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    node.create_subscription(TrafficLightGroupArray, args.topic,
                             received.append, qos)
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rate = len(received) / args.seconds

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(10.0)
    world = client.get_world()
    states_by_way = {}
    name_of = {carla.TrafficLightState.Red: "red",
               carla.TrafficLightState.Yellow: "yellow",
               carla.TrafficLightState.Green: "green",
               carla.TrafficLightState.Off: "off",
               carla.TrafficLightState.Unknown: "off"}
    for actor in world.get_actors().filter("traffic.traffic_light*"):
        way = actor.attributes.get("lanelet2_id")
        if way:
            kind = actor.attributes.get("signal_kind", "vehicle")
            arrow = actor.get_arrow_state() if kind == "vehicle" else 0
            states_by_way[int(way)] = (name_of[actor.get_state()], arrow, kind)
    rel2ways = conversion.build_relation_to_ways(args.osm_path)
    expected = {gid: elems for gid, _rep, elems in
                conversion.assemble_groups(rel2ways, states_by_way)}

    ok_rate = 8.0 <= rate <= 12.0
    last = received[-1] if received else None
    got = {}
    if last is not None:
        for group in last.traffic_light_groups:
            got[group.traffic_light_group_id] = [
                (e.color, e.shape, e.status, e.confidence)
                for e in group.elements]
    ok_ids = set(got) == set(expected)
    mismatched = [gid for gid in got
                  if [tuple(e) for e in got[gid]] != expected.get(gid)]
    print(f"v2i_verify msgs={len(received)} rate_hz={rate:.1f} "
          f"groups={len(got)} expected_groups={len(expected)} "
          f"mismatched={len(mismatched)}")
    verdict = ok_rate and ok_ids and not mismatched
    print(f"v2i_verify verdict={'PASS' if verdict else 'FAIL'} "
          f"rate_ok={ok_rate} ids_ok={ok_ids} sample_mismatch={mismatched[:5]}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
