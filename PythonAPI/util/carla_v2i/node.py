"""V2I publisher node: CARLA traffic light states -> TrafficLightGroupArray.

Port of AWSIM's V2I / V2IRos2Publisher. Read-only towards CARLA (never
modifies world or actor state). Do NOT run together with
scenario_simulator_v2 V2I traffic lights -- ss2 publishes the same topics
itself (see README).
"""
import argparse
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from autoware_perception_msgs.msg import (TrafficLightGroupArray,
                                          TrafficLightGroup,
                                          TrafficLightElement)

from carla_v2i import conversion

_STATE_NAMES = {
    carla.TrafficLightState.Red: "red",
    carla.TrafficLightState.Yellow: "yellow",
    carla.TrafficLightState.Green: "green",
    carla.TrafficLightState.Off: "off",
    carla.TrafficLightState.Unknown: "off",
}


def _assert_msg_constants():
    """Fail fast if the mirrored constants drift from the installed msgs."""
    expected = {
        "UNKNOWN": conversion.COLOR_UNKNOWN,
        "RED": conversion.COLOR_RED, "AMBER": conversion.COLOR_AMBER,
        "GREEN": conversion.COLOR_GREEN, "WHITE": conversion.COLOR_WHITE,
        "CIRCLE": conversion.SHAPE_CIRCLE, "LEFT_ARROW": conversion.SHAPE_LEFT,
        "RIGHT_ARROW": conversion.SHAPE_RIGHT, "UP_ARROW": conversion.SHAPE_UP,
        "UP_LEFT_ARROW": conversion.SHAPE_UP_LEFT,
        "UP_RIGHT_ARROW": conversion.SHAPE_UP_RIGHT,
        "DOWN_ARROW": conversion.SHAPE_DOWN,
        "DOWN_LEFT_ARROW": conversion.SHAPE_DOWN_LEFT,
        "DOWN_RIGHT_ARROW": conversion.SHAPE_DOWN_RIGHT,
        "CROSS": conversion.SHAPE_CROSS,
        "SOLID_OFF": conversion.STATUS_SOLID_OFF,
        "SOLID_ON": conversion.STATUS_SOLID_ON,
        "FLASHING": conversion.STATUS_FLASHING,
    }
    bad = [(n, getattr(TrafficLightElement, n), v)
           for n, v in expected.items()
           if getattr(TrafficLightElement, n) != v]
    if bad:
        raise RuntimeError(f"TrafficLightElement constants drifted: {bad}")


class V2IPublisherNode(Node):

    def __init__(self, args):
        super().__init__("carla_v2i_publisher")
        _assert_msg_constants()
        self._args = args
        self._warned = set()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._publisher = self.create_publisher(
            TrafficLightGroupArray, args.topic, qos)
        self._relation_to_ways = conversion.build_relation_to_ways(args.osm_path)
        self.get_logger().info(
            f"v2i relations={len(self._relation_to_ways)} osm={args.osm_path}")
        self._connect_carla()
        self.create_timer(1.0 / args.rate_hz, self._tick)

    def _connect_carla(self):
        while True:
            try:
                client = carla.Client(self._args.carla_host,
                                      self._args.carla_port)
                client.set_timeout(10.0)
                self._world = client.get_world()
                # Keep the client alive: the World's RPC channel dies if the
                # Client is garbage-collected.
                self._client = client
                break
            except RuntimeError as e:
                self.get_logger().warning(f"carla connect retry: {e}")
                time.sleep(2.0)
        self._lights_by_way = {}
        for actor in self._world.get_actors().filter("traffic.traffic_light*"):
            way = actor.attributes.get("lanelet2_id")
            if way:
                kind = actor.attributes.get("signal_kind", "vehicle")
                self._lights_by_way[int(way)] = (actor, kind)
        self.get_logger().info(f"v2i lights={len(self._lights_by_way)}")

    def _warn_once(self, key, message):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(message)

    def _ego_location(self):
        vehicles = list(self._world.get_actors().filter("vehicle.*"))
        for role in dict.fromkeys((self._args.ego_role_name, "hero")):
            for actor in vehicles:
                if actor.attributes.get("role_name") == role:
                    return actor.get_location()
        return None

    def _tick(self):
        radius = self._args.radius_m
        ego = None
        if radius > 0:
            ego = self._ego_location()
            if ego is None:
                self._warn_once(
                    "no_ego",
                    "v2i ego not found (role_name=%s/hero); nothing published"
                    " -- use --radius-m 0 to publish all lights"
                    % self._args.ego_role_name)
                return
        states_by_way = {}
        for way, (actor, kind) in self._lights_by_way.items():
            try:
                if ego is not None and actor.get_location().distance(ego) > radius:
                    continue
                states_by_way[way] = (
                    _STATE_NAMES.get(actor.get_state(), "off"),
                    actor.get_arrow_state(), kind)
            except RuntimeError as e:
                self._warn_once(f"read_fail:{way}",
                                f"v2i read failed way={way}: {e}")
        if not states_by_way and self._lights_by_way:
            # Likely a CARLA world reload: every cached actor handle is stale.
            self._warn_once(
                "all_reads_failed",
                "v2i all light reads failed lights=%d -- CARLA world may have"
                " been reloaded; restart this node to rebuild actor handles"
                % len(self._lights_by_way))
        groups = conversion.assemble_groups(
            self._relation_to_ways, states_by_way, self._args.id_mode)
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        for group_id, elements in groups:
            group = TrafficLightGroup()
            group.traffic_light_group_id = group_id
            for color, shape, status, confidence in elements:
                element = TrafficLightElement()
                element.color, element.shape, element.status = color, shape, status
                element.confidence = confidence
                group.elements.append(element)
            msg.traffic_light_groups.append(group)
        self._publisher.publish(msg)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="CARLA V2I traffic signal publisher (AWSIM port)")
    parser.add_argument("--osm-path", dest="osm_path", required=True)
    parser.add_argument("--topic", default="/v2x/traffic_signals")
    parser.add_argument("--rate-hz", dest="rate_hz", type=float, default=10.0)
    parser.add_argument("--radius-m", dest="radius_m", type=float, default=150.0,
                        help="<=0 publishes all lights (no ego needed)")
    parser.add_argument("--id-mode", dest="id_mode",
                        choices=("relation", "way"), default="relation")
    parser.add_argument("--ego-role-name", dest="ego_role_name", default="ego")
    parser.add_argument("--carla-host", dest="carla_host", default="localhost")
    parser.add_argument("--carla-port", dest="carla_port", type=int, default=2000)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = None
    try:
        node = V2IPublisherNode(args)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
