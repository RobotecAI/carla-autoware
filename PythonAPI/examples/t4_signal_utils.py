"""Helpers to find and inspect lanelet2-placed traffic lights (pure client side).

Ship alongside the map: `import t4_signal_utils as t4`.

Signals placed by the lanelet2 pipeline expose two string attributes:
  lanelet2_id  -- the lanelet2 sign way id (e.g. "1412")
  signal_kind  -- "vehicle" or "pedestrian"

Arrow mask layout (carla.TrafficLightArrow, int32, FROZEN):
  direction index: 0=Left 1=Straight 2=Right 3=UpLeft 4=UpRight
                   5=Down (reserved) 6=DownLeft 7=DownRight
  bits  0-7  green row (visually implemented)
  bits  8-15 yellow row (reserved)   bits 16-23 red row (reserved)
  bits 24-30 User1..User7 (special arrows, e.g. U-turn)
Sections OR together: set_arrow_state(GreenLeft | GreenStraight).
requested & ~get_arrow_capabilities() = bits with no physical face.
"""


def get_traffic_light_by_lanelet2_id(world, lanelet2_id):
    """The traffic light whose lanelet2_id attribute equals `lanelet2_id` (or None)."""
    wanted = str(lanelet2_id)
    for actor in world.get_actors().filter("traffic.traffic_light*"):
        if actor.attributes.get("lanelet2_id") == wanted:
            return actor
    return None


def get_traffic_lights_near(world, location, radius_m=30.0, kind=None):
    """Traffic lights within `radius_m` of `location` (carla.Location),
    optionally filtered by signal_kind ('vehicle' / 'pedestrian')."""
    out = []
    for actor in world.get_actors().filter("traffic.traffic_light*"):
        if kind and actor.attributes.get("signal_kind") != kind:
            continue
        if actor.get_location().distance(location) <= radius_m:
            out.append(actor)
    return out


def unlit_arrows(traffic_light, requested_mask):
    """Bits of `requested_mask` that cannot light on this signal (no physical face)."""
    return int(requested_mask) & ~traffic_light.get_arrow_capabilities()
