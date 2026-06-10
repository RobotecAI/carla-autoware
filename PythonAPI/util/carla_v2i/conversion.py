"""Pure CARLA -> Autoware traffic light element conversion (no ROS imports).

Mirrors autoware_perception_msgs/TrafficLightElement constants; node.py
asserts at startup that these values match the imported message definition.
Ported from AWSIM V2IRos2Publisher (AWF) -- the inverse of the SS2 decode
mapping used by ss2_carla_bridge.
"""

# autoware_perception_msgs/TrafficLightElement constants (mirrored).
COLOR_UNKNOWN, COLOR_RED, COLOR_AMBER, COLOR_GREEN, COLOR_WHITE = 0, 1, 2, 3, 4
SHAPE_CIRCLE = 1
SHAPE_LEFT, SHAPE_RIGHT, SHAPE_UP = 2, 3, 4
SHAPE_UP_LEFT, SHAPE_UP_RIGHT = 5, 6
SHAPE_DOWN, SHAPE_DOWN_LEFT, SHAPE_DOWN_RIGHT = 7, 8, 9
SHAPE_CROSS = 10
STATUS_SOLID_OFF, STATUS_SOLID_ON, STATUS_FLASHING = 1, 2, 3

CONFIDENCE = 1.0  # simulator ground truth (AWSIM parity)

# carla.TrafficLightArrow bit layout (FROZEN, see TrafficLightArrowState.h):
# bit = 1 << (8*row + dir); rows green/yellow/red; dir order below.
_ROW_COLORS = (COLOR_GREEN, COLOR_AMBER, COLOR_RED)
_DIR_SHAPES = (SHAPE_LEFT, SHAPE_UP, SHAPE_RIGHT, SHAPE_UP_LEFT,
               SHAPE_UP_RIGHT, SHAPE_DOWN, SHAPE_DOWN_LEFT, SHAPE_DOWN_RIGHT)

_VEHICLE_CIRCLE = {
    "red": (COLOR_RED, STATUS_SOLID_ON),
    "yellow": (COLOR_AMBER, STATUS_SOLID_ON),
    "green": (COLOR_GREEN, STATUS_SOLID_ON),
}
_PEDESTRIAN_CIRCLE = {
    "red": (COLOR_RED, STATUS_SOLID_ON),
    "green": (COLOR_GREEN, STATUS_SOLID_ON),
    # Pedestrian Yellow is rendered by the BP as green blink; report the
    # semantic state (green flashing), not the internal encoding.
    "yellow": (COLOR_GREEN, STATUS_FLASHING),
}


def elements_for(main_state: str, arrow_mask: int,
                 signal_kind: str) -> list:
    """Lit TrafficLightElement tuples (color, shape, status, confidence)
    for one CARLA traffic light.

    main_state: "red" | "yellow" | "green" | "off".
    signal_kind: "vehicle" | "pedestrian"; any other value falls back to
    the vehicle table (matches node.py's attribute default).
    """
    elements = []
    circle_table = (_PEDESTRIAN_CIRCLE if signal_kind == "pedestrian"
                    else _VEHICLE_CIRCLE)
    circle = circle_table.get(main_state)
    if circle is not None:
        color, status = circle
        elements.append((color, SHAPE_CIRCLE, status, CONFIDENCE))
    for row, row_color in enumerate(_ROW_COLORS):
        for direction, shape in enumerate(_DIR_SHAPES):
            if arrow_mask & (1 << (8 * row + direction)):
                elements.append((row_color, shape, STATUS_SOLID_ON, CONFIDENCE))
    return elements


def build_relation_to_ways(osm_path: str) -> dict:
    """{relation_id: [way_id, ...]} for traffic_light regulatory elements.

    Reuses the lanelet2_traffic_light parser (sibling package on PYTHONPATH).
    parse_osm may warn (warnings module) about malformed ways it skips;
    watch stderr on startup when pointing at a new map.
    """
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    _, groups = parse_osm(osm_path)
    return {g.relation_id: list(g.refers) for g in groups if g.refers}


def assemble_groups(relation_to_ways: dict, states_by_way: dict,
                    id_mode: str = "relation") -> list:
    """[(group_id, representative_way, elements), ...] from current CARLA states.

    states_by_way: {way_id: (main_state, arrow_mask, signal_kind)}.
    relation mode: the first way of a relation that has a CARLA state acts
    as the representative (members of one relation show the same signal).
    way mode: one group per way that has a state (AWSIM WayId parity).

    Returns (group_id, representative_way, elements) so callers can attach
    per-light extras (e.g. predictions) without re-deriving the representative.
    """
    groups = []
    if id_mode == "way":
        seen = set()
        for relation_ways in relation_to_ways.values():
            for way in relation_ways:
                if way in states_by_way and way not in seen:
                    seen.add(way)
                    groups.append((way, way, elements_for(*states_by_way[way])))
        return groups
    for relation_id, relation_ways in relation_to_ways.items():
        representative = next(
            (w for w in relation_ways if w in states_by_way), None)
        if representative is None:
            continue
        groups.append((relation_id, representative,
                       elements_for(*states_by_way[representative])))
    return groups


# CARLA autonomous cycle order (carla.TrafficLight state machine).
_CYCLE_ORDER = ("green", "yellow", "red")


def predict_states(main_state: str, elapsed: float, green_t: float,
                   yellow_t: float, red_t: float, steps: int) -> list:
    """[(seconds_from_now, next_state), ...] for a CARLA-cycled light.

    Mirrors AWSIM development-private PredictFutureStates: walk the
    deterministic cycle table forward from the current position. Returns []
    when the light is not in the cycle (off/unknown) -- e.g. frozen lights
    have no future plan and honestly report no predictions.

    The first offset may be negative when elapsed exceeds the current phase
    duration (CARLA reads can overshoot by one tick); clamping is the
    caller's responsibility (node.py uses max(dt, 0.0)).
    """
    if main_state not in _CYCLE_ORDER:
        return []
    if any(d <= 0 for d in (green_t, yellow_t, red_t)):
        # A zero-duration phase would emit duplicate same-time predictions;
        # treat such lights as having no usable cycle plan.
        return []
    durations = {"green": green_t, "yellow": yellow_t, "red": red_t}
    index = _CYCLE_ORDER.index(main_state)
    out = []
    offset = durations[main_state] - elapsed
    for _ in range(steps):
        index = (index + 1) % len(_CYCLE_ORDER)
        out.append((offset, _CYCLE_ORDER[index]))
        offset += durations[_CYCLE_ORDER[index]]
    return out
