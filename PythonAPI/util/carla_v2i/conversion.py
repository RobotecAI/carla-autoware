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
