"""Pure-logic tests for CARLA -> TrafficLightGroupArray conversion (no ROS)."""
from carla_v2i.conversion import (
    COLOR_RED, COLOR_AMBER, COLOR_GREEN,
    SHAPE_CIRCLE, SHAPE_LEFT, SHAPE_UP, SHAPE_RIGHT, SHAPE_DOWN_RIGHT,
    STATUS_SOLID_ON, STATUS_FLASHING, CONFIDENCE,
    elements_for,
)


def test_vehicle_red():
    assert elements_for("red", 0, "vehicle") == [
        (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_vehicle_yellow_is_amber():
    assert elements_for("yellow", 0, "vehicle") == [
        (COLOR_AMBER, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_vehicle_off_is_empty():
    assert elements_for("off", 0, "vehicle") == []


def test_pedestrian_yellow_is_green_flashing():
    assert elements_for("yellow", 0, "pedestrian") == [
        (COLOR_GREEN, SHAPE_CIRCLE, STATUS_FLASHING, CONFIDENCE)]


def test_pedestrian_red_and_green():
    assert elements_for("red", 0, "pedestrian") == [
        (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]
    assert elements_for("green", 0, "pedestrian") == [
        (COLOR_GREEN, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_green_arrows_appended():
    # GreenLeft(bit0) | GreenStraight(bit1) with red circle
    got = elements_for("red", 0b11, "vehicle")
    assert (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_GREEN, SHAPE_LEFT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_GREEN, SHAPE_UP, STATUS_SOLID_ON, CONFIDENCE) in got
    assert len(got) == 3


def test_arrow_rows_and_diagonal():
    # GreenDownRight(bit7) + YellowLeft(bit8) + RedRight(bit18)
    mask = (1 << 7) | (1 << 8) | (1 << 18)
    got = elements_for("off", mask, "vehicle")
    assert (COLOR_GREEN, SHAPE_DOWN_RIGHT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_AMBER, SHAPE_LEFT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_RED, SHAPE_RIGHT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert len(got) == 3


def test_user_bits_ignored():
    # User-defined bits (>=24) fall outside the 0-7 direction range the
    # row loop iterates, so they are never converted to elements.
    assert elements_for("off", 1 << 24, "vehicle") == []
