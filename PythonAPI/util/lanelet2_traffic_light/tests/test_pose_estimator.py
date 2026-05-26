import pytest
from lanelet2_traffic_light.core.ir.traffic_light_ir import Node, TrafficLightSpec
from lanelet2_traffic_light.core.geometry.pose_estimator import estimate_center_and_yaw


def _spec(p0_xy, p1_xy):
    n0 = Node(1, 0, 0, p0_xy[0], p0_xy[1], "")
    n1 = Node(2, 0, 0, p1_xy[0], p1_xy[1], "")
    return TrafficLightSpec(1001, "red_yellow_green", n0, n1, 0.45, {})


def test_center_is_midpoint():
    spec = _spec((0.0, 0.0), (10.0, 0.0))
    cx, cy, yaw = estimate_center_and_yaw(spec, yaw_offset_deg=0.0)
    assert cx == pytest.approx(5.0)
    assert cy == pytest.approx(0.0)


def test_horizontal_way_yaw_zero_minus_90():
    # way が +X 方向: dx=10, dy=0 -> atan2(-0, 10) = 0, +(-90) = -90
    spec = _spec((0.0, 0.0), (10.0, 0.0))
    _, _, yaw = estimate_center_and_yaw(spec, yaw_offset_deg=-90.0)
    assert yaw == pytest.approx(-90.0)


def test_vertical_way_yaw_minus_180():
    # way が lanelet2 +Y 方向 (Unreal では -Y): dx=0, dy=10
    # atan2(-10, 0) = -90deg, +(-90) = -180 deg
    spec = _spec((0.0, 0.0), (0.0, 10.0))
    _, _, yaw = estimate_center_and_yaw(spec, yaw_offset_deg=-90.0)
    assert yaw == pytest.approx(-180.0)


def test_diagonal_way():
    # way = (+10, +10), dx=10, dy=10
    # atan2(-10, 10) = -45deg, +(-90) = -135 deg
    spec = _spec((0.0, 0.0), (10.0, 10.0))
    _, _, yaw = estimate_center_and_yaw(spec, yaw_offset_deg=-90.0)
    assert yaw == pytest.approx(-135.0)
