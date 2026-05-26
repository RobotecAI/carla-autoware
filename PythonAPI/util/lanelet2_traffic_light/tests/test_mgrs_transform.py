import pytest
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer


def test_local_xy_to_unreal_cm_simple():
    """Odaiba 風: offset_y で y を反転。"""
    t = MgrsTransformer(offset_x_m=10.0, offset_y_m=20.0, offset_z_m=0.0,
                        x_sign=+1, y_sign=-1)
    # local=(15m, 30m), height=4.5m
    # X = +1 * (15 - 10) * 100 = +500
    # Y = -1 * (30 - 20) * 100 = -1000
    # Z = 4.5 * 100 = 450
    x, y, z = t.local_to_unreal_cm(local_x_m=15.0, local_y_m=30.0, height_m=4.5)
    assert x == pytest.approx(500.0)
    assert y == pytest.approx(-1000.0)
    assert z == pytest.approx(450.0)


def test_origin_at_offset_maps_to_zero():
    """local 座標が offset 値と同じなら Unreal は原点。"""
    t = MgrsTransformer(offset_x_m=100.0, offset_y_m=200.0, offset_z_m=0.0,
                        x_sign=+1, y_sign=-1)
    x, y, z = t.local_to_unreal_cm(100.0, 200.0, 0.0)
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.0))


def test_y_sign_positive_variant():
    """y_sign=+1 のケース (他マップ用の余地)。"""
    t = MgrsTransformer(offset_x_m=0.0, offset_y_m=0.0, offset_z_m=0.0,
                        x_sign=+1, y_sign=+1)
    x, y, _ = t.local_to_unreal_cm(10.0, 20.0, 0.0)
    assert x == pytest.approx(1000.0)
    assert y == pytest.approx(2000.0)


def test_x_sign_minus_one():
    """x_sign=-1 のケース。"""
    t = MgrsTransformer(offset_x_m=0.0, offset_y_m=0.0, offset_z_m=0.0,
                        x_sign=-1, y_sign=-1)
    x, y, _ = t.local_to_unreal_cm(10.0, 20.0, 0.0)
    assert x == pytest.approx(-1000.0)
    assert y == pytest.approx(-2000.0)


def test_odaiba_realistic_values():
    """Phase 0 で実機検証した Odaiba way_id=6621 のケース。"""
    t = MgrsTransformer(offset_x_m=92008.5, offset_y_m=45335.1, offset_z_m=0.0,
                        x_sign=+1, y_sign=-1)
    # way_id=6621 midpoint
    x, y, z = t.local_to_unreal_cm(local_x_m=89133.499, local_y_m=42693.062,
                                   height_m=12.3)
    # Predicted: (-287500.1, +264203.8, +1230.0)
    assert x == pytest.approx(-287500.1, abs=0.5)
    assert y == pytest.approx(+264203.8, abs=0.5)
    assert z == pytest.approx(+1230.0, abs=0.5)
