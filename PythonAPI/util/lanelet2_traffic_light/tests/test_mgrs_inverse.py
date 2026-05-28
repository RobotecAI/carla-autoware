import pytest
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer


def test_inverse_roundtrip():
    """Verify that unreal_cm_to_local is the correct inverse of local_to_unreal_cm."""
    tf = MgrsTransformer(offset_x_m=81000.0, offset_y_m=50000.0, offset_z_m=0.0, x_sign=1, y_sign=-1)
    # Forward: local (m) -> Unreal (cm)
    x, y, z = tf.local_to_unreal_cm(81596.1357, 50194.0803, 9.18)
    # Inverse: Unreal (cm) -> local (m)
    lx, ly, h = tf.unreal_cm_to_local(x, y, z)
    assert abs(lx - 81596.1357) < 1e-6
    assert abs(ly - 50194.0803) < 1e-6
    assert abs(h - 9.18) < 1e-6


def test_inverse_at_origin():
    """When Unreal position is the origin, inverse should map back to offset."""
    t = MgrsTransformer(offset_x_m=100.0, offset_y_m=200.0, offset_z_m=0.0,
                        x_sign=+1, y_sign=-1)
    lx, ly, h = t.unreal_cm_to_local(0.0, 0.0, 0.0)
    assert abs(lx - 100.0) < 1e-9
    assert abs(ly - 200.0) < 1e-9
    assert abs(h - 0.0) < 1e-9


def test_inverse_y_sign_positive():
    """Inverse with y_sign=+1."""
    t = MgrsTransformer(offset_x_m=0.0, offset_y_m=0.0, offset_z_m=0.0,
                        x_sign=+1, y_sign=+1)
    x, y, z = t.local_to_unreal_cm(10.0, 20.0, 5.0)
    lx, ly, h = t.unreal_cm_to_local(x, y, z)
    assert abs(lx - 10.0) < 1e-9
    assert abs(ly - 20.0) < 1e-9
    assert abs(h - 5.0) < 1e-9


def test_inverse_x_sign_negative():
    """Inverse with x_sign=-1."""
    t = MgrsTransformer(offset_x_m=0.0, offset_y_m=0.0, offset_z_m=0.0,
                        x_sign=-1, y_sign=-1)
    x, y, z = t.local_to_unreal_cm(10.0, 20.0, 5.0)
    lx, ly, h = t.unreal_cm_to_local(x, y, z)
    assert abs(lx - 10.0) < 1e-9
    assert abs(ly - 20.0) < 1e-9
    assert abs(h - 5.0) < 1e-9


def test_inverse_with_nonzero_z_offset():
    """Inverse with non-zero offset_z_m."""
    t = MgrsTransformer(offset_x_m=50.0, offset_y_m=100.0, offset_z_m=10.0,
                        x_sign=+1, y_sign=-1)
    x, y, z = t.local_to_unreal_cm(55.0, 105.0, 15.0)
    lx, ly, h = t.unreal_cm_to_local(x, y, z)
    assert abs(lx - 55.0) < 1e-9
    assert abs(ly - 105.0) < 1e-9
    assert abs(h - 15.0) < 1e-9
