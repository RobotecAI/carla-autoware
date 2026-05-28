import pytest
from lanelet2_traffic_light.corelib.osm_feedback.id_allocator import allocate_id_base


def test_default_base_when_small_max():
    assert allocate_id_base(3_013_207, 260) == 10_000_000


def test_next_million_boundary_when_large_max():
    assert allocate_id_base(10_500_000, 4) == 11_000_000
    assert allocate_id_base(12_000_000, 4) == 13_000_000  # strictly above max


def test_fallback_to_max_plus_one_near_limit():
    limit = 2**63 - 1
    base = allocate_id_base(limit - 2, 2, id_limit=limit)
    assert base == limit - 1  # max+1, since rounded base would overflow


def test_overflow_raises():
    limit = 2**63 - 1
    with pytest.raises(OverflowError):
        allocate_id_base(limit, 5, id_limit=limit)
