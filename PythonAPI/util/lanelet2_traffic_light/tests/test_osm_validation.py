"""Unit tests for osm_validation.validate_osm_path.

Pure-Python OSM path validation, split out so it can be unit-tested
without the unreal module (used by run_placement at editor runtime).
"""
import os
import tempfile


def test_empty_path_returns_error():
    from lanelet2_traffic_light.frontend_editor.osm_validation import validate_osm_path
    msg = validate_osm_path("")
    assert msg is not None
    assert "LANELET2_OSM_PATH" in msg


def test_nonexistent_path_returns_error():
    from lanelet2_traffic_light.frontend_editor.osm_validation import validate_osm_path
    msg = validate_osm_path("/no/such/file.osm")
    assert msg is not None
    assert "LANELET2_OSM_PATH" in msg


def test_existing_path_returns_none():
    from lanelet2_traffic_light.frontend_editor.osm_validation import validate_osm_path
    with tempfile.NamedTemporaryFile(suffix=".osm", delete=False) as f:
        path = f.name
    try:
        assert validate_osm_path(path) is None
    finally:
        os.unlink(path)


def test_none_path_returns_error():
    from lanelet2_traffic_light.frontend_editor.osm_validation import validate_osm_path
    msg = validate_osm_path(None)
    assert msg is not None
