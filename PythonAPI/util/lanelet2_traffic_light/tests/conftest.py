import os
import pytest

FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")


@pytest.fixture
def minimal_osm_path():
    return os.path.join(FIXTURE_DIR, "minimal.osm")
