from lanelet2_traffic_light.frontend_editor.save_level import resolve_save_level


def test_explicit_true_wins_over_env():
    assert resolve_save_level(True, environ={}) is True


def test_explicit_false_wins_over_env():
    assert resolve_save_level(False, environ={"LANELET2_SAVE_LEVEL": "1"}) is False


def test_env_truthy_values():
    for v in ("1", "true", "TRUE", "yes", "On"):
        assert resolve_save_level(None, environ={"LANELET2_SAVE_LEVEL": v}) is True


def test_env_falsy_or_unset():
    for env in (
        {},
        {"LANELET2_SAVE_LEVEL": ""},
        {"LANELET2_SAVE_LEVEL": "0"},
        {"LANELET2_SAVE_LEVEL": "false"},
    ):
        assert resolve_save_level(None, environ=env) is False
