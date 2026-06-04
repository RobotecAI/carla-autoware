from lanelet2_traffic_light.frontend_editor.group_dedup import claim_resolved


def test_first_relation_claims_its_members():
    claimed = set()
    out = claim_resolved(["1", "2"], resolvable={"1", "2"}, claimed=claimed)
    assert out == ["1", "2"]
    assert claimed == {"1", "2"}


def test_later_relation_loses_already_claimed_members():
    claimed = {"1"}
    out = claim_resolved(["1", "2"], resolvable={"1", "2"}, claimed=claimed)
    assert out == ["2"]          # "1" is owned by an earlier relation
    assert claimed == {"1", "2"}


def test_relation_with_all_members_claimed_yields_empty():
    claimed = {"1", "2"}
    assert claim_resolved(["1", "2"], resolvable={"1", "2"}, claimed=claimed) == []


def test_unresolvable_ids_pass_through_unclaimed():
    # An id without a placed actor must NOT be claimed: a later relation that
    # CAN resolve it (level-search fallback) should still be able to own it.
    claimed = set()
    out = claim_resolved(["7", "8"], resolvable={"8"}, claimed=claimed)
    assert out == ["7", "8"]
    assert claimed == {"8"}


def test_order_preserved():
    claimed = set()
    out = claim_resolved(["3", "1", "2"], resolvable={"1", "2", "3"}, claimed=claimed)
    assert out == ["3", "1", "2"]
