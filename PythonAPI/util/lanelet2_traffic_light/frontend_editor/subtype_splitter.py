"""Per-subtype member splitting logic (Phase 6).

Pure-Python helper extracted from editor_placer._place_groups.
Designed to be unit-testable without the unreal module dependency.
"""


def split_members_by_subtype(refers, sign_id_to_actor, sign_id_to_subtype):
    """Build a per-subtype actor list from a list of way_ids.

    Args:
        refers: GroupSpec.refers (list of way_id ints).
        sign_id_to_actor: dict of {sign_id_str: actor}. actor can be any type.
        sign_id_to_subtype: dict of {sign_id_str: subtype_str}.

    Returns:
        dict of {subtype_str: [actor, ...]}.
        - Ways in refers that are not present in sign_id_to_actor are skipped.
        - Unknown subtypes (not in sign_id_to_subtype, or empty string) are kept
          as an independent group under key "".
        - Actor order within each subtype preserves the order of appearance in refers.
    """
    result: dict = {}
    for way_id in refers:
        sid = str(way_id)
        actor = sign_id_to_actor.get(sid)
        if actor is None:
            continue
        subtype = sign_id_to_subtype.get(sid, "")
        result.setdefault(subtype, []).append(actor)
    return result
