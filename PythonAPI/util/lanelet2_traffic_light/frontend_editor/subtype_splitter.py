"""subtype 別 member 分割ロジック (Phase 6)。

editor_placer._place_groups から切り出した pure-Python ヘルパー。
unreal モジュール非依存で unit test 可能にするのが目的。
"""


def split_members_by_subtype(refers, sign_id_to_actor, sign_id_to_subtype):
    """way_id リストから subtype 別の actor リストを構築。

    Args:
        refers: GroupSpec.refers (way_id の int リスト)。
        sign_id_to_actor: {sign_id_str: actor} の dict。actor は何でも良い。
        sign_id_to_subtype: {sign_id_str: subtype_str} の dict。

    Returns:
        {subtype_str: [actor, ...]} の dict。
        - refers 内で sign_id_to_actor に無い way は skip。
        - subtype 未知 (sign_id_to_subtype に無い、または空文字) はキー "" の
          独立群として保持。
        - 同 subtype 内の actor 順は refers の出現順を保持。
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
