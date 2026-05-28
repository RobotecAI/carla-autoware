DEFAULT_BASE = 10_000_000
ROUND = 1_000_000
ID_LIMIT = 2**63 - 1


def allocate_id_base(max_existing_id: int, num_new: int,
                     *, default_base: int = DEFAULT_BASE,
                     round_to: int = ROUND, id_limit: int = ID_LIMIT) -> int:
    """Return the starting id for `num_new` globally-unique new OSM elements.

    Prefers a clean, well-separated base above the existing max id; falls back
    to max+1 if that would overflow; raises if even that cannot fit `num_new`
    ids within id_limit.
    """
    if num_new <= 0:
        return max_existing_id + 1
    if max_existing_id < default_base:
        base = default_base
    else:
        base = ((max_existing_id // round_to) + 1) * round_to
    if base + num_new - 1 > id_limit:
        base = max_existing_id + 1
        if base + num_new - 1 > id_limit:
            raise OverflowError(
                "cannot allocate %d ids: max_existing_id=%d would overflow id_limit=%d"
                % (num_new, max_existing_id, id_limit))
    return base
