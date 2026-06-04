"""First-claim deduplication of traffic lights across lanelet2 relations (pure).

A lanelet2 regulatory_element relation lists every traffic light a lanelet
obeys, so one physical signal is referenced by several relations
(NishiShinjuku: 93/99 placed signals sit in 2..5 relations each). Creating one
group + controller per relation therefore attached the same signal to multiple
controllers: only the last add kept the component back-reference, while the
other "ghost" groups kept driving (and overriding) the signal's state every
cycle -- the true root cause of the freeze defect ([[021]]#4: client set_state
was overridden even when the signal's own group was frozen, 2026-06-04
measurement: 111 groups, 42 with back-references, 69 ghosts).

Fix: the FIRST relation to claim a signal owns it; later relations see it
filtered out, and a relation left with no member spawns no group at all.
"""


def claim_resolved(refers, resolvable, claimed):
    """Filter `refers` (sign-id strings, relation order) down to ids not yet in
    `claimed`, and mark the surviving ids that are in `resolvable` (= have a
    placed actor) as claimed. Unresolvable ids pass through WITHOUT being
    claimed so a later relation that can resolve them (level-search fallback)
    may still own them. Returns the filtered list, order preserved."""
    out = []
    for sid in refers:
        if sid in claimed:
            continue
        out.append(sid)
        if sid in resolvable:
            claimed.add(sid)
    return out
