"""Beacon material-slot selection shared by audit, apply and docs.

A 'flashing beacon' actor is identified by its material slot names, not by
actor label: pre-apply the native lens slot is named Orange_Light (Odaiba
mesh family Scene_1269/1270); post-apply the slot holds M_JPFlashingBeaconLit,
so the applied material itself becomes the runtime tag (BP_FlashingBeaconManager
matches on the same substring). Pure module -- no unreal import -- so the
selection rule is unit-testable.
"""


def beacon_element_indices(material_names, marker):
    """Indices of material slots whose name contains `marker`.

    `material_names` may contain None for unassigned slots (skipped).
    """
    return [i for i, name in enumerate(material_names)
            if name is not None and marker in name]


def largest_gap(values):
    """(gap_size, gap_midpoint) of the largest gap in sorted `values`.

    A two-lamp mesh section shows a large gap along the lamp-separation axis
    (and along one UV axis when the lamps occupy disjoint texture regions);
    the midpoint is the natural mask split value.
    """
    s = sorted(values)
    if not s:
        return 0.0, 0.0
    if len(s) == 1:
        return 0.0, s[0]
    gap, mid = 0.0, s[0]
    for a, b in zip(s, s[1:]):
        if b - a > gap:
            gap, mid = b - a, (a + b) / 2.0
    return gap, mid
