"""Signal-head label predicate shared by snap targeting and native cleanup.

Odaiba's native meshes mix lamp heads (Traffic_Lights_249), pole+arm
assemblies (Traffic_Lights_Pole_035) and ground cabinets
(Traffic_Lights_Ground_030) in one prefix family. Only lamp heads are valid
snap targets / deletion candidates; matching the others snapped 3 signals
onto poles and collaterally deleted a pole next to a placed pedestrian light
(2026-06-11 map update #2 bake).
"""

# Substrings that mark a prefix-matching label as NOT a signal head.
NON_HEAD_SUBSTRINGS = ("Pole", "Ground")


def is_signal_head_label(label, prefixes, exclude_substrings=NON_HEAD_SUBSTRINGS):
    """True when `label` names a native signal-head mesh."""
    if not label.startswith(tuple(prefixes)):
        return False
    return not any(x in label for x in exclude_substrings)
