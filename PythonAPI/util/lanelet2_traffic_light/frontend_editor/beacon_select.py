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
