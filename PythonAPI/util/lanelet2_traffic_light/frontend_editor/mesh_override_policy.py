"""Policy: when to skip snap mesh-override and keep the BP's canonical mesh.

Pure logic, no `unreal` dependency (unit-testable).

Pedestrian traffic lights keep their parent BP's 3-element canonical mesh
(Walk/Frame/Stop) so the parent BP's lighting logic can drive red/green
directly. Vehicle traffic lights keep the existing snap-override behavior
(adopt the per-map Scene_NNNN mesh) so their placement stays unchanged.

The decision mirrors `_label_prefixes_for_bp_class`, which keys on the
substring "Pedestrian" in the BP class path.
"""


def should_skip_mesh_override(actor_class_path: str) -> bool:
    """Return True when the actor's snap mesh-override must be skipped.

    Args:
        actor_class_path: BP class path of the spawned actor
            (e.g. ".../BP_OdaibaPedestrianTL.BP_OdaibaPedestrianTL").

    Returns:
        True for pedestrian BPs (keep canonical mesh), False otherwise.
    """
    return "Pedestrian" in actor_class_path
