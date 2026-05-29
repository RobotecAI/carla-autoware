"""Policy: when to skip snap mesh-override.

Pure logic, no `unreal` dependency (unit-testable).

Inc4: pedestrians now adopt the snapped per-map Scene_NNN mesh (so the lamp
faces keep their natural orientation), and the lit figures are driven at
runtime via a material override (see BP_PedestrianTrafficLightSceneFigure).
Therefore snap mesh-override is NEVER skipped — both vehicle and pedestrian
traffic lights take the unified snap path.
"""


def should_skip_mesh_override(actor_class_path: str) -> bool:
    """Return True when the actor's snap mesh-override must be skipped.

    Always False post-Inc4 (kept as a stable seam for callers/tests).
    """
    return False
