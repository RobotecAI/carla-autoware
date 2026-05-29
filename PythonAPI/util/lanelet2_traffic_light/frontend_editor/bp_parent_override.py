"""Per-map parent-BP overrides for derived traffic-light BP generation.

Pure logic (no `unreal`). Inc4: Odaiba pedestrians (subtype "red_green") use a
dedicated T4 parent BP that drives the Scene_NNN lamp face via a runtime
material override. Other maps keep the profile default (canonical pedestrian BP).
"""

# T4 plugin asset path of the Scene_NNN figure-override pedestrian parent BP.
SCENE_FIGURE_PARENT_BP = (
    "/T4/TrafficLightSample/PedestrianTrafficLight/"
    "BP_PedestrianTrafficLightSceneFigure.BP_PedestrianTrafficLightSceneFigure"
)


def parent_bp_override_for_map(map_name: str) -> dict:
    """Return {subtype: parent_bp_path} overrides for the given map.

    Empty dict means "use profile defaults".
    """
    if map_name == "Odaiba":
        return {"red_green": SCENE_FIGURE_PARENT_BP}
    return {}
