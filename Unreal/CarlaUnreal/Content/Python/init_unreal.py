"""Python initialisation hook loaded by the CARLA UE5 Editor at startup.

UE5 automatically executes Project_Root/Content/Python/init_unreal.py when the
Editor starts.  This module adds the lanelet2_traffic_light utility to the
Tools menu via an Editor Utility Widget (EUW) entry.

Design spec: docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md
Phase 0 notes: PythonAPI/util/lanelet2_traffic_light/docs/phase0_validation.md
"""
import os
import sys

import unreal


# Add the lanelet2_traffic_light package parent directory to sys.path.
# Relative to the project root: PythonAPI/util/ is the parent of the package.
def _add_package_path_to_sys_path():
    project_dir = unreal.Paths.project_dir()  # e.g. ".../Unreal/CarlaUnreal/"
    package_parent = unreal.Paths.convert_relative_path_to_full(
        os.path.join(project_dir, "..", "..", "PythonAPI", "util")
    )
    if os.path.isdir(package_parent) and package_parent not in sys.path:
        sys.path.insert(0, package_parent)
        unreal.log(f"init_unreal: added sys.path: {package_parent}")


_add_package_path_to_sys_path()


# Editor Utility Widget for GUI-based placement (Phase 7). Lives in the T4 plugin content.
EUW_ASSET_PATH = "/T4/Lanelet2TrafficLight/EUW_LaneletTrafficLight.EUW_LaneletTrafficLight"


# ---------------------------------------------------------------------------
# Menu registration
# ---------------------------------------------------------------------------

def _register_menu_entries():
    """Register the Tool menu entry (the EUW) under LevelEditor.MainMenu.Tools."""
    menus = unreal.ToolMenus.get()
    main_menu = menus.find_menu("LevelEditor.MainMenu.Tools")
    if main_menu is None:
        unreal.log_warning("init_unreal: LevelEditor.MainMenu.Tools not found.")
        return

    section_name = "LaneletTrafficLight"

    # Open the Editor Utility Widget (GUI for OSM path + placement).
    entry_widget = unreal.ToolMenuEntry(
        name="LaneletTL_OpenWidget",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_widget.set_label("Generate Traffic Lights from lanelet2...")
    entry_widget.set_tool_tip(
        "Open the EUW_LaneletTrafficLight widget to enter an OSM path and run "
        "placement from a GUI."
    )
    entry_widget.set_string_command(
        type=unreal.ToolMenuStringCommandType.PYTHON,
        custom_type=unreal.Name(""),
        string=(
            "import unreal; "
            f"asset = unreal.EditorAssetLibrary.load_asset('{EUW_ASSET_PATH}'); "
            "(asset and unreal.EditorUtilitySubsystem().spawn_and_register_tab(asset)) "
            "or unreal.log_error('EUW_LaneletTrafficLight not found; create it first (Phase 3.5).')"
        ),
    )
    main_menu.add_menu_entry(section_name, entry_widget)

    menus.refresh_all_widgets()
    unreal.log("init_unreal: lanelet2_traffic_light menu entries registered.")


try:
    _register_menu_entries()
except Exception as e:
    unreal.log_error(f"init_unreal: menu registration failed: {e}")
