"""CARLA UE5 Editor 起動時に読み込まれる Python 初期化フック。

UE5 は Editor 起動時に Project_Root/Content/Python/init_unreal.py を
自動的に実行する。ここで Tools メニューに lanelet2_traffic_light
ユーティリティへのアクセスを追加する。

設計仕様: docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md
Phase 0 知見: PythonAPI/util/lanelet2_traffic_light/docs/phase0_validation.md
"""
import os
import sys

import unreal


# lanelet2_traffic_light パッケージへのパスを sys.path に追加。
# Project root から見ると PythonAPI/util/lanelet2_traffic_light がパッケージ本体。
def _add_package_path_to_sys_path():
    project_dir = unreal.Paths.project_dir()  # 例: ".../Unreal/CarlaUnreal/"
    package_parent = unreal.Paths.convert_relative_path_to_full(
        os.path.join(project_dir, "..", "..", "PythonAPI", "util")
    )
    if os.path.isdir(package_parent) and package_parent not in sys.path:
        sys.path.insert(0, package_parent)
        unreal.log(f"init_unreal: added sys.path: {package_parent}")


_add_package_path_to_sys_path()


# lanelet2 OSM file for Quick/Full Run, set via LANELET2_OSM_PATH env var (required).
# No hard-coded fallback: an unset/missing path is reported at Run time.
DEFAULT_OSM_PATH = os.environ.get("LANELET2_OSM_PATH", "")

# Editor Utility Widget for GUI-based placement (Phase 7). Lives in the T4 plugin content.
EUW_ASSET_PATH = "/T4/Lanelet2TrafficLight/EUW_LaneletTrafficLight.EUW_LaneletTrafficLight"


# ---------------------------------------------------------------------------
# Run scripts: Quick (smoke test, first 5) と Full (all ~522)
# 共通テンプレートで生成。snap_to_existing_mesh=True で既存メッシュに pivot 一致。
# ---------------------------------------------------------------------------

def _build_run_script(limit):
    """Build the Python command string for the Quick/Full Run menu entries.

    Args:
        limit: place only the first `limit` placements when an int; all when None.
    """
    limit_expr = "None" if limit is None else str(limit)
    return f'''
import os
import unreal
try:
    from lanelet2_traffic_light.frontend_editor.run_placement import run_placement
except Exception as e:
    unreal.log_error(f"lanelet2_traffic_light import failed: {{e}}")
    raise
run_placement(os.environ.get("LANELET2_OSM_PATH", ""), limit={limit_expr})
'''


FULL_RUN_SCRIPT = _build_run_script(None)


# ---------------------------------------------------------------------------
# Menu registration
# ---------------------------------------------------------------------------

def _register_menu_entries():
    """Register the Tool menu entries (Widget, Full Run) under LevelEditor.MainMenu.Tools."""
    menus = unreal.ToolMenus.get()
    main_menu = menus.find_menu("LevelEditor.MainMenu.Tools")
    if main_menu is None:
        unreal.log_warning("init_unreal: LevelEditor.MainMenu.Tools not found.")
        return

    section_name = "LaneletTrafficLight"

    # Entry 1: open the Editor Utility Widget (GUI for OSM path + placement).
    entry_widget = unreal.ToolMenuEntry(
        name="LaneletTL_OpenWidget",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_widget.set_label("Generate Traffic Lights from lanelet2... (Widget)")
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

    # Entry 2: Full-Run (places all parsed traffic lights).
    # Takes a few minutes for large maps. Save the level with Ctrl+S afterwards.
    entry_full = unreal.ToolMenuEntry(
        name="LaneletTL_FullRun",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_full.set_label("Generate Traffic Lights from lanelet2 (Full Run, all)")
    entry_full.set_tool_tip(
        "Parse the lanelet2 OSM set via the LANELET2_OSM_PATH environment "
        "variable and spawn ALL parsed traffic lights into the current level. "
        "Takes a few minutes for large maps. Save the level with Ctrl+S "
        "after verifying the result."
    )
    entry_full.set_string_command(
        type=unreal.ToolMenuStringCommandType.PYTHON,
        custom_type=unreal.Name(""),
        string=FULL_RUN_SCRIPT.strip(),
    )
    main_menu.add_menu_entry(section_name, entry_full)

    menus.refresh_all_widgets()
    unreal.log("init_unreal: lanelet2_traffic_light menu entries registered.")


try:
    _register_menu_entries()
except Exception as e:
    unreal.log_error(f"init_unreal: menu registration failed: {e}")
