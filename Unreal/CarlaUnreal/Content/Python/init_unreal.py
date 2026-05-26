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


# Quick-Run で使う既定 lanelet2 ファイル
DEFAULT_OSM_PATH = (
    "/mnt/dsk0/wk0/CARLA/autoware_map/odaiba_autoware_map_2025_01_16/lanelet2_map.osm"
)

# Phase 3.5 で作成予定の EUW (まだ無くて OK)
EUW_ASSET_PATH = "/Game/EditorUtilities/EUW_LaneletTrafficLight.EUW_LaneletTrafficLight"


# ---------------------------------------------------------------------------
# Run scripts: Quick (smoke test, first 5) と Full (all ~522)
# 共通テンプレートで生成。snap_to_existing_mesh=True で既存メッシュに pivot 一致。
# ---------------------------------------------------------------------------

def _build_run_script(limit):
    """Quick Run / Full Run 共通の Python スクリプト文字列を生成する。

    Args:
        limit: int なら先頭 limit 件のみ配置 (スモークテスト)。
               None なら placements 全件を配置 (Phase 4.2)。
    """
    if limit is None:
        slice_line = "preview = placements"
        scope = "all"
    else:
        slice_line = f"preview = placements[:{limit}]"
        scope = f"first {limit}"
    return f'''
import os
import unreal
try:
    from lanelet2_traffic_light.core.api import generate_placements
    from lanelet2_traffic_light.core.profile.profile_jp import PROFILE_JP
    from lanelet2_traffic_light.core.sign_id.way_id_resolver import WayIdResolver
    from lanelet2_traffic_light.frontend_editor.editor_placer import (
        get_world_mgrs_data, place_from_specs,
    )
    from lanelet2_traffic_light.frontend_editor.bp_factory import (
        ensure_subtype_bps, ensure_group_bp,
    )
except Exception as e:
    unreal.log_error(f"lanelet2_traffic_light import failed: {{e}}")
    raise

OSM_PATH = r"{DEFAULT_OSM_PATH}"

# Phase 4.2: 全件レポートを T4Fork.odaiba 直下 (git 管理外) に出力。
# project_dir = .../CarlaUE5/Unreal/CarlaUnreal/  → 3 つ上が T4Fork.odaiba/
PROJECT_DIR = unreal.Paths.project_dir()
REPORT_PATH = os.path.abspath(os.path.join(
    PROJECT_DIR, "..", "..", "..", "lanelet2_tl_full_run_report.txt",
))

# 現在開いているレベル (.umap) のフルパスとマップ名 (basename) を取得。
# asset path /Game/X/Y/Map.Map を Content/X/Y/Map.umap に変換しつつ
# マップ名を派生 BP 命名に使う。
try:
    _world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    _asset_path = _world.get_path_name() if _world is not None else ""
    if _asset_path.startswith("/Game/"):
        _rel = _asset_path[len("/Game/"):].split(".")[0]
        MAP_FILE_PATH = os.path.abspath(os.path.join(PROJECT_DIR, "Content", _rel + ".umap"))
        MAP_NAME = _rel.rsplit("/", 1)[-1]
    else:
        MAP_FILE_PATH = _asset_path or "<unknown>"
        MAP_NAME = "Unknown"
except Exception:
    MAP_FILE_PATH = "<unknown>"
    MAP_NAME = "Unknown"

# Phase 5 ①: subtype 別派生 BP をマップに合わせて自動生成 (既存なら再利用)。
# generate_placements の bp_override に渡すことで、profile デフォルトの BP path
# を上書きする。Odaiba では既存 BP_OdaibaVehicleTL を再利用、別マップでは
# T4 サンプル BP を親に派生 BP が新規作成される。
unreal.log(f"[lanelet2_tl] ensuring subtype BPs for map '{{MAP_NAME}}'...")
BP_OVERRIDE = ensure_subtype_bps(PROFILE_JP, MAP_NAME)
for _st, _bp in BP_OVERRIDE.items():
    unreal.log(f"[lanelet2_tl]   subtype={{_st}} -> {{_bp}}")

# Phase 5 ②: TrafficLightGroup 派生 BP も map ごとに自動生成 (C++ ATrafficLightGroup を親に派生)。
# place_from_specs に group_bp_path として渡し、_place_groups が各 lanelet2 regulatory
# element を 1 Group + 1 Controller で配置 + TL を AddTrafficLight で紐付け。
unreal.log(f"[lanelet2_tl] ensuring TrafficLightGroup BP for map '{{MAP_NAME}}'...")
GROUP_BP_PATH = ensure_group_bp(MAP_NAME)
unreal.log(f"[lanelet2_tl]   group_bp -> {{GROUP_BP_PATH}}")

transformer = get_world_mgrs_data()
placements, groups, report = generate_placements(
    osm_path=OSM_PATH,
    profile=PROFILE_JP,
    sign_id_resolver=WayIdResolver(),
    transformer=transformer,
    bp_override=BP_OVERRIDE,
)
unreal.log(
    f"[lanelet2_tl] parsed TL={{report.parsed_traffic_lights}} "
    f"groups={{report.parsed_groups}} skipped={{len(report.placements_skipped)}}"
)

{slice_line}
unreal.log(f"[lanelet2_tl] placing {{len(preview)}} traffic lights ({scope})...")
preport = place_from_specs(
    preview, groups,
    save_level=False,
    snap_to_existing_mesh=True,
    snap_radius_cm=300.0,
    report_path=REPORT_PATH,
    report_metadata={{
        "osm_path"      : OSM_PATH,
        "map_file_path" : MAP_FILE_PATH,
        "map_name"      : MAP_NAME,
        "ue_project_dir": os.path.abspath(PROJECT_DIR),
        "report_path"   : REPORT_PATH,
        "group_bp_path" : GROUP_BP_PATH,
    }},
    group_bp_path=GROUP_BP_PATH,
)
unreal.log(
    f"[lanelet2_tl] result: created={{preport.created}} updated={{preport.updated}} "
    f"snap_skipped={{len(preport.snap_skipped)}} failed={{len(preport.failed)}} "
    f"unused_meshes={{len(preport.unused_existing_meshes)}} | "
    f"groups created={{preport.groups_created}} updated={{preport.groups_updated}}"
)
unreal.log(f"[lanelet2_tl] full report file: {{REPORT_PATH}}")
'''


QUICK_RUN_SCRIPT = _build_run_script(5)
FULL_RUN_SCRIPT = _build_run_script(None)


# ---------------------------------------------------------------------------
# Menu registration
# ---------------------------------------------------------------------------

def _register_menu_entries():
    """LevelEditor.MainMenu.Tools にエントリ 2 つを追加。"""
    menus = unreal.ToolMenus.get()
    main_menu = menus.find_menu("LevelEditor.MainMenu.Tools")
    if main_menu is None:
        unreal.log_warning("init_unreal: LevelEditor.MainMenu.Tools not found.")
        return

    section_name = "LaneletTrafficLight"

    # Entry 1: open the Editor Utility Widget (Phase 3.5 の成果物)
    entry_widget = unreal.ToolMenuEntry(
        name="LaneletTL_OpenWidget",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_widget.set_label("Generate Traffic Lights from lanelet2... (Widget)")
    entry_widget.set_tool_tip(
        "Open the EUW_LaneletTrafficLight widget. "
        "Requires the asset to be created first (Phase 3.5)."
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

    # Entry 2: Quick-Run (Phase 3.5 完了前のスモークテスト用)
    entry_quick = unreal.ToolMenuEntry(
        name="LaneletTL_QuickRun",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_quick.set_label("Generate Traffic Lights from lanelet2 (Quick Run, first 5)")
    entry_quick.set_tool_tip(
        "Parses the default Odaiba lanelet2 and spawns the first 5 traffic "
        "lights as a smoke test. Useful before the EUW is created."
    )
    entry_quick.set_string_command(
        type=unreal.ToolMenuStringCommandType.PYTHON,
        custom_type=unreal.Name(""),
        string=QUICK_RUN_SCRIPT.strip(),
    )
    main_menu.add_menu_entry(section_name, entry_quick)

    # Entry 3: Full-Run (Phase 4.2: 全件配置)
    # 522 件配置するため数分かかる。レベル保存は明示的に Ctrl+S が必要。
    entry_full = unreal.ToolMenuEntry(
        name="LaneletTL_FullRun",
        type=unreal.MultiBlockType.MENU_ENTRY,
    )
    entry_full.set_label("Generate Traffic Lights from lanelet2 (Full Run, all)")
    entry_full.set_tool_tip(
        "Parses the default Odaiba lanelet2 and spawns ALL parsed traffic "
        "lights (~522 in Odaiba). Takes a few minutes. Save the level "
        "manually with Ctrl+S after verifying the result."
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
