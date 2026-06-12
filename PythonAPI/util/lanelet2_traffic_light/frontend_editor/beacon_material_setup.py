"""Build M_JPFlashingBeaconLit via MaterialEditingLibrary.

Emissive square wave driven by the material Time node -- the blink runs on
the GPU with zero Tick/BP cost:

    local_y     = TransformPosition(AbsoluteWorldPosition, World -> Local).Y
    lamp_select = 0.5 * saturate((local_y - YSplit) * 1000)
    on          = 1 - floor(frac(Time/Period + Phase + lamp_select) + 0.5)
    Emissive    = on * BeaconColor * Intensity

The two lamps of one beacon mesh share a single Orange_Light element but sit
apart on the local Y axis (2026-06-12 audit: y_gap 13.6-17.7 cm across all 32
Odaiba beacons, while UV could not be proven to separate the lamps), so the
mask keys on pre-skinned local position. YSplit varies per mesh family; the
apply script bakes it into per-family material instances (MIC), and the
runtime MIDs created by BP_FlashingBeaconManager inherit it.

Params: Period (full cycle seconds, default 1.2), Phase (0-1, randomized per
beacon by the manager), YSplit (local-Y lamp boundary), BeaconColor,
Intensity.

Run from the editor Output Log:
    py "<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/beacon_material_setup.py"
"""
import unreal

MATERIAL_DIR = "/T4/TrafficLightSample/FlashingBeacon"
MATERIAL_NAME = "M_JPFlashingBeaconLit"
MATERIAL_PATH = "%s/%s" % (MATERIAL_DIR, MATERIAL_NAME)

DEFAULT_PERIOD = 1.2        # full cycle: lamp A 0.6s, lamp B 0.6s
DEFAULT_INTENSITY = 20.0    # tuned in PIE with bloom (arrow ON precedent = 20)
DEFAULT_COLOR = unreal.LinearColor(1.0, 0.25, 0.02, 1.0)  # amber

mel = unreal.MaterialEditingLibrary
MP_BASE = unreal.MaterialProperty.MP_BASE_COLOR
MP_EMIS = unreal.MaterialProperty.MP_EMISSIVE_COLOR


def _expr(mat, cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)


def _con(a, ao, b, bi):
    if not mel.connect_material_expressions(a, ao, b, bi):
        unreal.log_error("connect FAIL %s[%s]->%s[%s]"
                         % (a.get_class().get_name(), ao, b.get_class().get_name(), bi))

def _scalar(mat, name, default, x, y):
    p = _expr(mat, unreal.MaterialExpressionScalarParameter, x, y)
    p.set_editor_property("parameter_name", name)
    p.set_editor_property("default_value", default)
    return p


def _const(mat, value, x, y):
    c = _expr(mat, unreal.MaterialExpressionConstant, x, y)
    c.set_editor_property("r", value)
    return c


def build():
    mat = unreal.load_asset(MATERIAL_PATH)
    if mat is None:
        at = unreal.AssetToolsHelpers.get_asset_tools()
        mat = at.create_asset(MATERIAL_NAME, MATERIAL_DIR,
                              unreal.Material, unreal.MaterialFactoryNew())
    if mat is None:
        unreal.log_error("build: could not create/load %s" % MATERIAL_PATH)
        return
    try:
        mel.delete_all_material_expressions(mat)
    except Exception as e:
        unreal.log_warning("delete_all: %s" % e)
    # delete_all proved unreliable while the material editor held the asset
    # open (2026-06-12: four builds accumulated dangling nodes); sweep any
    # survivors one by one.
    try:
        leftovers = list(mat.get_editor_property("expressions"))
        for ex in leftovers:
            mel.delete_material_expression(mat, ex)
        if leftovers:
            unreal.log("[beacon_material] swept %d leftover expressions"
                       % len(leftovers))
    except Exception as e:
        unreal.log_warning("leftover sweep: %s" % e)
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)

    period = _scalar(mat, "Period", DEFAULT_PERIOD, -1200, 0)
    phase = _scalar(mat, "Phase", 0.0, -1200, 100)
    ysplit = _scalar(mat, "YSplit", 0.0, -1200, 300)

    # lamp_select = 0.5 * saturate((LocalPos.Y - YSplit) * 1000): branchless
    # upper/lower mask. The lamps sit >= 6.8 cm from the split, so the steep
    # saturate snaps to exactly 0 or 0.5 per lamp. If-node pins and the
    # deprecated PreSkinnedPosition proved unreliable to wire from python
    # (2026-06-12), hence math-only nodes and a World->Local transform.
    wpos = _expr(mat, unreal.MaterialExpressionWorldPosition, -1350, 400)
    tpos = _expr(mat, unreal.MaterialExpressionTransformPosition, -1200, 400)
    # Enum class name differs across engine versions.
    pos_enum = None
    for enum_name in ("MaterialPositionTransformSource",
                      "MaterialExpressionTransformPosSource"):
        pos_enum = getattr(unreal, enum_name, None)
        if pos_enum is not None:
            break
    if pos_enum is None:
        unreal.log_error("build: transform-position enum class not found")
        return
    tpos.set_editor_property(
        "transform_source_type", pos_enum.TRANSFORMPOSSOURCE_WORLD)
    tpos.set_editor_property(
        "transform_type", pos_enum.TRANSFORMPOSSOURCE_LOCAL)
    _con(wpos, "", tpos, "")
    ymask = _expr(mat, unreal.MaterialExpressionComponentMask, -1050, 400)
    ymask.set_editor_property("r", False)
    ymask.set_editor_property("g", True)
    ymask.set_editor_property("b", False)
    ymask.set_editor_property("a", False)
    _con(tpos, "", ymask, "")
    ydelta = _expr(mat, unreal.MaterialExpressionSubtract, -900, 400)
    _con(ymask, "", ydelta, "A")
    _con(ysplit, "", ydelta, "B")
    steep = _const(mat, 1000.0, -900, 500)
    ysteep = _expr(mat, unreal.MaterialExpressionMultiply, -800, 400)
    _con(ydelta, "", ysteep, "A")
    _con(steep, "", ysteep, "B")
    ysat = _expr(mat, unreal.MaterialExpressionSaturate, -700, 400)
    _con(ysteep, "", ysat, "")
    half_a = _const(mat, 0.5, -700, 500)
    lamp_select = _expr(mat, unreal.MaterialExpressionMultiply, -600, 400)
    _con(ysat, "", lamp_select, "A")
    _con(half_a, "", lamp_select, "B")

    # frac(Time/Period + Phase + lamp_select)
    t = _expr(mat, unreal.MaterialExpressionTime, -1050, 0)
    div = _expr(mat, unreal.MaterialExpressionDivide, -900, 0)
    _con(t, "", div, "A")
    _con(period, "", div, "B")
    add1 = _expr(mat, unreal.MaterialExpressionAdd, -750, 0)
    _con(div, "", add1, "A")
    _con(phase, "", add1, "B")
    add2 = _expr(mat, unreal.MaterialExpressionAdd, -600, 0)
    _con(add1, "", add2, "A")
    _con(lamp_select, "", add2, "B")
    frac = _expr(mat, unreal.MaterialExpressionFrac, -450, 0)
    _con(add2, "", frac, "")

    # on = 1 - floor(frac + 0.5): 1 while frac < 0.5, else 0 (branchless).
    half_b = _const(mat, 0.5, -450, 100)
    shift = _expr(mat, unreal.MaterialExpressionAdd, -400, 0)
    _con(frac, "", shift, "A")
    _con(half_b, "", shift, "B")
    fl = _expr(mat, unreal.MaterialExpressionFloor, -350, 0)
    _con(shift, "", fl, "")
    on = _expr(mat, unreal.MaterialExpressionOneMinus, -300, 0)
    _con(fl, "", on, "")

    color = _expr(mat, unreal.MaterialExpressionVectorParameter, -300, 250)
    color.set_editor_property("parameter_name", "BeaconColor")
    color.set_editor_property("default_value", DEFAULT_COLOR)
    inten = _scalar(mat, "Intensity", DEFAULT_INTENSITY, -300, 450)

    m1 = _expr(mat, unreal.MaterialExpressionMultiply, -150, 0)
    _con(on, "", m1, "A")
    _con(color, "", m1, "B")
    m2 = _expr(mat, unreal.MaterialExpressionMultiply, -50, 0)
    _con(m1, "", m2, "A")
    _con(inten, "", m2, "B")

    dark = _expr(mat, unreal.MaterialExpressionConstant3Vector, -300, 550)
    dark.set_editor_property("constant", unreal.LinearColor(0.05, 0.03, 0.01, 1))

    mel.connect_material_property(dark, "", MP_BASE)
    mel.connect_material_property(m2, "", MP_EMIS)
    mel.layout_material_expressions(mat)
    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    unreal.log("[beacon_material] %s built" % MATERIAL_NAME)


build()
