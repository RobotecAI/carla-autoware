"""Build M_JPFlashingBeaconLit via MaterialEditingLibrary.

Emissive square wave driven by the material Time node -- the blink runs on
the GPU with zero Tick/BP cost:

    lamp_select = If(PreSkinnedLocalPosition.Y < YSplit, 0.0, 0.5)
    on          = If(frac(Time/Period + Phase + lamp_select) < 0.5, 1, 0)
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


# The If node exposes its comparison pins under display names ("A > B") in
# some engine versions and property names ("AGreaterThanB") in others; try
# all known spellings before reporting failure.
_IF_PIN_ALIASES = {
    "less": ("A < B", "ALessThanB"),
    "greater": ("A > B", "AGreaterThanB"),
    "equal": ("A == B", "AEqualsB"),
}


def _con_if(a, ao, if_node, pin_kind):
    for name in _IF_PIN_ALIASES[pin_kind]:
        if mel.connect_material_expressions(a, ao, if_node, name):
            return
    unreal.log_error("connect FAIL %s[%s]->If[%s] (all aliases)"
                     % (a.get_class().get_name(), ao, pin_kind))


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
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)

    period = _scalar(mat, "Period", DEFAULT_PERIOD, -1200, 0)
    phase = _scalar(mat, "Phase", 0.0, -1200, 100)
    ysplit = _scalar(mat, "YSplit", 0.0, -1200, 300)

    # lamp_select = If(LocalPos.Y < YSplit, 0.0, 0.5)
    pre = _expr(mat, unreal.MaterialExpressionPreSkinnedPosition, -1200, 400)
    ymask = _expr(mat, unreal.MaterialExpressionComponentMask, -1050, 400)
    ymask.set_editor_property("r", False)
    ymask.set_editor_property("g", True)
    ymask.set_editor_property("b", False)
    ymask.set_editor_property("a", False)
    _con(pre, "", ymask, "")
    # Pre-skinned position exists only in the vertex shader; interpolate the
    # masked Y down to the pixel shader. Lens triangles never span both lamps
    # (13-17 cm gap between the discs), so the interpolated value is constant
    # within each lamp and stays a clean mask.
    interp = _expr(mat, unreal.MaterialExpressionVertexInterpolator, -975, 400)
    _con(ymask, "", interp, "")
    zero_a = _const(mat, 0.0, -1050, 500)
    half_a = _const(mat, 0.5, -1050, 550)
    lamp_select = _expr(mat, unreal.MaterialExpressionIf, -900, 400)
    _con(interp, "", lamp_select, "A")
    _con(ysplit, "", lamp_select, "B")
    _con_if(zero_a, "", lamp_select, "less")
    _con_if(half_a, "", lamp_select, "greater")
    _con_if(half_a, "", lamp_select, "equal")

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

    # on = If(frac < 0.5, 1, 0)
    half_b = _const(mat, 0.5, -450, 100)
    one_b = _const(mat, 1.0, -450, 150)
    zero_b = _const(mat, 0.0, -450, 200)
    on = _expr(mat, unreal.MaterialExpressionIf, -300, 0)
    _con(frac, "", on, "A")
    _con(half_b, "", on, "B")
    _con_if(one_b, "", on, "less")
    _con_if(zero_b, "", on, "greater")
    _con_if(zero_b, "", on, "equal")

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
