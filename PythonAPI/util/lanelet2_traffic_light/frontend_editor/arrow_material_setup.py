"""Build the T4 green-arrow emissive material M_JPArrowLit via MaterialEditingLibrary.

Editor-only (depends on `unreal`). The .uasset lives in tracked T4 plugin content;
this script reproduces it so it can be rebuilt.

Graph: BaseColor = black; Emissive = Clamp((ArrowTex.RGB - BlackLevel)/(WhiteLevel -
       BlackLevel), 0, 1) * TintColor * Intensity. The black/white-level remap sends the
       dim (non-arrow) background to true black while rescaling the dim arrow LED dots up
       to full range, so only the dots emit (the arrow texture's dots and background are
       both low-valued; a plain subtract crushes the dots too — Inc 0 de-risk). Params:
       ArrowTex (Texture2D, set per-direction at runtime), TintColor (vector, white),
       Intensity (scalar), BlackLevel (scalar), WhiteLevel (scalar).

Run from the editor Output Log (after the T_JPArrow_* textures exist in T4):
    py "<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/arrow_material_setup.py"
"""
import unreal

MATERIAL_DIR = "/T4/TrafficLightSample/VehicleTrafficLight"
MATERIAL_NAME = "M_JPArrowLit"
MATERIAL_PATH = "%s/%s" % (MATERIAL_DIR, MATERIAL_NAME)
# Default texture so the material compiles/previews; the real per-direction texture is
# set at runtime via the ArrowTex parameter (apply_arrow_lighting). T4 = self-contained.
DEFAULT_ARROW_TEX = "%s/T_JPArrow_Right" % MATERIAL_DIR

# De-risk defaults; final values tuned in PIE (bloom on). The arrow LED dots are dim
# in the source texture, so WhiteLevel sits just above the dot value to rescale them up.
DEFAULT_INTENSITY = 10.0
DEFAULT_BLACK_LEVEL = 0.04
DEFAULT_WHITE_LEVEL = 0.12

mel = unreal.MaterialEditingLibrary
MP_BASE = unreal.MaterialProperty.MP_BASE_COLOR
MP_EMIS = unreal.MaterialProperty.MP_EMISSIVE_COLOR


def _expr(mat, cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)


def _con(a, ao, b, bi):
    if not mel.connect_material_expressions(a, ao, b, bi):
        unreal.log_error("connect FAIL %s[%s]->%s[%s]"
                         % (a.get_class().get_name(), ao, b.get_class().get_name(), bi))


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

    tp = _expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, 0)
    tp.set_editor_property("parameter_name", "ArrowTex")
    default_tex = unreal.load_asset(DEFAULT_ARROW_TEX)
    if default_tex is not None:
        tp.set_editor_property("texture", default_tex)
    else:
        unreal.log_warning("build: default texture %s not found "
                           "(copy T_JPArrow_* to T4 first; param still usable at runtime)"
                           % DEFAULT_ARROW_TEX)

    tint = _expr(mat, unreal.MaterialExpressionVectorParameter, -600, 200)
    tint.set_editor_property("parameter_name", "TintColor")
    tint.set_editor_property("default_value", unreal.LinearColor(1.0, 1.0, 1.0, 1.0))

    inten = _expr(mat, unreal.MaterialExpressionScalarParameter, -600, 350)
    inten.set_editor_property("parameter_name", "Intensity")
    inten.set_editor_property("default_value", DEFAULT_INTENSITY)

    blk = _expr(mat, unreal.MaterialExpressionScalarParameter, -600, 450)
    blk.set_editor_property("parameter_name", "BlackLevel")
    blk.set_editor_property("default_value", DEFAULT_BLACK_LEVEL)
    wht = _expr(mat, unreal.MaterialExpressionScalarParameter, -600, 550)
    wht.set_editor_property("parameter_name", "WhiteLevel")
    wht.set_editor_property("default_value", DEFAULT_WHITE_LEVEL)

    # Levels remap: Clamp((ArrowTex.RGB - BlackLevel)/(WhiteLevel - BlackLevel), 0, 1).
    # Background (< BlackLevel) -> 0; dim arrow dots (~WhiteLevel) -> ~1 (rescaled up).
    rng = _expr(mat, unreal.MaterialExpressionSubtract, -450, 500)  # WhiteLevel - BlackLevel
    _con(wht, "", rng, "A")
    _con(blk, "", rng, "B")
    num = _expr(mat, unreal.MaterialExpressionSubtract, -450, 0)    # Tex - BlackLevel
    _con(tp, "RGB", num, "A")
    _con(blk, "", num, "B")
    div = _expr(mat, unreal.MaterialExpressionDivide, -350, 0)
    _con(num, "", div, "A")
    _con(rng, "", div, "B")
    clamp = _expr(mat, unreal.MaterialExpressionClamp, -300, 0)  # defaults min 0 / max 1
    _con(div, "", clamp, "")
    m1 = _expr(mat, unreal.MaterialExpressionMultiply, -200, 0)
    _con(clamp, "", m1, "A")
    _con(tint, "", m1, "B")
    m2 = _expr(mat, unreal.MaterialExpressionMultiply, -100, 0)
    _con(m1, "", m2, "A")
    _con(inten, "", m2, "B")

    black = _expr(mat, unreal.MaterialExpressionConstant3Vector, -600, 600)
    black.set_editor_property("constant", unreal.LinearColor(0, 0, 0, 1))

    mel.connect_material_property(black, "", MP_BASE)
    mel.connect_material_property(m2, "", MP_EMIS)
    mel.layout_material_expressions(mat)
    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    unreal.log("%s built" % MATERIAL_NAME)


build()
