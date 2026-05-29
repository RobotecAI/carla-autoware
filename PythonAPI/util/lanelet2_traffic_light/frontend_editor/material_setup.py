"""Build the T4 pedestrian figure-emissive material via MaterialEditingLibrary.

Editor-only (depends on `unreal`). Reproduces M_PedestrianFigure so the asset
can be rebuilt; the .uasset itself lives in tracked T4 plugin content.

Graph: BaseColor=black; Emissive = topRegion*Red*StopIntensity*HDR
       + botRegion*Green*WalkIntensity*HDR; mask=Power(Tex.G,8);
       region split by UV.V vs SplitV. Params: StopIntensity/WalkIntensity/
       SplitV/HDRIntensity (scalar), RedColor/GreenColor (vector).

Run from the editor Output Log:
    py "<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/material_setup.py"
"""
import unreal

MATERIAL_DIR = "/T4/Lanelet2TrafficLight"
MATERIAL_NAME = "M_PedestrianFigure"
MATERIAL_PATH = "%s/%s" % (MATERIAL_DIR, MATERIAL_NAME)
TEXTURE_PATH = "/T4/Lanelet2TrafficLight/T_PedestrianFigures"

mel = unreal.MaterialEditingLibrary
MP_BASE = unreal.MaterialProperty.MP_BASE_COLOR
MP_EMIS = unreal.MaterialProperty.MP_EMISSIVE_COLOR

_n = [0]


def _expr(mat, cls, y=0):
    _n[0] += 1
    return mel.create_material_expression(mat, cls, -1400 + _n[0] * 170, y)


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
    tex = unreal.load_asset(TEXTURE_PATH)
    if mat is None or tex is None:
        unreal.log_error("build: load failed (mat=%s tex=%s)" % (mat, tex))
        return
    try:
        mel.delete_all_material_expressions(mat)
    except Exception as e:
        unreal.log_warning("delete_all: %s" % e)
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)

    ts = _expr(mat, unreal.MaterialExpressionTextureSample, -400)
    ts.set_editor_property("texture", tex)
    ts.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)

    p_stop = _expr(mat, unreal.MaterialExpressionScalarParameter, -300)
    p_stop.set_editor_property("parameter_name", "StopIntensity")
    p_stop.set_editor_property("default_value", 0.0)
    p_walk = _expr(mat, unreal.MaterialExpressionScalarParameter, -200)
    p_walk.set_editor_property("parameter_name", "WalkIntensity")
    p_walk.set_editor_property("default_value", 0.0)
    p_split = _expr(mat, unreal.MaterialExpressionScalarParameter, -100)
    p_split.set_editor_property("parameter_name", "SplitV")
    p_split.set_editor_property("default_value", 0.5)
    p_hdr = _expr(mat, unreal.MaterialExpressionScalarParameter, 0)
    p_hdr.set_editor_property("parameter_name", "HDRIntensity")
    p_hdr.set_editor_property("default_value", 30.0)
    p_red = _expr(mat, unreal.MaterialExpressionVectorParameter, 100)
    p_red.set_editor_property("parameter_name", "RedColor")
    p_red.set_editor_property("default_value", unreal.LinearColor(1, 0, 0, 1))
    p_green = _expr(mat, unreal.MaterialExpressionVectorParameter, 200)
    p_green.set_editor_property("parameter_name", "GreenColor")
    p_green.set_editor_property("default_value", unreal.LinearColor(0, 1, 0, 1))

    lum = _expr(mat, unreal.MaterialExpressionPower, 300)
    lum.set_editor_property("const_exponent", 8.0)
    _con(ts, "G", lum, "Base")
    uv = _expr(mat, unreal.MaterialExpressionTextureCoordinate, 400)
    uv.set_editor_property("coordinate_index", 0)
    vmask = _expr(mat, unreal.MaterialExpressionComponentMask, 500)
    vmask.set_editor_property("r", False)
    vmask.set_editor_property("g", True)
    vmask.set_editor_property("b", False)
    vmask.set_editor_property("a", False)
    _con(uv, "", vmask, "")
    sub = _expr(mat, unreal.MaterialExpressionSubtract, 600)
    _con(p_split, "", sub, "A")
    _con(vmask, "", sub, "B")
    sharp = _expr(mat, unreal.MaterialExpressionConstant, 650)
    sharp.set_editor_property("r", 1000.0)
    scaled = _expr(mat, unreal.MaterialExpressionMultiply, 700)
    _con(sub, "", scaled, "A")
    _con(sharp, "", scaled, "B")
    top = _expr(mat, unreal.MaterialExpressionClamp, 800)  # defaults min0/max1
    _con(scaled, "", top, "")
    bot = _expr(mat, unreal.MaterialExpressionOneMinus, 900)
    _con(top, "", bot, "")
    stop_m = _expr(mat, unreal.MaterialExpressionMultiply, 1000)
    _con(lum, "", stop_m, "A")
    _con(top, "", stop_m, "B")
    walk_m = _expr(mat, unreal.MaterialExpressionMultiply, 1100)
    _con(lum, "", walk_m, "A")
    _con(bot, "", walk_m, "B")
    red_i = _expr(mat, unreal.MaterialExpressionMultiply, 1200)
    _con(p_red, "", red_i, "A")
    _con(p_stop, "", red_i, "B")
    st0 = _expr(mat, unreal.MaterialExpressionMultiply, 1300)
    _con(stop_m, "", st0, "A")
    _con(red_i, "", st0, "B")
    st = _expr(mat, unreal.MaterialExpressionMultiply, 1400)
    _con(st0, "", st, "A")
    _con(p_hdr, "", st, "B")
    grn_i = _expr(mat, unreal.MaterialExpressionMultiply, 1500)
    _con(p_green, "", grn_i, "A")
    _con(p_walk, "", grn_i, "B")
    wt0 = _expr(mat, unreal.MaterialExpressionMultiply, 1600)
    _con(walk_m, "", wt0, "A")
    _con(grn_i, "", wt0, "B")
    wt = _expr(mat, unreal.MaterialExpressionMultiply, 1700)
    _con(wt0, "", wt, "A")
    _con(p_hdr, "", wt, "B")
    emis = _expr(mat, unreal.MaterialExpressionAdd, 1800)
    _con(st, "", emis, "A")
    _con(wt, "", emis, "B")
    black = _expr(mat, unreal.MaterialExpressionConstant3Vector, -350)
    black.set_editor_property("constant", unreal.LinearColor(0, 0, 0, 1))

    mel.connect_material_property(black, "", MP_BASE)
    mel.connect_material_property(emis, "", MP_EMIS)
    mel.layout_material_expressions(mat)
    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    unreal.log("%s built" % MATERIAL_NAME)


build()
