"""Build the T4 native-LED R/Y/G material M_JPVehicleLedRYG via MaterialEditingLibrary.

Editor-only (depends on `unreal`). The .uasset lives in tracked T4 plugin content;
this script reproduces it so it can be rebuilt.

Purpose: light the R/Y/G lamps of NishiShinjuku's native vehicle LED face (slot
`TrafficLightsLed`). The three lamps share one UV rect (UV masking impossible), but
they sit at distinct LOCAL positions, so the mask splits the lamp row along local Y
(the meshes are ~10x-authored; row spans local Y -631..631; thirds at +/-210 were
validated across all 21 mesh variants, whose bounding boxes are identical).

Graph: BaseColor = black; Emissive =
    region(Y <  RowB1) * RedColor    * RedIntensity
  + region(B1..B2)     * YellowColor * YellowIntensity
  + region(Y >  RowB2) * GreenColor  * GreenIntensity
(Japanese order as viewed from the front: left=green, mid=yellow, right=red.)
Intensities default 0 (unlit); the vehicle BP drives them per set_state (the same
parameter-driving pattern as the pedestrian SceneFigure BP). Solid lamp discs (no
dot texture) -- validated visually; LED-dot polish can come later.

Run from the editor Python console:
    exec(open('<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/vehicle_led_material_setup.py').read())
"""
import unreal

MATERIAL_DIR = "/T4/TrafficLightSample/VehicleTrafficLight"
MATERIAL_NAME = "M_JPVehicleLedRYG"
MATERIAL_PATH = "%s/%s" % (MATERIAL_DIR, MATERIAL_NAME)

DEFAULT_ROW_B1 = -210.0   # local-Y boundary red|yellow (validated 2026-06-03)
DEFAULT_ROW_B2 = 210.0    # local-Y boundary yellow|green
DEFAULT_SHARPNESS = 0.1   # step transition: 1/local-units (~1 cm world at 0.1 parent scale)
DEFAULT_INTENSITY = 0.0   # unlit until the BP drives the per-state intensity

mel = unreal.MaterialEditingLibrary
MP_BASE = unreal.MaterialProperty.MP_BASE_COLOR
MP_EMIS = unreal.MaterialProperty.MP_EMISSIVE_COLOR
_n = [0]


def _expr(mat, cls, y=0):
    _n[0] += 1
    return mel.create_material_expression(mat, cls, -1500 + _n[0] * 150, y)


def _con(a, ao, b, bi):
    if not mel.connect_material_expressions(a, ao, b, bi):
        unreal.log_error("connect FAIL %s[%s]->%s[%s]"
                         % (a.get_class().get_name(), ao, b.get_class().get_name(), bi))


def _scalar(mat, name, default, y):
    p = _expr(mat, unreal.MaterialExpressionScalarParameter, y)
    p.set_editor_property("parameter_name", name)
    p.set_editor_property("default_value", default)
    return p


def _vector(mat, name, color, y):
    p = _expr(mat, unreal.MaterialExpressionVectorParameter, y)
    p.set_editor_property("parameter_name", name)
    p.set_editor_property("default_value", color)
    return p


def _step(mat, coord, boundary, sharp):
    """clamp((coord - boundary) * sharp, 0, 1)"""
    sub = _expr(mat, unreal.MaterialExpressionSubtract, 0)
    _con(coord, "", sub, "A")
    _con(boundary, "", sub, "B")
    m = _expr(mat, unreal.MaterialExpressionMultiply, 0)
    _con(sub, "", m, "A")
    _con(sharp, "", m, "B")
    c = _expr(mat, unreal.MaterialExpressionClamp, 0)
    _con(m, "", c, "")
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

    # local-Y row coordinate (the lamp row axis)
    lp = _expr(mat, unreal.MaterialExpressionLocalPosition, 0)
    ymask = _expr(mat, unreal.MaterialExpressionComponentMask, 0)
    ymask.set_editor_property("r", False)
    ymask.set_editor_property("g", True)
    ymask.set_editor_property("b", False)
    ymask.set_editor_property("a", False)
    _con(lp, "", ymask, "")

    b1 = _scalar(mat, "RowB1", DEFAULT_ROW_B1, 200)
    b2 = _scalar(mat, "RowB2", DEFAULT_ROW_B2, 300)
    sharp = _scalar(mat, "Sharpness", DEFAULT_SHARPNESS, 400)
    s1 = _step(mat, ymask, b1, sharp)
    s2 = _step(mat, ymask, b2, sharp)
    one = _expr(mat, unreal.MaterialExpressionConstant, 500)
    one.set_editor_property("r", 1.0)
    r_red = _expr(mat, unreal.MaterialExpressionSubtract, 0)      # 1 - s1   (Y < B1)
    _con(one, "", r_red, "A")
    _con(s1, "", r_red, "B")
    r_yellow = _expr(mat, unreal.MaterialExpressionSubtract, 0)   # s1 - s2  (B1..B2)
    _con(s1, "", r_yellow, "A")
    _con(s2, "", r_yellow, "B")
    # r_green = s2 (Y > B2)

    specs = [
        (r_red, "RedColor", unreal.LinearColor(1, 0, 0, 1), "RedIntensity"),
        (r_yellow, "YellowColor", unreal.LinearColor(1, 0.8, 0, 1), "YellowIntensity"),
        (s2, "GreenColor", unreal.LinearColor(0, 1, 0, 1), "GreenIntensity"),
    ]
    terms = []
    yoff = 600
    for region, cname, cval, iname in specs:
        col = _vector(mat, cname, cval, yoff)
        inten = _scalar(mat, iname, DEFAULT_INTENSITY, yoff + 80)
        m1 = _expr(mat, unreal.MaterialExpressionMultiply, yoff)
        _con(region, "", m1, "A")
        _con(col, "", m1, "B")
        m2 = _expr(mat, unreal.MaterialExpressionMultiply, yoff)
        _con(m1, "", m2, "A")
        _con(inten, "", m2, "B")
        terms.append(m2)
        yoff += 200

    add1 = _expr(mat, unreal.MaterialExpressionAdd, 700)
    _con(terms[0], "", add1, "A")
    _con(terms[1], "", add1, "B")
    add2 = _expr(mat, unreal.MaterialExpressionAdd, 700)
    _con(add1, "", add2, "A")
    _con(terms[2], "", add2, "B")
    black = _expr(mat, unreal.MaterialExpressionConstant3Vector, 850)
    black.set_editor_property("constant", unreal.LinearColor(0, 0, 0, 1))
    mel.connect_material_property(black, "", MP_BASE)
    mel.connect_material_property(add2, "", MP_EMIS)
    mel.layout_material_expressions(mat)
    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    unreal.log("%s built" % MATERIAL_NAME)


build()
