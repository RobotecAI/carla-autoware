# lanelet2_traffic_light

Python package that parses a lanelet2 `.osm` map, generates traffic light placement data, and spawns traffic light actors in a CARLA/UE5 level with SignIDs assigned. Validated on the Odaiba map (AWSIM-derived).

## Package Layout

```
PythonAPI/util/lanelet2_traffic_light/
├── corelib/                        # Editor-independent logic (pytest target)
│   ├── parser/lanelet2_parser.py
│   ├── geometry/mgrs_transform.py
│   ├── geometry/pose_estimator.py
│   ├── profile/{profile_base,profile_jp}.py
│   ├── sign_id/way_id_resolver.py
│   ├── ir/traffic_light_ir.py
│   └── api.py                      # generate_placements()
├── frontend_editor/
│   └── editor_placer.py            # unreal module dependency isolated here
├── tests/                          # 80 tests PASS
└── README.md
```

## Prerequisites

Set the environment variable `LANELET2_OSM_PATH` to the path of your lanelet2 OSM file **before** launching the UE5 Editor. The value is used as the default path in the EUW input field and is also read by the command-line entry point.

```bash
export LANELET2_OSM_PATH=/path/to/lanelet2_map.osm
```

## Running from the UE5 Editor

The Tools menu exposes a single entry that opens the Editor Utility Widget (EUW).

1. Open the target level (e.g. `Odaiba.umap`).
2. **Tools** menu → **Generate Traffic Lights from lanelet2...**.
3. The EUW opens. The OSM path input field is pre-filled with the value of `LANELET2_OSM_PATH` (empty if the variable is unset).
4. Confirm or edit the OSM path, then click **Full Run**.
5. All traffic lights from the OSM file are spawned and SignIDs are assigned. If the path is unset or the file does not exist, an error dialog is shown.

> **Note:** `BP_OdaibaVehicleTL` and `BP_OdaibaPedestrianTL` must exist in the level for actors to be spawned.

## Running from Python (core library only)

```python
from lanelet2_traffic_light.corelib.api import generate_placements
from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer

# Odaiba offset values (validated in Phase 0)
transformer = MgrsTransformer(
    offset_x_m=92008.5, offset_y_m=45335.1, offset_z_m=0.0,
    x_sign=+1, y_sign=-1,
)

placements, groups, report = generate_placements(
    osm_path="/path/to/lanelet2_map.osm",
    profile=PROFILE_JP,
    sign_id_resolver=WayIdResolver(),
    transformer=transformer,
)
print(report)
```

## Coordinate Formula (Odaiba)

```
Unreal_X (cm) =  (local_x_m - offset_x_m) * 100
Unreal_Y (cm) = -(local_y_m - offset_y_m) * 100   # Y axis inverted
Unreal_Z (cm) =  pole_height_m * 100               # default_pole_height_m = 12.3
```

`MgrsOffsetPosition` units are **meters** (corrected from an earlier centimeter assumption).
See `docs/phase0_validation.md` for derivation details.

## Freeze Workaround

`control_traffic_light_by_sign_id.py --freeze` alone does not halt the cycle because
`ATrafficLightManager::TrafficGroups[]` is empty at init time, making `freeze_all_traffic_lights` a no-op.

**Workaround:** override cycle times with large values so the state effectively never advances.

```bash
# Example: hold sign_id=6621 in red state
python3 control_traffic_light_by_sign_id.py --id 6621 --state red --freeze \
    --green-time 99999 --yellow-time 99999 --red-time 99999 --all-in-group
```

`--all-in-group` applies the cycle times to all traffic lights in the same group.

> **Note (Phase 7):** Pedestrian signal lighting is not yet supported. Only vehicle traffic lights are lit correctly.

## Running Tests

```bash
cd PythonAPI/util/lanelet2_traffic_light
PYTHONPATH=.. python3 -m pytest tests/ -v
```

Expected result: **80 passed**.

Integration tests that require the actual Odaiba lanelet2 file
(`/mnt/dsk0/wk0/CARLA/autoware_map/odaiba_autoware_map_2025_01_16/lanelet2_map.osm`)
are automatically skipped in CI environments where the file is absent.

## Known Limitations

- Arrow signals are not supported (requires `light_bulbs` interpretation and `ETrafficLightState` extension).
- Pedestrian signals: only `red_green` subtype is supported.
- Pedestrian signal lighting is not yet implemented (vehicle traffic lights only).
- Other country profiles are not implemented (add `profile/profile_us.py` etc. to extend).
- `frontend_client/` (PythonAPI runtime spawn) and `frontend_json/` (intermediate JSON) are not implemented.
- Roll/Pitch values (`-90` / `+85.4`) for traffic light BPs must be baked into the StaticMeshComponent inside each BP.
- `BP_TrafficLightGroup` path (`BP_TRAFFIC_LIGHT_GROUP_PATH` in `frontend_editor/editor_placer.py:18`) must be verified against the actual Blueprint asset.

## Related Documents

- Design spec: `docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md`
- Implementation plan: `docs/superpowers/plans/2026-05-20-lanelet2-traffic-light.md`
- Phase 0 validation notes: `docs/phase0_validation.md` (inside package)
- Knowledge notes: `work.knowledge/` (workspace root)
