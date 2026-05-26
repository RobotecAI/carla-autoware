# lanelet2_traffic_light

lanelet2 (.osm) を入力に CARLA レベルへ信号機アクターを自動配置し SignID を付与する Python パッケージ。Odaiba (AWSIM 由来) マップで実機検証済み。

## 状態

- ✅ Phase 0: 座標式実地確定 (`docs/phase0_validation.md`)
- ✅ Phase 1: コアパッケージ + リファクタリング (pytest 28 件 PASS)
- ✅ Phase 3.1-3.4: frontend_editor.editor_placer 実装
- ✅ Phase 3.6: Tools メニュー登録 (`init_unreal.py`)
- ⏳ Phase 2: BP_OdaibaVehicleTL / BP_OdaibaPedestrianTL 作成 (Editor 手作業)
- ⏳ Phase 3.5: EUW_LaneletTrafficLight.uasset 作成 (Editor 手作業)
- ⏳ Phase 4: 実地配置と動作確認

## 構成

```
PythonAPI/util/lanelet2_traffic_light/
├── core/                          # Editor 非依存 (pytest 対象)
│   ├── parser/lanelet2_parser.py
│   ├── geometry/mgrs_transform.py
│   ├── geometry/pose_estimator.py
│   ├── profile/{profile_base,profile_jp}.py
│   ├── sign_id/way_id_resolver.py
│   ├── ir/traffic_light_ir.py
│   └── api.py                     # generate_placements()
├── frontend_editor/
│   └── editor_placer.py           # unreal モジュール依存はここのみ
├── docs/phase0_validation.md      # Phase 0 検証メモ
├── tests/                         # 28 件 PASS
└── README.md
```

## 使い方

### A. Editor から (Phase 3.5 後)

1. Odaiba.umap を開く
2. Tools → 「Generate Traffic Lights from lanelet2... (Widget)」
3. 設定ダイアログで .osm パス等を指定 → 実行

### B. Editor から スモークテスト (Phase 3.5 前でも可)

1. Odaiba.umap を開く
2. Tools → 「Generate Traffic Lights from lanelet2 (Quick Run, first 5)」
3. 既定 Odaiba lanelet2 から先頭 5 個を spawn

実行には `BP_OdaibaVehicleTL` / `BP_OdaibaPedestrianTL` (Phase 2 で作成予定) が存在する必要あり。

### C. Python から (コア単体)

```python
from lanelet2_traffic_light.corelib.api import generate_placements
from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer

# Odaiba 実値 (Phase 0 で確定)
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

## 確定した座標式 (Odaiba)

```
Unreal_X (cm) =  (local_x_m - offset_x_m) * 100
Unreal_Y (cm) = -(local_y_m - offset_y_m) * 100   # Y のみ反転
Unreal_Z (cm) =  pole_height_m * 100              # default_pole_height_m=12.3
```

`MgrsOffsetPosition` の単位は **m** (cm 想定だった旧設計から訂正)。
詳細は `docs/phase0_validation.md`。

## テスト

```bash
cd PythonAPI/util/lanelet2_traffic_light
PYTHONPATH=.. python3 -m pytest tests/ -v
```

実 Odaiba lanelet2 (`/mnt/dsk0/wk0/CARLA/autoware_map/odaiba_autoware_map_2025_01_16/lanelet2_map.osm`) が存在する環境では統合テストも自動実行 (CI では自動 skip)。

## 既知の制限事項 / 残課題

- 矢印信号には未対応 (`light_bulbs` の解釈と `ETrafficLightState` の拡張が必要)
- 歩行者信号は `red_green` subtype のみ対応
- 他国プロファイルは未実装 (`profile/profile_us.py` 等を追加すれば拡張可能)
- `frontend_client/` (PythonAPI ランタイム spawn) と `frontend_json/` (中間 JSON) は未実装
- 信号機 BP の Roll/Pitch (`-90` / `+85.4`) は Phase 2 で BP 内 StaticMeshComponent に baked-in する必要あり
- `BP_TrafficLightGroup` の Blueprint パス (`BP_TRAFFIC_LIGHT_GROUP_PATH`) は実 BP に合わせて要確認 (`frontend_editor/editor_placer.py:18`)

## Editor 起動回避パッチ (reduced Odaiba checkout 用)

このリポジトリは "reduced Odaiba CARLA checkout" 状態 (`tier4/odaiba-carla`) で、
標準の `cmake --build Build --target launch` だけでは Editor がクラッシュする。
以下のパッチが必要 (詳細は `work.knowledge/008_editor-startup-workarounds.md`):

1. `Content/Carla/.git/` ダミーディレクトリ作成
2. `RclcppBridge/CMakeLists.txt` 手動追加
3. `LoadAssetMaterialsCommandlet.cpp` の nullptr ガード
4. `DefaultEngine.ini` で `r.CARLA.EnableSegmentationRendering=0`

## 関連ドキュメント

- 設計仕様: `docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md`
- 実装プラン: `docs/superpowers/plans/2026-05-20-lanelet2-traffic-light.md`
- Phase 0 検証メモ: `docs/phase0_validation.md` (パッケージ内)
- 知見一覧: `work.knowledge/` (ワークスペース直下)

## 信号機の状態を維持して観察する (Phase 6 暫定)

`control_traffic_light_by_sign_id.py --freeze` 単独では cycle 進行が止まらない (Phase 6 時点で `freeze_all_traffic_lights` が `ATrafficLightManager::TrafficGroups[]` の空ループで no-op になる)。Phase 7+ で C++ 側の self-init fallback で対処予定。

それまでは cycle 時間を巨大値に上書きする workaround を使う:

```bash
# 例: sign_id=6621 を red のまま保持
python3 control_traffic_light_by_sign_id.py --id 6621 --state red --freeze \
    --green-time 99999 --yellow-time 99999 --red-time 99999 --all-in-group
```

`--all-in-group` で同じ Group 内の全 TL に cycle 時間が適用される。
