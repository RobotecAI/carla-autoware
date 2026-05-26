# Phase 0 検証メモ

`lanelet2_traffic_light` パッケージで使う座標変換式・プロファイル既定値を Odaiba 実物で確定するための検証記録。

## Task 0.1: DA_MGRS_Odaiba 実値（2026-05-21 取得）

WorldSettings は `AutowareWorldSettings`、参照アセット
`/Game/Autoware/Data/DA_MGRS_Odaiba.DA_MGRS_Odaiba` の値：

| プロパティ | 値 |
|---|---|
| MgrsOffsetPosition | X=92008.5, Y=45335.1, Z=0.0 **(単位: m と推定)** |
| MgrsGridZone | `54SUE` |
| MgrsMapName | `Odaiba` |
| GeoReference | latitude=92008.5, longitude=45335.1, altitude=0.0 ⚠️ MgrsOffsetPosition と同値が入っている (バグまたは未設定)。本プロジェクトでは未使用 |
| MgrsID | 空 GUID |

### 単位の判定根拠

lanelet2 Odaiba マップの node `local_x ≈ 89430 m` と比較：

- **offset が m と仮定**: `89430 - 92008 = -2578 m` → Odaiba マップ数 km スケール内、妥当 ✅
- offset が cm と仮定: `89430*100 - 92008 = 8,851,068 cm ≈ 88 km` → 破綻

Phase 0.2 の実地検証でも m 仮定で確認すること。

### lanelet2_map.osm の概要

(2026-05-20 計測)

| 指標 | 値 |
|---|---|
| 信号機 way (`type=traffic_light`) | 522 |
| regulatory_element traffic_light | 473 |
| mgrs_code 例 | "54SUE894431" |

→ MgrsGridZone `54SUE` と lanelet2 の `mgrs_code` が一致、整合性 OK。

## Task 0.2: 座標変換式の符号確定（2026-05-21）

### 検証に使った lanelet2 way

- way_id: **6621** (subtype=`red_yellow_green`)
- p0 local: (89133.157, 42693.555) m
- p1 local: (89133.840, 42692.569) m
- midpoint: (89133.499, 42693.062) m
- height tag: 0.45 m

### 検証経路

CARLA本体の ROS publisher (`AutowareGNSSPublisher.cpp:156-158`):

```cpp
pose.position.x = tx + mgrs_x;
pose.position.y = -ty + mgrs_y;   // Y を反転
pose.position.z = tz + mgrs_z;
```

これは「Unreal Location (m) + MgrsOffset (m) = MGRS local 座標 (m)」を意味する。
逆変換が **lanelet2 local → Unreal world 座標** の式。

### 確定式（標準 CARLA Z-up）

```
Unreal_X (cm) =  (local_x_m - offset_x_m) * 100   ← lanelet2 East → Unreal X
Unreal_Y (cm) = -(local_y_m - offset_y_m) * 100   ← lanelet2 Northing → Unreal Y (反転)
Unreal_Z (cm) =  pole_height_m * 100              ← 垂直方向、信号面高さ
```

**軸入れ替えは無い**。一度「軸入れ替えあり」と誤解したが、それは
`Traffic_Lights_*` が親アクター `OdaibaFinaL_ver7_lights` に Attach され、
親が回転を持つため Editor Details パネルの **Relative Location** と
**World Location** で Y/Z 値が入れ替わって見えたことが原因。
Python の `actor.get_actor_location()` で取得した World Location は
標準 Z-up と一致。

### 採用根拠

way_id=6621 の midpoint を上式に適用 → 予測 World 座標
`(-287500.1, +264203.8, +1230.0) cm` (pole_height=12.3m)。

実際の `Traffic_Lights_258` の **World Location** は
`(-287499.81, +264203.00, +1232.84) cm` (Python で実取得)。

| 軸 | 予測 | 実値 | 差 |
|---|---|---|---|
| X | -287500.1 | -287499.81 | 0.3 cm |
| Y | +264203.8 | +264203.00 | 0.8 cm |
| Z | +1230.0   | +1232.84  | 2.8 cm |

視認的にも完全に重なる (DBG_FORMULA_FINAL を実機確認済み)。

### Phase 1 実装への影響

`core/geometry/mgrs_transform.py` の `MgrsTransformer`:

- 単純な `(local_x → X, local_y → Y, pole_height → Z)` 対応 → **基本OK**
- `offset` 引数の単位が cm 想定だったが、**実際の単位は m** → 名前/単位調整
- pole_height は別途プロファイルから与える設計 → そのままで対応可

Phase 1 リファクタリング:
1. `MgrsTransformer` の offset 引数の単位を m に揃える (引数名を `offset_x_m` 等に)
2. `local_to_unreal_cm(local_x_m, local_y_m, height_m)` の挙動が現状 OK か再確認

## Task 0.3: プロファイル既定値 (`profile_jp.py`)（2026-05-21 部分確定）

| 定数 | 確定値 | 暫定値 (Phase 1) | 確定根拠 |
|---|---|---|---|
| `default_pole_height_m` | **12.3** | 4.5 | Traffic_Lights_258 World Z=1232.84 cm、Traffic_Lights_124 World Z=1228.18 cm の実値。Odaiba は overhead gantry 型信号機が多く 12m 級 |
| `yaw_offset_deg` | **+90** (推定) | -90.0 | 下記計算による推定。Phase 1 リファクタリング時に最終検証 |
| `front_is_left_to_right` | True (要確認) | True | 下記参照 |

### yaw 計算（推定）

way_id=6621:
- p0=(89133.157, 42693.555), p1=(89133.840, 42692.569)
- lanelet2 方向: dx=+0.683, dy=-0.986
- lanelet2 way yaw = atan2(dy, dx) = atan2(-0.986, +0.683) ≈ -55.27°
- Unreal world は Y 反転なので dy → -dy: Unreal way yaw = atan2(+0.986, +0.683) ≈ +55.27°
- 信号面の正面 = way ベクトルから 90° 回転: +55.27 + 90 = **+145.27°**

実値: Traffic_Lights_258 World Yaw = **+148.635°**
予測 +145.27° との差は **+3.36°**。誤差はメッシュ作成時の人為的な向き調整
と推定。許容範囲内なので `yaw_offset_deg = +90` を採用候補とする。

### 信号機 Rotation 詳細

Traffic_Lights_258 / Traffic_Lights_124 とも同値 (World):
- Roll  = -90°
- Pitch = +85.405°
- Yaw   = +148.635°

Roll=-90 + Pitch=85.4 はメッシュ自体を地面に対して垂直に立てるための調整
(信号面が「下を向く」のではなく「水平方向に向く」ように)。
BP 作成時 (Phase 2) で BP 内のメッシュ component に同じ Rotation を埋め
込めば、配置時の Actor Rotation は Yaw のみで済む。

### Phase 0 完了サマリ

- Phase 0.1: MgrsOffsetPosition = (92008.5, 45335.1, 0.0) m  ✅
- Phase 0.2: 確定式 (標準 Z-up, Y反転) ✅
- Phase 0.3: pole_height_m=12.3, yaw_offset_deg=+90 (推定) ✅

Phase 1 リファクタリング項目:
1. `MgrsTransformer` の offset 引数の単位を m に統一
2. `profile_jp.py` の `_default_pole_height_m` を 4.5 → 12.3 に更新
3. `profile_jp.py` の `_yaw_offset_deg` を -90 → +90 に更新
4. `MgrsTransformer.local_to_unreal_cm` 内部の符号確認 (`x_sign=+1, y_sign=-1` は現状OK)

## 動作確認 (Phase 4 で再記入)

- Dry run 実行結果: <件数>
- 点灯確認: red / yellow / green / off の検証結果
