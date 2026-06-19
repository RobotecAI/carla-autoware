# LiDAR UDP Raw Packet (AWSIM 互換)

CARLA RGL LiDAR センサーから、実 Velodyne / Hesai 製品と同じワイヤフォーマットの
生 UDP パケットを送出する機能。tier4/nebula のような実車向けドライバが、
シミュレーション出力をそのまま消費できます。

## 対応 LiDAR モデル

| モデル名 | パケット形式 | 既定 dst_port |
|---|---|---|
| `VelodyneVLP16` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLP32C` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLS128` | Velodyne Legacy (1206 B) | 2368 |
| `HesaiPandar40P` | Hesai Standard | 2368 |
| `HesaiPandarQT` | Hesai PandarQT | 2368 |
| `HesaiPandarXT32` | Hesai XT32 (UDP seq) | 2368 |
| `HesaiQT128C2X` | Hesai QT128 (UDP seq) | 2368 |
| `HesaiPandar128E4X` | Hesai Pandar128 (UDP seq) | 2368 |

## 前提条件: UDP 拡張入り RGL のビルド

`RobotecAI/RGL-extension-udp` (private リポジトリ) へのアクセス権が必要です。

```bash
cd /path/to/RobotecGPULidar/extensions
git clone git@github.com:RobotecAI/RGL-extension-udp.git udp
# extensions.repos に従ったバージョンを checkout
cd udp
git checkout $(git -C .. show HEAD:extensions.repos | \
               sed -n '/extensions\/udp:/,/version:/{s/.*version: //p}')

# RGL を UDP 拡張入りで再ビルド (CarlaUE5/RglSetup.sh の prepare ステップで
# -DRGL_BUILD_UDP_EXTENSION=ON を渡す)
cd /path/to/CarlaUE5
bash RglSetup.sh prepare -DRGL_BUILD_UDP_EXTENSION=ON
bash RglSetup.sh build
```

UDP 拡張なしでも CARLA 本体はビルド可能 (UDP 関連の機能はランタイムで自動無効化、
警告ログのみ)。`libRobotecGPULidar.so` の UDP 拡張有無は、シミュレーション起動時に
`RGLBackendImpl: UDP publishing requested but RGL_EXTENSION_UDP not present ...`
というログが出るかどうかで判別できます。

## Python 使用例

```python
import carla
from lidar_models import apply_preset

client = carla.Client("127.0.0.1", 2000); client.set_timeout(5.0)
world  = client.get_world()
bp     = world.get_blueprint_library().find("sensor.lidar.rgl")

# 例1: Velodyne VLP16 を localhost:2368 に送出
# VLP16 は UDP whitelist 上 "first" を受け付けないので strongest を明示
apply_preset(bp, "VelodyneVLP16",
             udp_publish={"dest_ip": "127.0.0.1", "dest_port": 2368})
bp.set_attribute("return_mode", "strongest")

# 例2: Hesai Pandar40P + ROS2 driver 座標互換 + UDP 送出
apply_preset(bp, "HesaiPandar40P",
             hesai_ros_driver_compat=True,
             udp_publish={"dest_ip": "127.0.0.1",
                          "dest_port": 2368,
                          "ensure_hesai_pandar_driver_compat": True})
bp.set_attribute("return_mode", "strongest")  # Pandar40P も "first" 非対応

# 例3: HesaiQT128C2X (UDP シーケンス番号 + blockage 検出)
# QT128C2X は "first" を受け付けるので return_mode 明示は不要 (もしくは "first")
apply_preset(bp, "HesaiQT128C2X",
             hesai_ros_driver_compat=True,
             udp_publish={"dest_ip": "127.0.0.1",
                          "enable_hesai_udp_sequence": True,
                          "enable_hesai_blockage_detection": True})

spawn_point = world.get_map().get_spawn_points()[0]
sensor = world.spawn_actor(bp, spawn_point)
```

### Return mode 対応表 (重要)

各モデルが UDP raw packet 化に際して受理する `return_mode` のホワイトリスト
(AWSIM `LidarUdpPublisher.cs::SupportedLidarsAndReturnModes` 由来):

| モデル | 対応 return_mode |
|---|---|
| VelodyneVLP16 / VLP32C / VLS128 | `strongest`, `last`, `last_strongest` |
| HesaiPandar40P | `strongest`, `last`, `last_strongest` |
| HesaiPandarXT32 | `strongest`, `last`, `last_strongest` |
| HesaiPandarQT | `first`, `last`, `first_last` |
| HesaiQT128C2X | `first`, `second`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest`, `strongest_second_strongest`, `first_second` |
| HesaiPandar128E4X | `first`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest` |

CARLA のデフォルト `return_mode = "first"` は **Velodyne / Pandar40P / XT32 では非対応**
なので、これらのモデルで UDP を使う場合は `bp.set_attribute("return_mode", "strongest")`
等を明示的に設定してください。設定しない場合、UDP ノードは whitelist チェックで
ブロックされ警告ログを出して無効化されます。

## 属性一覧 (Blueprint レベル直接設定する場合)

| 属性名 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `rgl_lidar_model_name` | string | `""` | 上表のモデル名 |
| `horizontal_start_angle` | float | `0.0` | sweep 開始角 (deg)、Hesai ROS driver 互換時 `-90.0` |
| `rgl_udp_enabled` | bool | `false` | UDP 送信トグル |
| `rgl_udp_source_ip` | string | `"0.0.0.0"` | 送信元 IP |
| `rgl_udp_dest_ip` | string | `""` | 宛先 IP (空 → 無効) |
| `rgl_udp_dest_port` | int | `2368` | 宛先ポート |
| `rgl_udp_hesai_enable_udp_sequence` | bool | `false` | UDP シーケンス番号有効化 |
| `rgl_udp_hesai_blockage_detection` | bool | `false` | 近距離 blockage 検出 (QT128C2X 限定) |
| `rgl_udp_hesai_pandar_driver_compat` | bool | `false` | Pandar driver 互換 (PandarQT 限定) |

UDP 送信が有効化される条件は `rgl_udp_enabled = true` **かつ** `rgl_udp_dest_ip` が
空でないこと。両者が満たされない場合は他の属性値によらず UDP は無効。

## 動作確認

```bash
# Phase 1 検証
cd /path/to/CarlaUE5/PythonAPI/rgl/tests
python3 test_udp_raw_packets.py
```

スクリプトの終了コード:
- `0`: 全テスト PASS
- `1`: いずれかのテストが FAIL
- `2`: UDP 拡張が `.so` に含まれていないため SKIP

UDP 拡張入りの `.so` がロードされていれば、8 機種のスモークテスト + VLP16 詳細
デコード + HesaiPandar40P の HFOV start offset 検証 (3 つのテスト) が走ります。

## tier4/nebula との連携 (Phase 2 以降)

実 ROS2 ドライバでの E2E 検証は Phase 2 / Phase 3 (別 spec / plan) で扱います。
詳細は `docs/superpowers/specs/` 配下の Phase 2/3 spec を参照。

## トラブルシュート

| 症状 | 確認ポイント |
|---|---|
| `UDP publishing requested but RGL_EXTENSION_UDP not present` 警告 | `libRobotecGPULidar.so` が UDP 拡張入りでビルドされているか (`nm -D` で `rgl_node_points_udp_publish` を grep) |
| パケットが届かない | `rgl_udp_dest_ip` が `127.0.0.1` 以外なら受信側のファイアウォール、NIC バインドを確認 |
| azimuth がずれる (Hesai) | `hesai_ros_driver_compat=True` を指定したか、対応する `ensure_hesai_pandar_driver_compat` を `udp_publish` 内に含めたか |
| `Return mode '...' not supported by model '...'` 警告 | 上記「Return mode 対応表」を参照し、当該モデル対応モードを `bp.set_attribute("return_mode", ...)` で設定 |
| `UDP requires HorizontalFov=360.0 but is ...` 警告 | LiDAR の HorizontalFov を 360 にする (UDP raw packet は full rotation 前提) |
