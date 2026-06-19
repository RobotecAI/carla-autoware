# LiDAR UDP Raw Packet (AWSIM Compatible)

Emit raw UDP packets from a CARLA RGL LiDAR sensor in the wire format
of real Velodyne / Hesai products. Production ROS 2 drivers such as
tier4/nebula can consume the simulated stream unchanged.

## Supported LiDAR Models

| Model name | Packet format | Default dst_port |
|---|---|---|
| `VelodyneVLP16` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLP32C` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLS128` | Velodyne Legacy (1206 B) | 2368 |
| `HesaiPandar40P` | Hesai Standard | 2368 |
| `HesaiPandarQT` | Hesai PandarQT | 2368 |
| `HesaiPandarXT32` | Hesai XT32 (UDP seq) | 2368 |
| `HesaiQT128C2X` | Hesai QT128 (UDP seq) | 2368 |
| `HesaiPandar128E4X` | Hesai Pandar128 (UDP seq) | 2368 |

## Prerequisite: build RGL with the UDP extension

The UDP extension lives in the private repository
`RobotecAI/RGL-extension-udp`. An SSH key with read access to
`git@github.com:` is required to clone it.

The CARLA RGL setup script `RglSetup.sh` already exposes a `--with-udp`
flag that clones and builds the extension automatically (the same
mechanism used by `--with-weather`):

```bash
cd /path/to/CarlaUE5

# Add --with-udp to the usual prepare arguments (--optix-dir, etc.)
bash RglSetup.sh prepare --optix-dir=/path/to/optix --with-udp

# Standard CARLA setup
bash CarlaSetup.sh -i

# Build CARLA linked against the UDP-enabled RGL
bash RglSetup.sh build
```

CARLA itself still builds without the UDP extension; UDP features then
disable themselves at runtime with a warning, leaving every other RGL
feature (ROS 2 publish, Yield) untouched. The presence of the UDP
extension in `libRobotecGPULidar.so` can be checked at simulation
startup by looking for the log line:

`RGLBackendImpl: UDP publishing requested but RGL_EXTENSION_UDP not present ...`

## Python usage

```python
import carla
from lidar_models import apply_preset

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(5.0)
world = client.get_world()
bp    = world.get_blueprint_library().find("sensor.lidar.rgl")

# Example 1: Velodyne VLP16 -> localhost:2368
# VLP16 does not accept "first" in the UDP whitelist, so set "strongest".
apply_preset(bp, "VelodyneVLP16",
             udp_publish={"dest_ip": "127.0.0.1", "dest_port": 2368})
bp.set_attribute("return_mode", "strongest")

# Example 2: Hesai Pandar40P with ROS 2 driver coordinate compat + UDP
apply_preset(bp, "HesaiPandar40P",
             hesai_ros_driver_compat=True,
             udp_publish={"dest_ip": "127.0.0.1",
                          "dest_port": 2368,
                          "ensure_hesai_pandar_driver_compat": True})
bp.set_attribute("return_mode", "strongest")  # Pandar40P also rejects "first"

# Example 3: Hesai QT128C2X (UDP sequence + blockage detection)
# QT128C2X accepts "first", so explicit return_mode is optional.
apply_preset(bp, "HesaiQT128C2X",
             hesai_ros_driver_compat=True,
             udp_publish={"dest_ip": "127.0.0.1",
                          "enable_hesai_udp_sequence": True,
                          "enable_hesai_blockage_detection": True})

spawn_point = world.get_map().get_spawn_points()[0]
sensor = world.spawn_actor(bp, spawn_point)
```

### Return mode whitelist (important)

Each model has a whitelist of `return_mode` values that the UDP encoder
accepts (mirrored from AWSIM's
`LidarUdpPublisher.cs::SupportedLidarsAndReturnModes`):

| Model | Accepted return_mode values |
|---|---|
| VelodyneVLP16 / VLP32C / VLS128 | `strongest`, `last`, `last_strongest` |
| HesaiPandar40P | `strongest`, `last`, `last_strongest` |
| HesaiPandarXT32 | `strongest`, `last`, `last_strongest` |
| HesaiPandarQT | `first`, `last`, `first_last` |
| HesaiQT128C2X | `first`, `second`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest`, `strongest_second_strongest`, `first_second` |
| HesaiPandar128E4X | `first`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest` |

CARLA's default `return_mode = "first"` is **rejected by Velodyne,
Pandar40P, and PandarXT32**. For those models you must set an accepted
mode explicitly, for example with
`bp.set_attribute("return_mode", "strongest")`. Without it the UDP node
fails the whitelist check, logs a warning, and stays inactive while the
rest of the LiDAR continues to operate normally.

## Blueprint attribute reference

| Attribute | Type | Default | Description |
|---|---|---|---|
| `rgl_lidar_model_name` | string | `""` | Model name from the table above |
| `horizontal_start_angle` | float | `0.0` | Sweep start angle (deg). Set to `-90.0` for Hesai ROS driver compat |
| `rgl_udp_enabled` | bool | `false` | UDP publish toggle |
| `rgl_udp_source_ip` | string | `"0.0.0.0"` | Source IP |
| `rgl_udp_dest_ip` | string | `""` | Destination IP (empty disables UDP) |
| `rgl_udp_dest_port` | int | `2368` | Destination port |
| `rgl_udp_hesai_enable_udp_sequence` | bool | `false` | Add UDP sequence-number field |
| `rgl_udp_hesai_blockage_detection` | bool | `false` | Up-close blockage detection (HesaiQT128C2X only) |
| `rgl_udp_hesai_pandar_driver_compat` | bool | `false` | Pandar driver compatibility (HesaiPandarQT only) |

UDP publishing activates only when `rgl_udp_enabled = true` **and**
`rgl_udp_dest_ip` is non-empty. With either condition unsatisfied UDP
stays off regardless of the other attributes.

## Smoke test

```bash
# Phase 1 verification
cd /path/to/CarlaUE5/PythonAPI/rgl/tests
python3 test_udp_raw_packets.py
```

Exit codes:

- `0`: all tests passed
- `1`: at least one test failed
- `2`: UDP extension missing from the loaded `libRobotecGPULidar.so`
  (the run is skipped, not failed)

When the UDP-enabled `.so` is loaded the script runs three tests: an
all-eight-models smoke test, a VLP16 deep decode (Velodyne Legacy
Format packet structure), and a HesaiPandar40P horizontal-start-angle
end-to-end check.

## tier4/nebula integration (Phase 2 and later)

End-to-end verification against the production ROS 2 driver is covered
by Phase 2 / Phase 3, which live in separate spec / plan documents
under `docs/superpowers/specs/`.

## Troubleshooting

| Symptom | What to check |
|---|---|
| `UDP publishing requested but RGL_EXTENSION_UDP not present` warning | Confirm `libRobotecGPULidar.so` was built with the UDP extension (`nm -D` and grep for `rgl_node_points_udp_publish`) |
| No packets received | If `rgl_udp_dest_ip` is not `127.0.0.1`, verify firewall and NIC binding on the receiver side |
| Hesai azimuth looks shifted | Confirm `hesai_ros_driver_compat=True` was set, and that `ensure_hesai_pandar_driver_compat` is inside the `udp_publish` dict when applicable |
| `Return mode '...' not supported by model '...'` warning | Use the return-mode whitelist above and set the attribute with `bp.set_attribute("return_mode", ...)` |
| `UDP requires HorizontalFov=360.0 but is ...` warning | Set the LiDAR `HorizontalFov` to 360 (UDP raw packets assume a full rotation) |
