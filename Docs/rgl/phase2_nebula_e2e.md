# Phase 2: tier4/nebula E2E verification

Runs the Phase 1 UDP raw-packet output through the production ROS 2
driver `tier4/nebula` and validates the resulting PointCloud2 messages
on five indicators (receipt count, ring count, point count, azimuth
coverage, distance range) across all eight supported LiDAR models.

## Prerequisites

- Phase 1 shipping package built
  (`Build/Package/Carla-0.10.0-Linux-Shipping/Linux/CarlaUnreal.sh`).
- `libRobotecGPULidar.so` with the UDP extension
  (`RglSetup.sh prepare --with-udp`).
- CARLA Python wheel installed
  (`pip install Build/PythonAPI/dist/carla-*.whl`).
- A colcon workspace containing `nebula_ros` (the path used by our team
  is `/mnt/dsk0/wk0/ROS2/humble/X2/shiojiri/pilot-auto.x2_v4.3.1_awsim/install`).
- Python: `numpy`, `pyyaml`, `rclpy` (from ROS 2 Humble), `sensor_msgs`.

While Phase 2 is running, **no other Autoware ROS 2 process should be
publishing on the same ROS_DOMAIN_ID** — Nebula's `/aw_points` topic
must come solely from the test-launched Nebula process. The test
inherits whatever `ROS_DOMAIN_ID` is set in the shell (no hard-coded
value); the team's default is `88`.

## Running the test

```bash
source /opt/ros/humble/setup.bash
source /path/to/colcon_workspace/install/setup.bash

cd /path/to/CarlaUE5/PythonAPI/rgl/tests
python3 test_phase2_nebula_e2e.py
```

The test will:

1. Auto-launch `CarlaUnreal.sh -RenderOffScreen -nosound`.
2. Generate a per-model Nebula override YAML under `/tmp/phase2_<model>.yaml`
   that pins `host_ip`, `sensor_ip`, `data_port`, `return_mode`, `max_range`,
   and forces `udp_only=true` / `setup_sensor=false` (these cannot be set
   via launch CLI args because nebula_ros's `*_launch_all_hw.xml` only
   propagates `sensor_model` and `launch_hw` to the node).
3. For each model in turn: launch
   `ros2 launch nebula_ros <vendor>_launch_all_hw.xml config_file:=...`,
   spawn the CARLA RGL LiDAR with matching UDP destination, collect
   five PointCloud2 messages, validate the five indicators, tear down.
4. Print a final `N/8 passed` summary. Exit 0 only if every model passes.

The full run takes about 4 minutes (8 models × ~30 s each).

## Exit codes

| code | meaning |
|---|---|
| 0 | all 8 models passed |
| 1 | at least one model failed |
| 2 | prerequisite missing (CARLA binary, Python wheels, PyYAML, Nebula on AMENT_PREFIX_PATH) |

## Indicators per model

| model | rings | min_pts_avg | max_range_m |
|---|---|---|---|
| VelodyneVLP16 | 16 | 1 000 | 200 |
| VelodyneVLP32C | 32 | 1 000 | 200 |
| VelodyneVLS128 | 128 | 5 000 | 200 |
| HesaiPandar40P | 40 | 1 000 | 200 |
| HesaiPandarQT | 64 | 1 000 | 200 |
| HesaiPandarXT32 | 32 | 1 000 | 200 |
| HesaiQT128C2X | 128 | 5 000 | 200 |
| HesaiPandar128E4X | 128 | 5 000 | 200 |

Soft indicators (azimuth coverage and distance range) require at least
3 of 5 collected scans to satisfy the threshold, so a single bad scan
does not fail the model.

## Return-mode notes

Phase 2 uses Single Return everywhere. Velodyne's nebula_ros wrapper
goes through the generic `return_mode_from_string` in
`nebula_common.hpp`, which accepts only `SingleFirst` / `SingleStrongest`
/ `SingleLast` / `Dual`. Hesai's wrapper uses the model-aware
`return_mode_from_string_hesai`, which accepts `Strongest` / `First` /
`Last` / etc. Concretely:

| model | CARLA `return_mode` | Nebula `return_mode` |
|---|---|---|
| Velodyne VLP16/VLP32C/VLS128 | `strongest` | `SingleStrongest` |
| HesaiPandar40P / XT32 / QT128C2X / Pandar128E4X | `strongest` | `Strongest` |
| HesaiPandarQT | `first` | `First` |

The HesaiPandarQT exception exists because the AWSIM whitelist for that
model excludes Strongest (`{first, last, first_last}`).

## HesaiPandarQT-specific UDP options

For HesaiPandarQT (PandarQT64) the test sets two additional flags on
the CARLA side via `apply_preset(..., udp_publish=...)`:

- `enable_hesai_udp_sequence=True` — Nebula's `PacketQT64` struct ends
  with a mandatory `uint32_t udp_sequence` field, so the trailing 4 bytes
  must be present even though AWSIM does not force this flag for QT64.
- `ensure_hesai_pandar_driver_compat=True` — Nebula descends from the
  official Hesai Pandar ROS driver (TIER IV fork) and inherits the
  driver's accumulated packet-layout adjustments. The most visible ones
  are the **per-channel azimuth offsets** that compensate for how the
  PandarQT reports angles relative to the laser firing order rather than
  the geometric centre. The AWSIM patch
  (`RGL_UDP_FIT_QT64_TO_HESAI_PANDAR_DRIVER`) reproduces those
  adjustments on the emitted packets, so this flag is required for
  Nebula to output geometrically-correct points; without it the cloud
  parses but is azimuth-rotated.

## Troubleshooting

| symptom | likely cause | what to try |
|---|---|---|
| `Nebula '/aw_points' topic did not appear within 15s` | Nebula launch failed; the YAML override might be malformed | Run the failing `ros2 launch nebula_ros ... config_file:=/tmp/phase2_<model>.yaml` line manually and read stderr |
| `Invalid echo model provided` on Velodyne | Wrong `return_mode` string for the generic parser | Use `SingleStrongest` (not `Strongest`) for any Velodyne sensor |
| `only N PointCloud2 msgs received` | UDP not reaching Nebula or decode failure | `nc -ul 127.0.0.1 <port>` to inspect raw bytes; verify the `data_port` in the generated YAML matches the test's `DEST_PORT_BASE + index` |
| `ring_count_exact: False` | Nebula calibration file mismatch | Inspect the Nebula `calibration_file` parameter for the failing model |
| `distance_in_range: 0/5` even with plenty of points | Nebula emits points beyond the Phase 2 distance threshold (default 200 m) | The test caps Nebula's `max_range` via the YAML overlay; if you change `EXPECTED[model]["max_range_m"]`, the cap follows automatically |
| Hesai cloud parses but azimuth looks rotated | `ensure_hesai_pandar_driver_compat` missing for a PandarQT-family sensor | Enable it on the CARLA side via `udp_publish={..., "ensure_hesai_pandar_driver_compat": True}` |
| Hangs at `Launching CARLA` | `CarlaUnreal.sh` cannot find a GPU / `-RenderOffScreen` fails | Run the binary manually first and inspect `/tmp/carla_phase2.log` |
