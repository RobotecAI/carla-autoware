CARLA with RobotecGPULidar (RGL)
================================

This document describes how to build and run CARLA with
[RobotecGPULidar (RGL)](https://github.com/RobotecAI/RobotecGPULidar) — a
GPU-accelerated ray-traced LiDAR simulation library. The setup is driven by
`RglSetup.sh`, which complements the standard `CarlaSetup.sh` flow with the
steps required to clone, build and integrate RGL and its dependencies.

> [!NOTE]
> RGL integration requires an NVIDIA GPU, CUDA, NVIDIA OptiX SDK 7.2 and
> ROS 2 Humble on the host machine. The GPU LiDAR pipeline also uses the
> `RclcppBridge` helper library under
> `Unreal/CarlaUnreal/Plugins/CarlaRGL/ThirdParty/RclcppBridge` to coordinate
> rclcpp/DDS domain ownership between CARLA and RGL.

## Recommended system

* Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD Ryzen 7 / AMD Ryzen 9
* +32 GB RAM
* NVIDIA RTX 3070 / 3080 / 3090 / RTX 4090 / RTX 5090 or better
* 16 GB or more VRAM
* Ubuntu 22.04 (Ubuntu 24.04 not yet verified)
* CUDA Toolkit (tested with 12.x)
* NVIDIA OptiX SDK 7.2
* ROS 2 Humble

> [!NOTE]
> Windows is not yet supported by `RglSetup.sh` — **TBD**.

## Workflow overview

RGL integration uses a three-step workflow. `CarlaSetup.sh` is run between
the two `RglSetup.sh` invocations:

```
1. bash RglSetup.sh prepare [OPTIONS]   # Check prerequisites, clone+build RGL
2. bash CarlaSetup.sh -i                # Standard CARLA setup
3. bash RglSetup.sh build  [OPTIONS]    # Reconfigure CARLA with RGL, rebuild
```

Step 1 stages the RGL library and its colcon-built dependencies
(`agnocast`, `radar_msgs`, …). Step 2 runs the standard CARLA setup (Unreal
Engine download, content, initial build). Step 3 reconfigures CMake with
`ENABLE_RGL=ON` and rebuilds CARLA so that the CarlaRGL plugin is linked in.

## Building CARLA with RGL

### Step 1 — `prepare`

Run from the CARLA root directory (`CarlaUE5/`):

```sh
cd CarlaUE5
bash RglSetup.sh prepare --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64
```

What it does:

1. Verifies CUDA, OptiX SDK, ROS 2 Humble and `patchelf` are available.
2. Clones `RobotecGPULidar/` alongside `CarlaUE5/` (git branch: `develop`).
3. Sources `/opt/ros/humble/setup.bash` and installs RGL core, PCL, ROS 2 and
   Agnocast dependencies via `python3 setup.py --install-*-deps`.
4. Builds RGL with the selected extensions and produces
   `RobotecGPULidar/build/lib/libRobotecGPULidar.so`.

> [!NOTE]
> You can also set `OptiX_INSTALL_DIR` in the environment instead of passing
> `--optix-dir=`.

### Step 2 — `CarlaSetup.sh`

Run the standard CARLA setup to download Unreal Engine 5, content and
produce a first CARLA build:

```sh
bash CarlaSetup.sh -i
```

See [README.md](README.md) for full `CarlaSetup.sh` usage.

> [!IMPORTANT]
> Do **not** use `CarlaSetup.sh -l` here — it launches the editor *without*
> RGL. RGL is only enabled after Step 3.

### Step 3 — `build`

Reconfigure CMake with `ENABLE_RGL=ON` and rebuild:

```sh
bash RglSetup.sh build                       # rebuild only
bash RglSetup.sh build --package=shipping    # + build Shipping package
bash RglSetup.sh build --package=development # + build Development package
bash RglSetup.sh build --package=launch      # + open UE5 editor with RGL
```

The script:

1. Detects and wipes stale CMake caches under `Build/` (see
   [Stale cache detection](#stale-cache-detection)).
2. Runs `cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/Toolchain.cmake
   -DENABLE_ROS2=ON -DENABLE_RGL=ON -DCARLA_DDS_VENDOR=CycloneDDS …`.
3. Invokes `cmake --build Build` and, if requested, the package target.

`RclcppBridge` is built as an ExternalProject under `Build/` using the system
compiler (not the UE toolchain), linking against `rclcpp` from
`/opt/ros/humble`.

## Rebuilding CARLA (manual)

Once `RglSetup.sh build` has succeeded once, subsequent builds can be done
with the standard CMake commands:

```sh
export CARLA_UNREAL_ENGINE_PATH=$(realpath ../UnrealEngine5_carla)
cmake --build Build                              # rebuild
cmake --build Build --target package             # Shipping package
cmake --build Build --target package-development # Development package
cmake --build Build --target launch              # UE5 editor
```

## Running a packaged build

```sh
cd Build/Package/Carla-*-Linux-Shipping/Linux
./CarlaUnreal.sh --ros2
```

## Command reference

### `prepare` options

| Option | Description |
|---|---|
| `--optix-dir=PATH`       | OptiX SDK path (or set `OptiX_INSTALL_DIR`) |
| `--rgl-branch=BRANCH`    | RGL git branch (default: `develop`) |
| `--rgl-repo=URL`         | RGL git repository URL |
| `--skip-rgl-build`       | Skip RGL build (use pre-built library) |
| `--no-pcl`               | Disable PCL extension |
| `--no-ros2-standalone`   | Disable ROS 2 standalone extension |
| `--no-agnocast`          | Disable Agnocast extension |
| `--with-weather`         | Enable weather extension (private repo) |
| `--with-udp`             | Enable UDP extension (private repo) |

### `build` options

| Option | Description |
|---|---|
| `--package=shipping`    | Build the CARLA Shipping package after rebuild |
| `--package=development` | Build the CARLA Development package after rebuild |
| `--package=launch`      | Launch the UE5 editor after rebuild |
| `--package=none`        | (default) Rebuild only, no package step |

## Stale cache detection

`RglSetup.sh` detects three classes of stale build artifacts that arise when
the workspace is renamed, copied, or when `SOURCE_DIR` changes in CMake:

| Detector | Trigger | Action |
|---|---|---|
| `verify_cmake_cache_path`              | top-level `CMAKE_HOME_DIRECTORY` mismatch | wipe `build_dir` |
| `verify_cmake_subcache_sources_exist`  | ExternalProject sub-cache points to a missing directory | wipe the affected sub-build only |
| `verify_colcon_install_path`           | colcon `setup.sh` has a baked-in path from a previous workspace | wipe `build/`, `install/`, `log/` |

These are invoked automatically:

- `prepare`: scans `RobotecGPULidar/build/` and every colcon workspace under
  `RobotecGPULidar/external/*/` and `RobotecGPULidar/extensions/*/`.
- `build`:  scans `CarlaUE5/Build/` and every ExternalProject sub-cache
  underneath it.

No flags are needed — relocating the workspace is handled transparently.

## Package compression

### When compression runs

Whether the `package*` target compresses the staged build is controlled by the
upstream `CARLA_UNREAL_PACKAGE_NO_COMPRESSION` option. Its default depends on
the build type (inherited from upstream CARLA, not RGL-specific):

| `CARLA_UNREAL_PACKAGE_BUILD_TYPE` | `NO_COMPRESSION` default | Behavior |
|---|---|---|
| `Shipping`       | `OFF` | Compression runs |
| others (Development, Debug, …) | `ON`  | Compression is skipped |

Override with `-DCARLA_UNREAL_PACKAGE_NO_COMPRESSION=ON|OFF`.

### Compression method

When compression runs, the method is selectable via `CARLA_PACKAGE_COMPRESSION`
(default: `pigz` — parallel gzip, auto-detects CPU cores):

```sh
cmake ... -DCARLA_PACKAGE_COMPRESSION=pigz   # .tar.gz, parallel gzip (default)
cmake ... -DCARLA_PACKAGE_COMPRESSION=zstd   # .tar.zst, parallel zstd (pzstd)
cmake ... -DCARLA_PACKAGE_COMPRESSION=gzip   # .tar.gz, single-threaded (compatibility)
```

`pigz` and `zstd` both auto-detect CPU cores and parallelize. Install via
`apt install pigz` or `apt install zstd`.

## Troubleshooting

### `agnocastlib` not found

```
Could not find a package configuration file provided by "agnocastlib" …
```

The colcon install for Agnocast is from a different workspace. `prepare`
detects and removes it automatically; rerun `bash RglSetup.sh prepare`.

### `CMakeCache.txt directory is different than the directory …`

A build directory was generated in a different workspace. `prepare` / `build`
detect this automatically and wipe the cache; rerun the same command.

### iceoryx link errors when building CycloneDDS

Fixed upstream in this fork by setting `-DENABLE_SHM=OFF` in CycloneDDS's
ExternalProject — CycloneDDS no longer picks up the system iceoryx built
against libstdc++ while the UE toolchain uses libc++.

### Editor crash on launch (Nanite / SkeletalMesh assertion)

Symptom — editor process aborts shortly after `cmake --build Build --target launch`
with a message similar to:

```
Assertion failed: ExternalEdgeOffset == ExternalEdges.Num()
  [File:./Developer/NaniteBuilder/Private/ClusterDAG.cpp] [Line: 126]

  Stack: Nanite::BuildDAG ← FSkeletalMeshBuilder::Build
       ← USkeletalMesh::ExecutePostLoadInternal
```

Root cause — the project's `DerivedDataCache/` (DDC) holds Nanite / asset
artefacts produced by a previous build whose internal layout no longer
matches the current code. Common after an upstream sync, branch switch, or
rebase that touches asset post-load paths. DDC is an internal cache: it
does not invalidate itself when the surrounding CARLA/UE5 code changes.

Recovery — wipe the project DDC and let UE5 rebuild it on next launch:

```sh
# 1. Kill any lingering editor / crash reporter
pkill -9 -f UnrealEditor
pkill -9 -f CrashReportClient

# 2. Remove the project DDC (typically <100 MB on a cold build)
rm -rf Unreal/CarlaUnreal/DerivedDataCache

# 3. Relaunch — initial start is slow (~5–10 min) while DDC is rebuilt
cmake --build Build --target launch
```

`RglSetup.sh build --package=launch` automates this: it tracks the last
launch's git HEAD in `Build/.last-launch-head` and clears DDC when HEAD has
moved. Suppress with `--no-clear-ddc` if you want to keep the warm cache
across switches. Manually clear with the steps above when the heuristic
misses (e.g. uncommitted local edits that change asset behaviour).

If clearing the project DDC does not resolve the crash, also check:

- Engine-shared DDC (rare): `~/Documents/Unreal Engine/Common/DerivedDataCache/`
- Engine-internal DDC (rare): `UnrealEngine5_carla/Engine/DerivedDataCache/`

Both are safe to delete; expect a longer rebuild.

## Windows

**TBD** — `RglSetup.sh` is currently a Bash script and targets Linux
(Ubuntu 22.04). A Windows equivalent (e.g. `RglSetup.bat` or PowerShell) is
not yet implemented.

## See also

- [README.md](README.md) — standard CARLA UE5 build instructions.
- [RobotecGPULidar](https://github.com/RobotecAI/RobotecGPULidar) — upstream
  RGL repository.

## UDP Raw Packet 拡張 (オプション)

AWSIM 互換の UDP Raw Packet 送出機能を有効化するには、private リポジトリ
`RobotecAI/RGL-extension-udp` を `RobotecGPULidar/extensions/udp/` に clone し、
`RGL_BUILD_UDP_EXTENSION=ON` で RGL を再ビルドします。詳細手順と Python 使用例は
[`Docs/rgl/udp_raw_packets.md`](Docs/rgl/udp_raw_packets.md) を参照してください。

UDP 拡張なしでも CARLA 本体はそのままビルド可能で、UDP 関連機能はランタイムで
自動的に無効化され (警告ログのみ出力)、他の RGL 機能 (ROS2 publish、Yield) は
影響を受けません。
