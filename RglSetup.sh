#!/bin/bash
# RglSetup.sh — RobotecGPULidar setup for CARLA.
#
# Three-step workflow (run CarlaSetup.sh between them):
#   1. bash RglSetup.sh prepare [OPTIONS]   # Check prerequisites, clone+build RGL
#   2. bash CarlaSetup.sh -i                # Standard CARLA setup (user runs this)
#   3. bash RglSetup.sh build [OPTIONS]     # Reconfigure CARLA with RGL, rebuild
#
# Subcommands:
#   prepare   Check prerequisites and build RGL library
#   build     Reconfigure CARLA with RGL options and rebuild
#
# Options for 'prepare':
#   --optix-dir=PATH          OptiX SDK path (or set OptiX_INSTALL_DIR env var)
#   --rgl-branch=BRANCH       RGL git branch (default: develop)
#   --rgl-repo=URL            RGL git repository URL
#   --skip-rgl-build          Skip RGL build (use pre-built library)
#   --no-pcl                  Disable PCL extension
#   --no-ros2-standalone      Disable ROS2 standalone extension
#   --no-agnocast             Disable Agnocast extension
#   --with-weather            Enable weather extension (disabled by default, private repo)
#   --with-udp                Enable UDP extension (disabled by default, private repo)
#
# Options for 'build':
#   --package=TYPE            shipping, development, launch, none (default: none)
#   --no-clear-ddc            Skip automatic DDC clear on launch (default: clear
#                             when git HEAD has changed since last launch)

set -e

workspace_path="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"

# ============================================================================
# Usage
# ============================================================================

usage() {
    head -28 "$0" | tail -27
    exit 1
}

# ============================================================================
# Helper: verify_cmake_cache_path
# ----------------------------------------------------------------------------
# Detect stale CMake cache from a relocated workspace and wipe the build dir
# when its CMAKE_HOME_DIRECTORY no longer matches the expected source path.
# Arguments:
#   $1 = build directory (e.g. /path/to/RobotecGPULidar/build)
#   $2 = expected CMake source directory (CMAKE_HOME_DIRECTORY)
# ============================================================================

verify_cmake_cache_path() {
    local build_dir="$1"
    local expected_src="$2"
    local cache_file="$build_dir/CMakeCache.txt"

    [ -f "$cache_file" ] || return 0

    local cached_src
    cached_src=$(grep -E '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$cache_file" | cut -d= -f2-)
    [ -n "$cached_src" ] || return 0

    # Compare resolved real paths so symlinks don't trigger false positives.
    local expected_real cached_real
    expected_real=$(realpath -m "$expected_src")
    cached_real=$(realpath -m "$cached_src" 2>/dev/null || echo "$cached_src")

    if [ "$expected_real" != "$cached_real" ]; then
        echo "[INFO] Stale CMake cache detected in $build_dir"
        echo "  cached source: $cached_src"
        echo "  expected:      $expected_src"
        echo "  Removing $build_dir to force reconfiguration..."
        rm -rf "$build_dir"
    fi
}

# ============================================================================
# Helper: verify_cmake_subcache_sources_exist
# ----------------------------------------------------------------------------
# Walk every CMakeCache.txt under a build root and delete the enclosing build
# directory when its CMAKE_HOME_DIRECTORY no longer exists. Catches
# ExternalProject sub-builds whose SOURCE_DIR was moved/renamed in the parent
# CMakeLists.txt — those sub-caches hold stale absolute source paths that the
# top-level Build/CMakeCache.txt check cannot detect.
# Arguments:
#   $1 = build root to scan (e.g. $workspace_path/Build)
# ============================================================================

verify_cmake_subcache_sources_exist() {
    local build_root="$1"
    [ -d "$build_root" ] || return 0

    local cache_file sub_build_dir cached_src
    while IFS= read -r -d '' cache_file; do
        sub_build_dir="$(dirname "$cache_file")"
        cached_src=$(grep -E '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$cache_file" | cut -d= -f2-)
        if [ -n "$cached_src" ] && [ ! -d "$cached_src" ]; then
            echo "[INFO] Stale CMake sub-cache (source missing): $sub_build_dir"
            echo "  cached source: $cached_src (does not exist)"
            echo "  Removing $sub_build_dir..."
            rm -rf "$sub_build_dir"
        fi
    done < <(find "$build_root" -name CMakeCache.txt -print0 2>/dev/null)
}

# ============================================================================
# Helper: verify_colcon_install_path
# ----------------------------------------------------------------------------
# Detect stale colcon build artifacts from a relocated workspace. colcon bakes
# the absolute install prefix into generated setup.sh files; if they no longer
# match the current location, later `source install/setup.sh` lookups fail and
# find_package() can't resolve the packages. Delete both build/ and install/
# so the next install-*-deps step rebuilds them in-place.
# Arguments:
#   $1 = colcon workspace root (e.g. $rgl_dir/external/agnocast)
# ============================================================================

verify_colcon_install_path() {
    local colcon_ws="$1"
    local setup_sh="$colcon_ws/install/setup.sh"

    [ -f "$setup_sh" ] || return 0

    local cached_prefix
    cached_prefix=$(grep -m1 -E '^_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=' "$setup_sh" | cut -d= -f2-)
    [ -n "$cached_prefix" ] || return 0

    local expected_real cached_real
    expected_real=$(realpath -m "$colcon_ws/install")
    cached_real=$(realpath -m "$cached_prefix" 2>/dev/null || echo "$cached_prefix")

    if [ "$expected_real" != "$cached_real" ]; then
        echo "[INFO] Stale colcon install detected in $colcon_ws/install"
        echo "  cached prefix: $cached_prefix"
        echo "  expected:      $colcon_ws/install"
        echo "  Removing $colcon_ws/{build,install,log} to force rebuild..."
        rm -rf "$colcon_ws/build" "$colcon_ws/install" "$colcon_ws/log"
    fi
}

# ============================================================================
# Helper: verify_ddc_freshness
# ----------------------------------------------------------------------------
# UE5's DerivedDataCache (DDC) stores baked Nanite, shader, and asset data
# keyed by content hash. After an upstream/CARLA sync the asset post-load path
# can change without invalidating the existing DDC entries, leading to runtime
# assertions inside engine code (e.g. Nanite::BuildDAG ExternalEdgeOffset).
# Heuristic: track the git HEAD at last launch; wipe the project DDC if HEAD
# moved since then. False-positives only cost one DDC rebuild on next launch.
# Arguments:
#   $1 = workspace path
# Honours: --no-clear-ddc (suppresses the wipe via _no_clear_ddc=1)
# ============================================================================

verify_ddc_freshness() {
    local ws="$1"
    [ "${_no_clear_ddc:-0}" -eq 1 ] && return 0

    local marker="$ws/Build/.last-launch-head"
    local ddc="$ws/Unreal/CarlaUnreal/DerivedDataCache"
    local current_head
    current_head=$(git -C "$ws" rev-parse HEAD 2>/dev/null || echo "")
    [ -z "$current_head" ] && return 0

    local last_head=""
    [ -f "$marker" ] && last_head=$(cat "$marker" 2>/dev/null)

    if [ "$current_head" != "$last_head" ]; then
        if [ -d "$ddc" ]; then
            echo "[INFO] git HEAD changed since last launch attempt:"
            echo "  previous: ${last_head:-(none)}"
            echo "  current:  $current_head"
            echo "  Clearing project DDC at $ddc to avoid stale Nanite/asset cache..."
            rm -rf "$ddc"
        fi
        mkdir -p "$(dirname "$marker")"
        echo "$current_head" > "$marker"
    fi
}

# ============================================================================
# Subcommand: prepare
# ============================================================================

cmd_prepare() {
    local optix_dir="${OptiX_INSTALL_DIR:-}"
    local rgl_branch="develop"
    local rgl_repo="https://github.com/RobotecAI/RobotecGPULidar.git"
    local skip_rgl_build=0

    # RGL extensions (1=enabled, 0=disabled)
    local ext_pcl=1
    local ext_ros2_standalone=1
    local ext_weather=0
    local ext_agnocast=1
    local ext_udp=0

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --optix-dir=*)  optix_dir="${1#*=}"; shift ;;
            --rgl-branch=*) rgl_branch="${1#*=}"; shift ;;
            --rgl-repo=*)   rgl_repo="${1#*=}"; shift ;;
            --skip-rgl-build) skip_rgl_build=1; shift ;;
            --no-pcl)             ext_pcl=0; shift ;;
            --no-ros2-standalone) ext_ros2_standalone=0; shift ;;
            --no-agnocast)        ext_agnocast=0; shift ;;
            --with-weather)       ext_weather=1; shift ;;
            --with-udp)           ext_udp=1; shift ;;
            *) echo "Unknown option for prepare: $1"; usage ;;
        esac
    done

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh prepare"
    echo "========================================================================"
    echo "  workspace:    $workspace_path"
    echo "  optix_dir:    ${optix_dir:-(not set)}"
    echo "  rgl_branch:   $rgl_branch"
    echo "  extensions:   pcl=$ext_pcl ros2=$ext_ros2_standalone weather=$ext_weather agnocast=$ext_agnocast udp=$ext_udp"
    echo "========================================================================"

    # ---- Prerequisites check ----

    echo ""
    echo "--- Prerequisites check ---"

    local errors=0

    # CUDA
    if command -v nvcc &>/dev/null; then
        echo "[OK] CUDA: $(nvcc --version | grep 'release' | sed 's/.*release //' | sed 's/,.*//')"
    else
        echo "[ERROR] CUDA (nvcc) not found."
        echo "  Install CUDA Toolkit: https://developer.nvidia.com/cuda-downloads"
        errors=1
    fi

    # OptiX SDK
    if [ -z "$optix_dir" ]; then
        echo "[ERROR] OptiX SDK not specified."
        echo "  Download OptiX 7.2 from:"
        echo "    https://developer.nvidia.com/designworks/optix/downloads/legacy"
        echo "  Then run with: --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64"
        echo "  Or: export OptiX_INSTALL_DIR=/path/to/..."
        errors=1
    elif [ ! -f "$optix_dir/include/optix.h" ]; then
        echo "[ERROR] optix.h not found in $optix_dir/include/"
        errors=1
    else
        echo "[OK] OptiX SDK: $optix_dir"
    fi

    # ROS2 Humble
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "[OK] ROS2 Humble: /opt/ros/humble/"
    else
        echo "[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash"
        echo "  Install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
        errors=1
    fi

    # patchelf (required by RGL ros2_standalone build)
    if command -v patchelf &>/dev/null; then
        echo "[OK] patchelf: installed"
    else
        echo "[INFO] Installing patchelf..."
        sudo apt-get install -y patchelf
        echo "[OK] patchelf: installed"
    fi

    if [ $errors -ne 0 ]; then
        echo ""
        echo "Prerequisites check failed. Fix the errors above and re-run."
        exit 1
    fi

    echo "[OK] All prerequisites OK."

    # ---- RGL clone + build ----

    echo ""
    echo "--- RGL clone + build ---"

    local rgl_dir="$workspace_path/../RobotecGPULidar"

    if [ $skip_rgl_build -eq 1 ]; then
        echo "[SKIP] RGL build (--skip-rgl-build)"
        if [ ! -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
            echo "[WARNING] RGL library not found at $rgl_dir/build/lib/libRobotecGPULidar.so"
            echo "  CARLA RGL build may fail. Remove --skip-rgl-build to build RGL."
        fi
        return
    fi

    # Detect symlink (shared via build-share.sh) — skip to avoid modifying shared RGL
    if [ -L "$rgl_dir" ]; then
        local rgl_target
        rgl_target="$(readlink -f "$rgl_dir")"
        echo "[INFO] RGL is a symlink to a shared environment:"
        echo "  $rgl_dir → $rgl_target"
        if [ -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
            echo "[OK] Shared RGL is already built. Skipping prepare."
        else
            echo "[WARNING] Shared RGL has no build artifacts."
            echo "  Build RGL in the original environment first:"
            echo "    cd $rgl_target && bash ../CarlaUE5/RglSetup.sh prepare --optix-dir=..."
        fi
        return
    fi

    # Clone if not present
    if [ -d "$rgl_dir" ]; then
        echo "[OK] RGL repository found: $rgl_dir"
    else
        echo "Cloning RobotecGPULidar (branch: $rgl_branch)..."
        git clone -b "$rgl_branch" "$rgl_repo" "$rgl_dir"
    fi

    # Detect stale CMake cache from a relocated workspace (e.g. renamed parent dir)
    # and wipe the build dir so setup.py can reconfigure cleanly.
    verify_cmake_cache_path "$rgl_dir/build" "$rgl_dir"

    # Scan all external/ and extensions/ subdirs for stale colcon installs.
    # Forward-compatible: any current or future colcon-built dependency (agnocast,
    # radar_msgs, UDP/weather private extensions, etc.) is detected automatically.
    for _colcon_ws in "$rgl_dir"/external/*/ "$rgl_dir"/extensions/*/; do
        [ -d "$_colcon_ws" ] || continue
        verify_colcon_install_path "${_colcon_ws%/}"
    done

    pushd "$rgl_dir" > /dev/null

    # Clone extension repos (separate repos, .gitignored in main RGL repo).
    # URLs and branches are parsed from extensions.repos to stay in sync with RGL upstream.
    if [ $ext_weather -eq 1 ] && [ ! -d "extensions/weather" ]; then
        local weather_url weather_branch
        weather_url=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/weather']['url'])")
        weather_branch=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/weather']['version'])")
        echo "Cloning RGL weather extension ($weather_branch)..."
        git clone -b "$weather_branch" "$weather_url" extensions/weather
    fi
    if [ $ext_udp -eq 1 ] && [ ! -d "extensions/udp" ]; then
        local udp_url udp_branch
        udp_url=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/udp']['url'])")
        udp_branch=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/udp']['version'])")
        echo "Cloning RGL UDP extension ($udp_branch)..."
        git clone -b "$udp_branch" "$udp_url" extensions/udp
    fi

    # Source ROS2
    source /opt/ros/humble/setup.bash
    export OptiX_INSTALL_DIR="$optix_dir"

    # Install core dependencies
    echo "Installing RGL core dependencies..."
    python3 setup.py --install-deps

    # Install extension dependencies
    if [ $ext_pcl -eq 1 ]; then
        echo "Installing RGL PCL dependencies..."
        python3 setup.py --install-pcl-deps
    fi
    if [ $ext_ros2_standalone -eq 1 ]; then
        echo "Installing RGL ROS2 dependencies..."
        python3 setup.py --install-ros2-deps
    fi
    if [ $ext_agnocast -eq 1 ]; then
        echo "Installing RGL Agnocast dependencies..."
        python3 setup.py --install-agnocast-deps
    fi

    # Build RGL
    local rgl_build_flags=()
    [ $ext_pcl -eq 1 ]             && rgl_build_flags+=("--with-pcl")
    [ $ext_ros2_standalone -eq 1 ] && rgl_build_flags+=("--with-ros2-standalone")
    [ $ext_weather -eq 1 ]         && rgl_build_flags+=("--with-weather")
    [ $ext_agnocast -eq 1 ]        && rgl_build_flags+=("--with-agnocast")
    [ $ext_udp -eq 1 ]             && rgl_build_flags+=("--with-udp")

    echo "Building RGL with extensions: ${rgl_build_flags[*]:-none}"
    python3 setup.py "${rgl_build_flags[@]}"

    popd > /dev/null

    # Verify
    if [ -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
        echo "[OK] RGL build succeeded."
    else
        echo "[ERROR] RGL build failed: libRobotecGPULidar.so not found."
        exit 1
    fi

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh prepare completed!"
    echo "========================================================================"
    echo ""
    echo "Next steps:"
    echo ""
    echo "  1. Set up CARLA (Content, UE5, standard build):"
    echo "     bash CarlaSetup.sh -i"
    echo "     (options: -i interactive, -p skip prerequisites)"
    echo "     (do NOT use -l here — it launches the editor without RGL)"
    echo ""
    echo "  2. Reconfigure CARLA with RGL and rebuild (after step 1):"
    echo "     bash RglSetup.sh build"
    echo "     bash RglSetup.sh build --package=shipping      # + Shipping package"
    echo "     bash RglSetup.sh build --package=development   # + Development package"
    echo "     bash RglSetup.sh build --package=launch        # + open UE5 editor with RGL"
    echo ""
    echo "  Manual cmake build (after step 2):"
    echo "     export CARLA_UNREAL_ENGINE_PATH=\$(realpath ../UnrealEngine5_carla)"
    echo "     cmake --build Build                              # rebuild"
    echo "     cmake --build Build --target package             # Shipping package"
    echo "     cmake --build Build --target package-development # Development package"
    echo "     cmake --build Build --target launch              # UE5 editor"
    echo ""
}

# ============================================================================
# Subcommand: build
# ============================================================================

cmd_build() {
    local package_type="none"
    local _no_clear_ddc=0

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --package=*) package_type="${1#*=}"; shift ;;
            --no-clear-ddc) _no_clear_ddc=1; shift ;;
            *) echo "Unknown option for build: $1"; usage ;;
        esac
    done

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh build"
    echo "========================================================================"

    # Ensure CARLA_UNREAL_ENGINE_PATH is set
    if [ -z "$CARLA_UNREAL_ENGINE_PATH" ]; then
        local ue5_path
        ue5_path="$(realpath "$workspace_path/../UnrealEngine5_carla" 2>/dev/null || true)"
        if [ -d "$ue5_path" ]; then
            export CARLA_UNREAL_ENGINE_PATH="$ue5_path"
        else
            echo "[ERROR] UnrealEngine5_carla not found."
            echo "  Expected at: $workspace_path/../UnrealEngine5_carla"
            echo "  Or set: export CARLA_UNREAL_ENGINE_PATH=/path/to/..."
            exit 1
        fi
    fi
    echo "CARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH"

    cd "$workspace_path"

    # Detect stale CMake cache from a relocated workspace and wipe Build/ so cmake
    # can reconfigure cleanly. (ExternalProject sub-caches under Build/ are also
    # wiped along with the parent directory.)
    verify_cmake_cache_path "$workspace_path/Build" "$workspace_path"

    # Also catch sub-cache drift: e.g. an ExternalProject whose SOURCE_DIR was
    # moved in CMakeLists.txt (parent workspace unchanged). The top-level cache
    # passes the check, but the sub-build still references the old path.
    verify_cmake_subcache_sources_exist "$workspace_path/Build"

    # Reconfigure with RGL
    echo ""
    echo "--- Reconfigure CARLA with RGL ---"
    cmake -G Ninja -S . -B Build \
        --toolchain="$PWD/CMake/Toolchain.cmake" \
        -DLAUNCH_ARGS="-prefernvidia" \
        -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_ROS2=ON \
        -DENABLE_RGL=ON \
        -DCARLA_DDS_VENDOR=CycloneDDS \
        -DBUILD_LIBCARLA_TESTS=OFF \
        -DCARLA_UNREAL_ENGINE_PATH="$CARLA_UNREAL_ENGINE_PATH"

    echo ""
    echo "--- Build CARLA with RGL ---"
    cmake --build Build

    # Package build
    echo ""
    echo "--- Package ---"
    case "$package_type" in
        shipping)
            echo "Building Shipping package..."
            cmake --build Build --target package
            echo "[OK] Shipping package built."
            ;;
        development)
            echo "Building Development package..."
            cmake --build Build --target package-development
            echo "[OK] Development package built."
            ;;
        launch)
            # Auto-clear DDC when git HEAD moved since last launch attempt;
            # avoids editor-side Nanite/asset assertions from stale cache.
            # Suppress with --no-clear-ddc.
            verify_ddc_freshness "$workspace_path"
            echo "Launching UE5 editor..."
            cmake --build Build --target launch
            ;;
        none)
            echo "[SKIP] Package build (--package=none)"
            ;;
        *)
            echo "[ERROR] Unknown package type: $package_type (expected: shipping, development, launch, none)"
            exit 1
            ;;
    esac

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh build completed!"
    echo "========================================================================"
    echo ""
    echo "To build packages later:"
    echo "  export CARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH"
    echo "  cd $workspace_path"
    echo "  cmake --build Build --target package              # Shipping"
    echo "  cmake --build Build --target package-development  # Development"
    echo ""
    echo "To run the server:"
    echo "  cd Build/Package/Carla-*-Linux-Shipping/Linux"
    echo "  ./CarlaUnreal.sh --ros2"
}

# ============================================================================
# Main: dispatch subcommand
# ============================================================================

if [ $# -eq 0 ]; then
    usage
fi

subcommand="$1"
shift

case "$subcommand" in
    prepare) cmd_prepare "$@" ;;
    build)   cmd_build "$@" ;;
    *)       echo "Unknown subcommand: $subcommand"; usage ;;
esac
