#!/bin/bash
# ============================================================================
# Multi-sensor distance-culling regression test (automated)
#
# Launches the packaged CARLA server, then for each sensor config resets the
# world (clean baseline), runs the RGL demo, and reads the server's
# RGLSceneManager log. Verifies the union behavior: a spatially separated 2nd
# sensor must register ADDITIONAL geometry (cachedMeshes rises vs single
# sensor) — the regression the old first-tick-wins single-sensor design failed.
#
# Runs unattended. Requires GPU + package built with `package-development`.
# Override any VAR=... at invocation, e.g.:
#   SEP=250 DURATION=30 bash test_multisensor_culling.sh
# ============================================================================
set -u

# ---- config (override via env) ----
PKG_DIR="${PKG_DIR:-/mnt/dsk0/wk0/CARLA/T4Fork.RGL/CarlaUE5/Build/Package/Carla-0.10.0-Linux-Development/Linux}"
DEMO="${DEMO:-/mnt/dsk0/wk0/CARLA/T4Fork.RGL/CarlaUE5/PythonAPI/examples/rgl_test_autoware_demo.py}"
MAP="${MAP:-Town10HD_Opt}"
PORT="${PORT:-2000}"
DURATION="${DURATION:-25}"        # seconds each config runs before SIGINT
SEP="${SEP:-150}"                 # meters between the 2 separated sensors (config c)
RESET_WAIT="${RESET_WAIT:-9}"     # > SensorStaleTimeoutSeconds(5s): let registry clear
SERVER_EXTRA="${SERVER_EXTRA:--RenderOffScreen -nosound -quality-level=Low}"
LOG="${LOG:-/tmp/carla_server_$$.log}"
RESULT="${RESULT:-/tmp/rgl_multisensor_result.txt}"

SERVER_SH="$PKG_DIR/CarlaUnreal.sh"
: > "$RESULT"
log() { echo "[test] $*" | tee -a "$RESULT"; }

cleanup() {
    log "tearing down server group (pgid=$SERVER_PID)"
    # CarlaUnreal.sh launches the real binary as a child; kill the whole group.
    kill -INT  -- "-$SERVER_PID" 2>/dev/null
    sleep 5
    kill -9    -- "-$SERVER_PID" 2>/dev/null
    # belt-and-suspenders: any CarlaUnreal binary still holding our port
    pkill -9 -f "Binaries/Linux/CarlaUnreal .*carla-rpc-port=$PORT" 2>/dev/null
    sleep 1
}
trap cleanup EXIT INT TERM

# ---- 1. launch server in its own process group (setsid) ----
[ -x "$SERVER_SH" ] || { log "ERROR: launcher not found/executable: $SERVER_SH"; exit 2; }
log "launching CARLA: $MAP --ros2 -prefernvidia $SERVER_EXTRA -carla-rpc-port=$PORT"
setsid "$SERVER_SH" "$MAP" --ros2 -prefernvidia $SERVER_EXTRA -carla-rpc-port="$PORT" > "$LOG" 2>&1 &
SERVER_PID=$!   # session leader => PGID == SERVER_PID

# ---- 2. wait for RPC port ----
log "waiting for RPC port $PORT ..."
ready=0
for i in $(seq 1 90); do
    if (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then ready=1; break; fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then log "ERROR: server died early. tail:"; tail -20 "$LOG" | tee -a "$RESULT"; exit 3; fi
    sleep 2
done
[ "$ready" = 1 ] || { log "ERROR: server not ready after ~180s. tail:"; tail -20 "$LOG" | tee -a "$RESULT"; exit 3; }
log "server ready; warm-up 12s"
sleep 12

# ---- helpers ----
# max integer value of a \K-anchored PCRE across a log window (from line $1)
max_field() { tail -n +"$1" "$LOG" | grep -oP "$2" 2>/dev/null | sort -n | tail -1; }

reset_world() {  # destroy lingering actors so each config starts from a clean registry
    timeout 30 python3 - "$PORT" <<'PY' 2>/dev/null || true
import sys, carla
c = carla.Client('127.0.0.1', int(sys.argv[1])); c.set_timeout(20)
w = c.get_world()
for pat in ('sensor.*', 'vehicle.*', 'walker.*'):
    for a in w.get_actors().filter(pat):
        try: a.destroy()
        except Exception: pass
PY
    sleep "$RESET_WAIT"   # let UnregisterSensor + stale-TTL clear the registry & static meshes
}

declare -A CACHED SENSORS VRAM
run_config() {  # $1=name  rest=demo args
    local name="$1"; shift
    log "--- config '$name': reset + demo $* (${DURATION}s) ---"
    reset_world
    local before; before=$(( $(wc -l < "$LOG") + 1 ))
    timeout --signal=INT "$DURATION" python3 "$DEMO" --carla_lidar_type rgl "$@" \
        > "/tmp/demo_${name}.log" 2>&1 || true
    sleep 3
    VRAM[$name]=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    SENSORS[$name]=$(max_field "$before" 'sensors=\s*\K[0-9]+')
    # cachedMeshes appears in sync lines ("cachedMeshes= N") and the init scan ("Uploaded N unique meshes")
    local m1 m2; m1=$(max_field "$before" 'cachedMeshes=\s*\K[0-9]+'); m2=$(max_field "$before" 'Uploaded\s+\K[0-9]+')
    CACHED[$name]=$(printf '%s\n%s\n' "${m1:-0}" "${m2:-0}" | sort -n | tail -1)
    log "  VRAM=${VRAM[$name]}MiB  maxSensors=${SENSORS[$name]:-?}  maxCachedMeshes=${CACHED[$name]:-?}"
    log "  last sync: $(tail -n +"$before" "$LOG" | grep 'Dynamic sync active' | tail -1 | sed 's/.*RGLSceneManager/RGLSceneManager/')"
}

# ---- 3. run configs ----
run_config a_single    --num_lidars 1
run_config b_cluster   --num_lidars 4 --lidar_spawn_delta_x 2 --lidar_spawn_delta_y 2
run_config c_separated --num_lidars 2 --lidar_spawn_delta_x "$SEP"

# ---- 4. verdict ----
log "============================================================"
log "RESULT SUMMARY  (map=$MAP, sep=${SEP}m)"
log "  (a) single    : maxSensors=${SENSORS[a_single]:-?}  maxCachedMeshes=${CACHED[a_single]:-?}  VRAM=${VRAM[a_single]:-?}MiB"
log "  (b) cluster x4: maxSensors=${SENSORS[b_cluster]:-?}  maxCachedMeshes=${CACHED[b_cluster]:-?}  VRAM=${VRAM[b_cluster]:-?}MiB"
log "  (c) separated : maxSensors=${SENSORS[c_separated]:-?}  maxCachedMeshes=${CACHED[c_separated]:-?}  VRAM=${VRAM[c_separated]:-?}MiB"
log "------------------------------------------------------------"
verdict=0
a=${CACHED[a_single]:-0}; c=${CACHED[c_separated]:-0}
sa=${SENSORS[a_single]:-0}; sc=${SENSORS[c_separated]:-0}
[ "${a:-0}" -gt 0 ] || { log "FAIL: (a) produced no cachedMeshes reading"; verdict=1; }
if [ "${sc:-0}" -gt "${sa:-0}" ]; then log "PASS: more sensors registered when separated (c=$sc > a=$sa)"; else log "FAIL: expected sensors(c) > sensors(a), got c=$sc a=$sa"; verdict=1; fi
if [ "${c:-0}" -gt "${a:-0}" ]; then log "PASS: union registers more geometry for separated sensors (c=$c > a=$a)"; else log "FAIL: expected cachedMeshes(c) > cachedMeshes(a), got c=$c a=$a — if c≈a, increase SEP and re-run"; verdict=1; fi
[ "$verdict" = 0 ] && log "OVERALL: PASS" || log "OVERALL: FAIL (full server log: $LOG)"
log "============================================================"
exit "$verdict"
