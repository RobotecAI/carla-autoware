"""Spawn a vehicle, control it via keyboard using AccelerationControl, and plot I/O in real time."""

"""
Vehicle controls:
    W / ↑    : increase acceleration (max +2 m/s²)
    S / ↓    : decrease acceleration (min -2 m/s², decelerate/reverse)
    A / ←    : steer left
    D / →    : steer right
    Space    : reset acceleration target to 0
    P        : toggle autopilot
    ESC      : quit

Plots:
    - Acceleration I/O: commanded acceleration vs estimated actual acceleration
    - Steering I/O: input steer vs applied steer
"""

import carla
from carla import ColorConverter as cc
import argparse
import random
import sys
import weakref
import math
import time
import threading
import types
from collections import deque

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use the TkAgg backend
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.ticker import MultipleLocator
except ImportError:
    raise RuntimeError('cannot import matplotlib, make sure matplotlib package is installed')

#
# =========================
# TUNING PARAMETERS (EDIT ME)
# =========================
# Group rate/limit parameters here so you can tell others: "Try tweaking only this section."
#
# - steer_rate_limit_1ps:
#     Steering rate limit applied on the server (C++) side. Units: [1/s] on normalized steer [-1, 1].
#     Example: 20.0 means it takes ~0.05 s (1/20) to go from 0 to 1.
# - constant_accel_jerk_limit_*_mps3:
#     Jerk limits for AccelerationControl (constant acceleration) in [m/s^3].
#     You can set different values for increasing vs decreasing acceleration.
# - key_accel_change_rate_mps2ps:
#     How fast the acceleration target changes with keyboard input [m/s^2 per second].
# - key_accel_max/min_mps2:
#     Upper/lower bounds for the acceleration target [m/s^2].
# - key_steer_max:
#     Max target steer generated from key input (CARLA normalized steer range [-1, 1]).
# - steer_first_order_lag_tau_s:
#     First-order lag time constant (seconds) applied on the server side to steering output.
#     Use <= 0 to disable.
# - constant_accel_first_order_lag_tau_s:
#     First-order lag time constant (seconds) applied on the server side to constant-acceleration target.
#     Use <= 0 to disable.
# - wheel_physics:
#     Per-axle wheel parameters applied once after spawn (lateral-focused). Omit a key or use None to
#     leave the vehicle default for that field. Valid key names are listed in WHEEL_PHYSICS_KEYS below.
#     Command-line flags override these values when the flag is passed.
#
TUNING = {
    # Simulator-side (server) rate limits & first-order lag
    # rate limits
    "constant_accel_jerk_limit_pos_mps3": 10000,
    "constant_accel_jerk_limit_neg_mps3": 10000,
    "steer_rate_limit_1ps": 10000,

    # first-order lag
    "steer_first_order_lag_tau_s": 0,
    "constant_accel_first_order_lag_tau_s": 0,

    # Keyboard input shaping (client-side)
    "key_accel_change_rate_mps2ps": 6.0,
    "key_accel_max_mps2": 2.0,
    "key_accel_min_mps2": -2.0,
    "key_steer_max": 1.0,

    # Wheel physics: only list keys you want to change (see WHEEL_PHYSICS_KEYS below for all names).
    # Note: weakening only the front axle often still corners because the rear axle generates lateral force.
    "wheel_physics": {
        "front_friction_force_multiplier_mul": 1,
        "rear_friction_force_multiplier_mul": 1,
        "all_wheels_cornering_stiffness_mul": 1,
    },
}

WHEEL_PHYSICS_KEYS = (
    "front_cornering_stiffness_mul",
    "rear_cornering_stiffness_mul",
    "front_friction_force_multiplier_mul",
    "rear_friction_force_multiplier_mul",
    "front_side_slip_modifier",
    "rear_side_slip_modifier",
    "front_side_slip_modifier_mul",
    "rear_side_slip_modifier_mul",
    "front_skid_threshold_mul",
    "rear_skid_threshold_mul",
    "front_lateral_slip_graph_y_mul",
    "rear_lateral_slip_graph_y_mul",
    # Applied to every wheel after per-axle multipliers (sanity test / global grip).
    "all_wheels_cornering_stiffness_mul",
    "all_wheels_friction_force_multiplier_mul",
    "all_wheels_side_slip_modifier_mul",
    "all_wheels_lateral_slip_graph_y_mul",
    "all_wheels_skid_threshold_mul",
)

# Ensure merge_wheel_physics_tuning sees every key (unset -> None) without repeating them inside TUNING.
for _k in WHEEL_PHYSICS_KEYS:
    TUNING.setdefault("wheel_physics", {}).setdefault(_k, None)


def merge_wheel_physics_tuning(args):
    """Merge TUNING['wheel_physics'] with CLI; CLI wins when the argument is not None."""
    base = TUNING.get("wheel_physics", {})
    merged = {}
    for k in WHEEL_PHYSICS_KEYS:
        cli = getattr(args, k, None)
        merged[k] = cli if cli is not None else base.get(k)
    return types.SimpleNamespace(**merged)


def _wheel_field_snapshot(w):
    """Scalar snapshot of one wheel for verify prints (copies floats, not references)."""
    def gv(name):
        try:
            return float(getattr(w, name))
        except Exception:
            return float("nan")

    lsg_n = 0
    lsg_y0 = float("nan")
    try:
        g = getattr(w, "lateral_slip_graph", None)
        if g is not None and len(g) > 0:
            lsg_n = int(len(g))
            lsg_y0 = float(g[0].y)
    except Exception:
        pass

    return (
        gv("cornering_stiffness"),
        gv("friction_force_multiplier"),
        gv("side_slip_modifier"),
        gv("skid_threshold"),
        lsg_n,
        lsg_y0,
    )


def _print_wheel_snapshots_table(title, rows):
    print("=" * 76)
    print(title)
    print("idx  cornering   frict_mul  side_slip_m  skid_thr  lsg_n  lsg_y[0]")
    for i, row in enumerate(rows):
        cs, ffm, ssm, sk, ln, y0 = row
        print(
            "  %d  %11.5g  %9.5g  %11.5g  %9.5g  %5d  %9.5g"
            % (i, cs, ffm, ssm, sk, ln, y0)
        )
    print("=" * 76)


def _print_wheel_verify_delta(before_rows, after_rows):
    if len(before_rows) != len(after_rows):
        print("wheel_physics verify: wheel count mismatch (%d vs %d)" % (len(before_rows), len(after_rows)))
        return
    names = ("cornering", "frict_mul", "side_slip_m", "skid_thr", "lsg_n", "lsg_y0")

    def cell_changed(a, b):
        if isinstance(a, float) and isinstance(b, float):
            if math.isnan(a) and math.isnan(b):
                return False
            if math.isnan(a) or math.isnan(b):
                return True
            return abs(a - b) > max(1e-6, 1e-9 * max(abs(a), abs(b), 1.0))
        return a != b

    any_change = False
    for i, (b, a) in enumerate(zip(before_rows, after_rows)):
        if not any(cell_changed(b[j], a[j]) for j in range(len(names))):
            continue
        any_change = True
        parts = []
        for j, name in enumerate(names):
            if cell_changed(b[j], a[j]):
                parts.append("%s: %r -> %r" % (name, b[j], a[j]))
        print("  wheel[%d]  %s" % (i, "; ".join(parts)))
    if not any_change:
        print("wheel_physics verify: no differences between before and after snapshots.")


def _wheel_physics_verify_requested(verify_cli=False):
    return bool(verify_cli) or bool(TUNING.get("wheel_physics_verify_print", False))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _scale_lateral_slip_graph(graph, y_mul):
    """Scale Y of each (x, y) in lateral_slip_graph."""
    if graph is None:
        return None
    try:
        out = []
        for p in graph:
            out.append(carla.Vector2D(float(p.x), float(p.y) * float(y_mul)))
        return out
    except Exception:
        return None


def _is_front_by_steering_flags(w):
    try:
        if hasattr(w, "affected_by_steering") and bool(w.affected_by_steering):
            return True
    except Exception:
        pass
    try:
        if hasattr(w, "max_steer_angle") and float(w.max_steer_angle) > 0.0:
            return True
    except Exception:
        pass
    return False


def _front_axle_indices_for_wheels(wheels, mode):
    """Return a set of wheel indices treated as 'front' for front_* tuning keys."""
    mode = (mode or "offset_x").lower().strip()
    n = len(wheels)
    if mode == "steering":
        return {i for i in range(n) if _is_front_by_steering_flags(wheels[i])}
    # "offset_x": two wheels with largest offset.x (vehicle-local offset from RPC).
    if n >= 2:
        try:
            scored = []
            for i in range(n):
                ox = float(wheels[i].offset.x)
                scored.append((i, ox))
            scored.sort(key=lambda t: t[1], reverse=True)
            k = min(2, n)
            return {scored[j][0] for j in range(k)}
        except Exception:
            pass
    if n >= 4:
        return {0, 1}
    if n > 0:
        return {0}
    return set()


def apply_wheel_physics_tuning(vehicle, tuning, verify=False):
    """
    Apply per-axle WheelPhysicsControl tuning (focused on lateral behavior).

    Notes:
      - This runs once after spawn.
      - It edits the VehiclePhysicsControl returned by vehicle.get_physics_control()
        and reapplies it via vehicle.apply_physics_control().
    """
    if vehicle is None:
        return

    verify = _wheel_physics_verify_requested(verify)

    if not any(getattr(tuning, k, None) is not None for k in WHEEL_PHYSICS_KEYS):
        if verify:
            try:
                rows = [_wheel_field_snapshot(w) for w in vehicle.get_physics_control().wheels]
                _print_wheel_snapshots_table(
                    "wheel_physics verify: no overrides (apply skipped); current server state",
                    rows,
                )
            except Exception as ex:
                print("wheel_physics verify (no apply): %s" % ex)
        return

    pc = vehicle.get_physics_control()
    wheels = list(pc.wheels)
    before_rows = [_wheel_field_snapshot(w) for w in wheels] if verify else None

    split_mode = str(TUNING.get("wheel_axle_split", "offset_x"))
    front_idx = _front_axle_indices_for_wheels(wheels, split_mode)

    if bool(TUNING.get("wheel_physics_debug_print", False)):
        try:
            parts = []
            for i, w in enumerate(wheels):
                tag = "F" if i in front_idx else "R"
                ox = float(w.offset.x) if hasattr(w, "offset") else float("nan")
                parts.append(f"[{i}]{tag} off.x={ox:.3f} steer={getattr(w, 'affected_by_steering', '?')}")
            print("wheel_physics: axle_split=%r %s" % (split_mode, " ".join(parts)))
        except Exception as ex:
            print("wheel_physics debug print failed:", ex)

    all_cs = getattr(tuning, "all_wheels_cornering_stiffness_mul", None)
    all_ffm = getattr(tuning, "all_wheels_friction_force_multiplier_mul", None)
    all_ssm = getattr(tuning, "all_wheels_side_slip_modifier_mul", None)
    all_lsg = getattr(tuning, "all_wheels_lateral_slip_graph_y_mul", None)
    all_sk = getattr(tuning, "all_wheels_skid_threshold_mul", None)

    for i, w in enumerate(wheels):
        front = i in front_idx

        # --- cornering stiffness ---
        m_cs = 1.0
        ax_cs = getattr(tuning, "front_cornering_stiffness_mul" if front else "rear_cornering_stiffness_mul", None)
        if ax_cs is not None:
            m_cs *= float(ax_cs)
        if all_cs is not None:
            m_cs *= float(all_cs)
        if m_cs != 1.0 and hasattr(w, "cornering_stiffness"):
            try:
                w.cornering_stiffness = float(w.cornering_stiffness) * m_cs
            except Exception:
                pass

        # --- friction force multiplier ---
        m_ffm = 1.0
        ax_ffm = getattr(
            tuning,
            "front_friction_force_multiplier_mul" if front else "rear_friction_force_multiplier_mul",
            None,
        )
        if ax_ffm is not None:
            m_ffm *= float(ax_ffm)
        if all_ffm is not None:
            m_ffm *= float(all_ffm)
        if m_ffm != 1.0 and hasattr(w, "friction_force_multiplier"):
            try:
                w.friction_force_multiplier = float(w.friction_force_multiplier) * m_ffm
            except Exception:
                pass

        # --- side slip modifier (set / mul per axle, then optional all-wheels mul) ---
        ssm_set = getattr(tuning, "front_side_slip_modifier" if front else "rear_side_slip_modifier", None)
        if ssm_set is not None and hasattr(w, "side_slip_modifier"):
            try:
                w.side_slip_modifier = float(_clamp(float(ssm_set), 0.0, 1.0))
            except Exception:
                pass

        ssm_mul = getattr(tuning, "front_side_slip_modifier_mul" if front else "rear_side_slip_modifier_mul", None)
        if ssm_mul is not None and hasattr(w, "side_slip_modifier"):
            try:
                w.side_slip_modifier = float(_clamp(float(w.side_slip_modifier) * float(ssm_mul), 0.0, 1.0))
            except Exception:
                pass

        if all_ssm is not None and hasattr(w, "side_slip_modifier"):
            try:
                w.side_slip_modifier = float(_clamp(float(w.side_slip_modifier) * float(all_ssm), 0.0, 1.0))
            except Exception:
                pass

        # --- skid threshold ---
        m_sk = 1.0
        ax_sk = getattr(tuning, "front_skid_threshold_mul" if front else "rear_skid_threshold_mul", None)
        if ax_sk is not None:
            m_sk *= float(ax_sk)
        if all_sk is not None:
            m_sk *= float(all_sk)
        if m_sk != 1.0 and hasattr(w, "skid_threshold"):
            try:
                w.skid_threshold = float(w.skid_threshold) * m_sk
            except Exception:
                pass

        # --- lateral slip graph (combine axle Y-scale and all-wheels Y-scale) ---
        m_lsg = 1.0
        ax_lsg = getattr(tuning, "front_lateral_slip_graph_y_mul" if front else "rear_lateral_slip_graph_y_mul", None)
        if ax_lsg is not None:
            m_lsg *= float(ax_lsg)
        if all_lsg is not None:
            m_lsg *= float(all_lsg)
        if m_lsg != 1.0 and hasattr(w, "lateral_slip_graph"):
            scaled = _scale_lateral_slip_graph(getattr(w, "lateral_slip_graph", None), m_lsg)
            if scaled is not None:
                try:
                    w.lateral_slip_graph = scaled
                except Exception:
                    pass

    # get_physics_control().wheels returns a Python list of copies; edits do not affect `pc`
    # until we assign the list back (SetWheels copies into rpc::VehiclePhysicsControl).
    try:
        pc.wheels = wheels
    except Exception as ex:
        print("Warning: failed to assign pc.wheels after tuning (changes may not apply): %s" % ex)

    if verify and before_rows is not None:
        _print_wheel_snapshots_table(
            "wheel_physics verify: server snapshot before local edits (get_physics_control)",
            before_rows,
        )
        active = [(k, getattr(tuning, k)) for k in WHEEL_PHYSICS_KEYS if getattr(tuning, k, None) is not None]
        print("wheel_physics verify: active overrides (%d):" % len(active))
        for k, v in active:
            print("    %s = %r" % (k, v))
        try:
            w0 = pc.wheels[0]
            print(
                "wheel_physics verify: RPC payload wheel[0] (pre-send)  cornering_stiffness=%r "
                "friction_force_multiplier=%r  n_wheels=%d"
                % (
                    getattr(w0, "cornering_stiffness", None),
                    getattr(w0, "friction_force_multiplier", None),
                    len(pc.wheels),
                )
            )
        except Exception as ex:
            print("wheel_physics verify: payload wheel[0] print failed:", ex)
        try:
            edited = [_wheel_field_snapshot(w) for w in pc.wheels]
            _print_wheel_snapshots_table(
                "wheel_physics verify: local payload after edits (pc.wheels write-back)",
                edited,
            )
        except Exception as ex:
            print("wheel_physics verify: post-edit table failed:", ex)
        try:
            import carla as _carla_mod
            print("wheel_physics verify: carla module file:", getattr(_carla_mod, "__file__", "?"))
        except Exception:
            pass

    vehicle.apply_physics_control(pc)

    # apply_physics_control is synchronous in LibCarla (CallAndWait). If you use an older
    # carla egg built with AsyncCall, wait one server frame before readback (avoid sync deadlock).
    try:
        world = vehicle.get_world()
        if not world.get_settings().synchronous_mode:
            world.wait_for_tick()
    except Exception:
        pass

    if verify and before_rows is not None:
        try:
            pc_after = vehicle.get_physics_control()
            after_rows = [_wheel_field_snapshot(w) for w in pc_after.wheels]
            _print_wheel_snapshots_table(
                "wheel_physics verify: AFTER apply_physics_control (get_physics_control readback)",
                after_rows,
            )
            print("wheel_physics verify: per-field deltas (before -> after)")
            _print_wheel_verify_delta(before_rows, after_rows)
        except Exception as ex:
            print("wheel_physics verify (after apply): %s" % ex)


def get_post_process_profile_for_map(map_name):
    """RGB camera exposure/tone-mapping profile.

    Without an explicit profile, some scenes may appear overly dark depending on the map.
    """
    if map_name and 'Town10HD_Opt' in map_name:
        return 'Town10HD_Opt'
    if map_name and 'Town_C' in map_name:
        return 'Town_C'
    return 'Default'


def get_actor_blueprints(world, filter, generation):
    """Return blueprints matching the given filter and generation."""
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3, 4]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


class CameraManager(object):
    """Manages a third-person (rear) RGB camera for the vehicle."""
    def __init__(self, parent_actor, width, height, gamma_correction=1.0):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self._width = width
        self._height = height

        # Get the vehicle bounding box
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        # Rear camera pose (behind the vehicle and slightly above)
        camera_transform = carla.Transform(
            carla.Location(x=-2.0 * bound_x, y=0.0 * bound_y, z=2.0 * bound_z),
            carla.Rotation(pitch=8.0)
        )

        # Get the RGB camera blueprint
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(width))
        bp.set_attribute('image_size_y', str(height))
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', str(gamma_correction))
        if bp.has_attribute('post_process_profile'):
            profile = get_post_process_profile_for_map(world.get_map().name)
            bp.set_attribute('post_process_profile', profile)

        # Spawn the camera (SpringArmGhost attachment provides smooth follow behavior)
        self.sensor = world.spawn_actor(
            bp,
            camera_transform,
            attach_to=self._parent,
            attachment_type=carla.AttachmentType.SpringArmGhost
        )

        # Register an image callback
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        """Convert the camera image to a pygame surface."""
        self = weak_self()
        if not self:
            return

        # BGRA -> RGB (copy the buffer to avoid callback/main-thread races)
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1].copy()
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        """Render the camera image to the display."""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def destroy(self):
        """Destroy the camera sensor."""
        if self.sensor is not None:
            self.sensor.destroy()
            self.sensor = None


class PlotDataManager(object):
    """Stores time-series data for plotting (acceleration/steering)."""
    def __init__(self, max_data_points=1000):
        self.max_data_points = max_data_points
        self.lock = threading.Lock()

        # Fixed-size deques for memory efficiency
        self.time_data = deque(maxlen=max_data_points)
        self.input_acceleration_data = deque(maxlen=max_data_points)  # [m/s^2] longitudinal (forward/backward)
        self.actual_acceleration_data = deque(maxlen=max_data_points)  # [m/s^2] estimated actual acceleration
        self.input_steer_data = deque(maxlen=max_data_points)  # [rad or normalized]
        self.output_steer_data = deque(maxlen=max_data_points)  # [rad or normalized]

        self.start_time = time.time()

    def add_data(self, input_acceleration, actual_acceleration, input_steer, output_steer):
        """Append one sample."""
        with self.lock:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.input_acceleration_data.append(input_acceleration)
            self.actual_acceleration_data.append(actual_acceleration)
            self.input_steer_data.append(input_steer)
            self.output_steer_data.append(output_steer)

    def get_data(self):
        """Return the current data (thread-safe)."""
        with self.lock:
            return {
                'time': list(self.time_data),
                'input_acceleration': list(self.input_acceleration_data),
                'actual_acceleration': list(self.actual_acceleration_data),
                'input_steer': list(self.input_steer_data),
                'output_steer': list(self.output_steer_data)
            }


class VehicleAccelerationControl(object):
    """Handles vehicle control using AccelerationControl."""
    def __init__(self, vehicle, start_in_autopilot=False, plot_data_manager=None,
                 steer_rate_limit_1ps=20.0, jerk_limit_pos_mps3=3.0, jerk_limit_neg_mps3=5.0):
        self._vehicle = vehicle
        self._autopilot_enabled = start_in_autopilot
        # _steer_cache: raw target steer derived from key input (unfiltered)
        self._steer_cache = 0.0
        # _steer_filtered: smoothed steer sent to the vehicle
        self._steer_filtered = 0.0
        self._target_acceleration_ms2 = 0.0  # target acceleration [m/s^2]
        self._acceleration_control_enabled = False
        self._control = carla.VehicleControl()
        self.plot_data_manager = plot_data_manager
        vehicle.set_autopilot(self._autopilot_enabled)

        # For estimating actual acceleration via forward-speed differentiation
        self._prev_forward_speed = 0.0  # [m/s]
        self._prev_time = None

        # Max steering angle (used for plot normalization if needed)
        self._max_steer_angle_deg = 70.0  # default
        try:
            physics_control = vehicle.get_physics_control()
            if len(physics_control.wheels) > 0:
                for wheel in physics_control.wheels:
                    if wheel.affected_by_steering:
                        self._max_steer_angle_deg = wheel.max_steer_angle
                        break
        except:
            pass

        self._use_wheel_steer_angle = False
        try:
            test_angle = vehicle.get_wheel_steer_angle(carla.WheelLocation.FrontLeft)
            if test_angle != 0.0:
                self._use_wheel_steer_angle = True
        except:
            self._use_wheel_steer_angle = False

        # Optional input shaping on the simulator side (rate limits / first-order lag).
        # These are applied in C++ (server) so they work for any client using the API.
        #
        # - Steering rate limit: [1/s] on normalized steer [-1, 1]
        # - Steering first-order lag: tau [s]
        # - Acceleration jerk limits: [m/s^3] for constant-acceleration mode
        # - Constant-acceleration first-order lag: tau [s]
        try:
            self._vehicle.set_steer_rate_limit(float(steer_rate_limit_1ps))
            self._vehicle.set_steer_first_order_lag_tau(float(TUNING["steer_first_order_lag_tau_s"]))
            self._vehicle.set_constant_acceleration_jerk_limit(
                float(jerk_limit_pos_mps3), float(jerk_limit_neg_mps3))
            self._vehicle.set_constant_acceleration_first_order_lag_tau(
                float(TUNING["constant_accel_first_order_lag_tau_s"]))
        except Exception as e:
            print(f"Warning: rate limit API not available: {e}")

    def parse_events(self, clock):
        """Process input events; return True if the program should quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == K_ESCAPE:
                    return True
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self._vehicle.set_autopilot(self._autopilot_enabled)
                    if self._autopilot_enabled:
                        self._disable_acceleration_control()
                    print('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.key == K_SPACE:
                    self._target_acceleration_ms2 = 0.0
                    self._update_acceleration_control()
                    print('Acceleration reset to 0')

        if not self._autopilot_enabled:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._update_acceleration_control()
            self._vehicle.apply_control(self._control)

            # Update plot data
            if self.plot_data_manager is not None:
                vehicle_control = self._vehicle.get_control()
                # Vehicle forward direction
                transform = self._vehicle.get_transform()
                forward_vector = transform.get_forward_vector()

                # Longitudinal speed component [m/s]
                velocity = self._vehicle.get_velocity()
                forward_speed = (
                    velocity.x * forward_vector.x +
                    velocity.y * forward_vector.y +
                    velocity.z * forward_vector.z
                )

                # Estimate acceleration by numerical differentiation
                current_time = time.time()
                if self._prev_time is None:
                    actual_acceleration = 0.0
                else:
                    dt = current_time - self._prev_time
                    if dt > 0.0:
                        actual_acceleration = (forward_speed - self._prev_forward_speed) / dt
                    else:
                        actual_acceleration = 0.0
                self._prev_time = current_time
                self._prev_forward_speed = forward_speed

                input_acceleration = self._target_acceleration_ms2
                input_steer = self._control.steer
                output_steer = vehicle_control.steer

                self.plot_data_manager.add_data(
                    input_acceleration,
                    actual_acceleration,
                    input_steer,
                    output_steer
                )

        return False

    def _enable_acceleration_control(self):
        """Enable AccelerationControl."""
        if not self._acceleration_control_enabled:
            self._acceleration_control_enabled = True
            self._update_acceleration_control()

    def _disable_acceleration_control(self):
        """Disable AccelerationControl."""
        if self._acceleration_control_enabled:
            self._acceleration_control_enabled = False
            self._vehicle.disable_constant_acceleration()

    def _update_acceleration_control(self):
        """Update AccelerationControl (set the acceleration vector)."""
        if self._acceleration_control_enabled:
            # Set acceleration in the vehicle's local frame (x axis is forward)
            acceleration_vector = carla.Vector3D(self._target_acceleration_ms2, 0.0, 0.0)
            self._vehicle.enable_constant_acceleration(acceleration_vector)
        else:
            self._disable_acceleration_control()

    def _parse_vehicle_keys(self, keys, milliseconds):
        """Generate acceleration and steering commands from keyboard input."""
        MAX_ACCELERATION_MS2 = float(TUNING["key_accel_max_mps2"])  # [m/s^2]
        MIN_ACCELERATION_MS2 = float(TUNING["key_accel_min_mps2"])  # [m/s^2]
        ACCELERATION_CHANGE_RATE = float(TUNING["key_accel_change_rate_mps2ps"])  # [m/s^2 per second]
        STEER_MAX = float(TUNING["key_steer_max"])  # normalized steer [-1, 1]

        dt = max(0.0, milliseconds) / 1000.0

        # Acceleration control (W/↑ accelerate, S/↓ decelerate)
        if keys[K_UP] or keys[K_w]:
            self._target_acceleration_ms2 += ACCELERATION_CHANGE_RATE * dt
            self._target_acceleration_ms2 = min(MAX_ACCELERATION_MS2, self._target_acceleration_ms2)
            self._enable_acceleration_control()
        elif keys[K_DOWN] or keys[K_s]:
            self._target_acceleration_ms2 -= ACCELERATION_CHANGE_RATE * dt
            self._target_acceleration_ms2 = max(MIN_ACCELERATION_MS2, self._target_acceleration_ms2)
            self._enable_acceleration_control()
        else:
            pass

        # Steering control (instantaneous): left = -max, none = 0, right = +max
        left = keys[K_LEFT] or keys[K_a]
        right = keys[K_RIGHT] or keys[K_d]
        if left and not right:
            self._steer_cache = -STEER_MAX
        elif right and not left:
            self._steer_cache = STEER_MAX
        else:
            self._steer_cache = 0.0

        self._steer_filtered = float(self._steer_cache)
        self._control.steer = float(self._steer_filtered)
        self._control.throttle = 0.0
        self._control.brake = 0.0
        self._control.hand_brake = False
        velocity = self._vehicle.get_velocity()
        forward = self._vehicle.get_transform().get_forward_vector()
        forward_speed = velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z
        self._control.reverse = forward_speed < 0.0 or self._target_acceleration_ms2 < 0.0


def spawn_vehicle(world, actor_filter='vehicle.*', generation='All', role_name='hero'):
    """Spawn a vehicle actor."""
    # Prefer spawning a Lincoln MKZ (same idea as PythonAPI/examples/autoware_demo.py).
    blueprint_library = world.get_blueprint_library()

    def _try_find(bp_id):
        try:
            return blueprint_library.find(bp_id)
        except RuntimeError:
            # CARLA raises if the id is not found.
            return None

    preferred_bp = None
    for bp_id in ('vehicle.lincoln.mkz', 'vehicle.lincoln.mkz_2017'):
        preferred_bp = _try_find(bp_id)
        if preferred_bp is not None:
            break

    if preferred_bp is None:
        # Some CARLA builds expose different suffixes; try a wildcard match.
        mkz_candidates = list(blueprint_library.filter('vehicle.lincoln.mkz*'))
        preferred_bp = mkz_candidates[0] if len(mkz_candidates) > 0 else None

    if preferred_bp is not None:
        blueprint = preferred_bp
    else:
        blueprint_list = get_actor_blueprints(world, actor_filter, generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)

    blueprint.set_attribute('role_name', role_name)

    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)

    if blueprint.has_attribute('is_invincible'):
        blueprint.set_attribute('is_invincible', 'true')

    map = world.get_map()
    spawn_points = map.get_spawn_points()

    if not spawn_points:
        print('There are no spawn points available in your map/town.')
        print('Please add some Vehicle Spawn Point to your UE5 scene.')
        sys.exit(1)

    spawn_point = spawn_points[1]
    vehicle = world.try_spawn_actor(blueprint, spawn_point)

    if vehicle is None:
        raise RuntimeError("Failed to spawn vehicle")

    # Ensure the Acceleration Control API is available (requires PythonAPI built from this repo)
    if not hasattr(vehicle, 'enable_constant_acceleration'):
        vehicle.destroy()
        print("\n" + "=" * 70)
        print("ERROR: enable_constant_acceleration is not available.")
        print("This script requires the 'carla' Python package built from the")
        print("CarlaUE5 repo (with Acceleration Control support).")
        print("Do NOT use 'pip install carla'. Instead, from the repo root:")
        print("  cd PythonAPI/carla && pip install -e .")
        print("=" * 70 + "\n")
        sys.exit(1)

    return vehicle


class RealtimePlotter(object):
    """Displays real-time plots (acceleration and steering)."""
    def __init__(self, plot_data_manager):
        self.plot_data_manager = plot_data_manager
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('Vehicle Acceleration Control - Input/Output Comparison', fontsize=14)

        # Acceleration plot setup
        self.ax1.set_xlabel('Time [s]')
        self.ax1.set_ylabel('Acceleration [m/s²]')
        self.ax1.set_title('Acceleration: Input vs Actual')
        self.ax1.grid(True)
        self.line1_input, = self.ax1.plot([], [], 'b-', label='Input Acceleration', linewidth=2)
        self.line1_actual, = self.ax1.plot([], [], 'r-', label='Actual Acceleration', linewidth=2)
        self.ax1.legend(loc='upper right')
        self.ax1.set_ylim([-2, 2])  # m/s^2
        self.ax1.xaxis.set_major_locator(MultipleLocator(1.0))

        # Steering plot setup
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Steer [normalized]')
        self.ax2.set_title('Steer: Input vs Output')
        self.ax2.grid(True)
        self.line2_input, = self.ax2.plot([], [], 'b-', label='Input Steer', linewidth=2)
        self.line2_output, = self.ax2.plot([], [], 'r-', label='Output Steer', linewidth=2)
        self.ax2.legend(loc='upper right')
        self.ax2.set_ylim([-1.0, 1.0])
        self.ax2.xaxis.set_major_locator(MultipleLocator(1.0))

        plt.tight_layout()
        self.animation = FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False
        )

    def update_plot(self, frame):
        """Update the plots."""
        data = self.plot_data_manager.get_data()

        if len(data['time']) > 0:
            # Update acceleration plot
            self.line1_input.set_data(data['time'], data['input_acceleration'])
            self.line1_actual.set_data(data['time'], data['actual_acceleration'])

            # Update steering plot
            self.line2_input.set_data(data['time'], data['input_steer'])
            self.line2_output.set_data(data['time'], data['output_steer'])

            max_time = max(data['time'])
            min_time = max(0, max_time - 30)
            self.ax1.set_xlim([min_time, max_time + 1])
            self.ax2.set_xlim([min_time, max_time + 1])

        return [self.line1_input, self.line1_actual, self.line2_input, self.line2_output]

    def show(self):
        """Show the plot window."""
        plt.show(block=False)

    def close(self):
        """Close the plot window."""
        plt.close(self.fig)


def game_loop(args):
    """Main game loop."""
    pygame.init()
    vehicle = None
    plotter = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        world = client.get_world()

        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        print("Spawning vehicle...")
        vehicle = spawn_vehicle(
            world,
            actor_filter=args.filter,
            generation=args.generation,
            role_name=args.rolename
        )
        print("Vehicle spawned successfully!")

        # Apply wheel-physics tuning (especially lateral) right after spawn.
        try:
            apply_wheel_physics_tuning(
                vehicle,
                merge_wheel_physics_tuning(args),
                verify=getattr(args, "verify_wheel_physics", False),
            )
        except Exception as e:
            print(f"Warning: failed to apply wheel physics tuning: {e}")

        # Initialize the pygame display before matplotlib (prevents a black camera window on Linux)
        display = pygame.display.set_mode((args.width, args.height))
        pygame.display.set_caption('CARLA Vehicle Acceleration Control Plot')
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 36)

        plot_data_manager = PlotDataManager(max_data_points=2000)
        plotter = RealtimePlotter(plot_data_manager)
        plotter.show()

        controller = VehicleAccelerationControl(
            vehicle,
            args.autopilot,
            plot_data_manager,
            steer_rate_limit_1ps=args.steer_rate_limit_1ps,
            jerk_limit_pos_mps3=args.jerk_limit_pos_mps3,
            jerk_limit_neg_mps3=args.jerk_limit_neg_mps3,
        )

        camera_manager = CameraManager(vehicle, args.width, args.height, args.gamma)

        print(__doc__)
        print("\nVehicle Acceleration Control ready. Use WASD or Arrow keys to control the vehicle.")
        print("Press ESC to exit.\n")
        print("Graphs show input vs actual acceleration.")

        while True:
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            clock.tick(60)

            if controller.parse_events(clock):
                break

            display.fill((0, 0, 0))
            camera_manager.render(display)

            velocity = vehicle.get_velocity()
            acc = vehicle.get_acceleration()
            forward_vector = vehicle.get_transform().get_forward_vector()
            # Longitudinal acceleration (forward component): project world acceleration onto forward direction
            acc_longitudinal = (
                acc.x * forward_vector.x +
                acc.y * forward_vector.y +
                acc.z * forward_vector.z
            )
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            info_text = [
                f"FPS: {int(clock.get_fps())}",
                f"Speed: {speed_kmh:.1f} km/h",
                f"Target Accel: {controller._target_acceleration_ms2:.2f} m/s²",
                f"Actual Accel (long.): {acc_longitudinal:.2f} m/s²",
                f"Steer: {vehicle.get_control().steer:.2f}",
                f"Target Steer: {controller._control.steer:.2f}",
                f"Acceleration Control: {'ON' if controller._acceleration_control_enabled else 'OFF'}",
                "ESC: Exit"
            ]

            y_offset = 10
            for line in info_text:
                text_surface = font.render(line, True, (255, 255, 255))
                text_rect = text_surface.get_rect()
                display.blit(text_surface, (args.width - text_rect.width - 20, y_offset))
                y_offset += 30

            pygame.display.flip()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print('\nCancelled by user.')
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if plotter is not None:
            plotter.close()
            print("Plotter closed.")
        if 'camera_manager' in locals() and camera_manager is not None:
            camera_manager.destroy()
            print("Camera destroyed.")
        if vehicle is not None:
            try:
                vehicle.disable_constant_acceleration()
            except:
                pass
            vehicle.destroy()
            print("Vehicle destroyed.")
        pygame.quit()


def main():
    """Entry point."""
    argparser = argparse.ArgumentParser(description='CARLA Vehicle Acceleration Control Plot')
    argparser.add_argument(
        '--host', metavar='H', default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port', metavar='P', default=2000, type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot', action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--filter', metavar='PATTERN', default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation', metavar='G', default='All',
        help='restrict to certain actor generation (values: "2","3","4","All" - default: "All")')
    argparser.add_argument(
        '--rolename', metavar='NAME', default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--res', metavar='WIDTHxHEIGHT', default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--gamma', default=1.0, type=float,
        help='Gamma correction of the camera (default: 1.0)')
    argparser.add_argument(
        '--sync', action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--steer-rate-limit-1ps', default=TUNING["steer_rate_limit_1ps"], type=float,
        help='Steering rate limit [1/s] on normalized steer [-1,1]. Use <=0 to disable. (default: 20.0)')
    argparser.add_argument(
        '--jerk-limit-pos-mps3', default=TUNING["constant_accel_jerk_limit_pos_mps3"], type=float,
        help='Acceleration jerk limit for increasing accel [m/s^3]. Use <=0 to disable. (default: 3.0)')
    argparser.add_argument(
        '--jerk-limit-neg-mps3', default=TUNING["constant_accel_jerk_limit_neg_mps3"], type=float,
        help='Acceleration jerk limit for decreasing accel [m/s^3]. Use <=0 to disable. (default: 5.0)')

    # Wheel physics tuning (per axle). Defaults come from TUNING['wheel_physics']; CLI overrides when set.
    argparser.add_argument(
        '--front-cornering-stiffness-mul', default=None, type=float,
        help='Multiply front wheels cornering_stiffness (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-cornering-stiffness-mul', default=None, type=float,
        help='Multiply rear wheels cornering_stiffness (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--front-friction-force-multiplier-mul', default=None, type=float,
        help='Multiply front wheels friction_force_multiplier (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-friction-force-multiplier-mul', default=None, type=float,
        help='Multiply rear wheels friction_force_multiplier (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--front-side-slip-modifier', default=None, type=float,
        help='Set front wheels side_slip_modifier [0..1] (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-side-slip-modifier', default=None, type=float,
        help='Set rear wheels side_slip_modifier [0..1] (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--front-side-slip-modifier-mul', default=None, type=float,
        help='Multiply front side_slip_modifier, clamped [0..1] (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-side-slip-modifier-mul', default=None, type=float,
        help='Multiply rear side_slip_modifier, clamped [0..1] (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--front-skid-threshold-mul', default=None, type=float,
        help='Multiply front wheels skid_threshold (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-skid-threshold-mul', default=None, type=float,
        help='Multiply rear wheels skid_threshold (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--front-lateral-slip-graph-y-mul', default=None, type=float,
        help='Multiply Y of front lateral_slip_graph (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--rear-lateral-slip-graph-y-mul', default=None, type=float,
        help='Multiply Y of rear lateral_slip_graph (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--all-wheels-cornering-stiffness-mul', default=None, type=float,
        help='Multiply every wheel cornering_stiffness (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--all-wheels-friction-force-multiplier-mul', default=None, type=float,
        help='Multiply every wheel friction_force_multiplier (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--all-wheels-side-slip-modifier-mul', default=None, type=float,
        help='Multiply every wheel side_slip_modifier, clamped [0..1] (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--all-wheels-lateral-slip-graph-y-mul', default=None, type=float,
        help='Multiply Y of every wheel lateral_slip_graph (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--all-wheels-skid-threshold-mul', default=None, type=float,
        help='Multiply every wheel skid_threshold (overrides TUNING wheel_physics if set)')
    argparser.add_argument(
        '--verify-wheel-physics', action='store_true',
        help='Print wheel physics before/after apply_physics_control once (also TUNING wheel_physics_verify_print)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        game_loop(args)
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

