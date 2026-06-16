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
    def __init__(self, vehicle, start_in_autopilot=False, plot_data_manager=None):
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
        MAX_ACCELERATION_MS2 = 2.0   # max acceleration [m/s^2]
        MIN_ACCELERATION_MS2 = -2.0   # min acceleration (decelerate/reverse) [m/s^2]
        ACCELERATION_CHANGE_RATE = 6.0  # acceleration change rate from key input [m/s^2 per second]
        STEER_CHANGE_RATE = 1.5e-3
        STEER_RETURN_RATE = 0.80
        # Smooth steering via a first-order lag (seconds).
        # Smaller = snappier, larger = smoother.
        STEER_SMOOTH_TAU_S = 0.20

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

        # Steering control
        steer_increment = STEER_CHANGE_RATE * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            if abs(self._steer_cache) > 0.01:
                self._steer_cache *= STEER_RETURN_RATE
            else:
                self._steer_cache = 0.0

        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        # Follow the target steer with an exponential moving average
        if STEER_SMOOTH_TAU_S <= 0.0 or dt <= 0.0:
            self._steer_filtered = float(self._steer_cache)
        else:
            alpha = 1.0 - math.exp(-dt / STEER_SMOOTH_TAU_S)
            self._steer_filtered = (1.0 - alpha) * self._steer_filtered + alpha * float(self._steer_cache)

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

    spawn_point = random.choice(spawn_points)
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

        # Initialize the pygame display before matplotlib (prevents a black camera window on Linux)
        display = pygame.display.set_mode((args.width, args.height))
        pygame.display.set_caption('CARLA Vehicle Acceleration Control Plot')
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 36)

        plot_data_manager = PlotDataManager(max_data_points=2000)
        plotter = RealtimePlotter(plot_data_manager)
        plotter.show()

        controller = VehicleAccelerationControl(vehicle, args.autopilot, plot_data_manager)

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

