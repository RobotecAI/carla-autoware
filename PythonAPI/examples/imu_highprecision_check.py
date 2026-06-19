#!/usr/bin/env python3
"""Verify sensor.other.imu_highprecision gyro tracks the vehicle's physical
angular velocity (the gyro fix). Run against a running CARLA server.

This is a fast smoke check, deliberately unit- and sign-convention agnostic
(CARLA's get_angular_velocity() unit/handedness is not assumed). It drives the
vehicle through a STRAIGHT phase then a TURNING phase under manual control and
asserts the IMU gyro.z (a) moves clearly while turning, (b) is driven by the
turn rather than noise (turning RMS >> straight RMS), and (c) is consistently
sign-correlated (either polarity) with the vehicle's physical yaw rate.

The OLD/standard IMU fails (a)+(b): gyro.z is stuck near 0 regardless of turning.
The authoritative sign/correctness check is the Autoware E2E (yaw must follow).
"""

import argparse
import sys
import time
import carla


def rms(values):
    return (sum(v * v for v in values) / len(values)) ** 0.5 if values else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--blueprint", default="sensor.other.imu_highprecision")
    parser.add_argument("--straight", type=float, default=5.0)
    parser.add_argument("--turn", type=float, default=8.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    bl = world.get_blueprint_library()

    imu_bp = bl.find(args.blueprint)  # raises if blueprint missing
    print(f"found blueprint: {args.blueprint}")

    vehicle_bp = bl.filter("vehicle.*")[0]
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn)

    imu = world.spawn_actor(
        imu_bp, carla.Transform(carla.Location(z=1.0)), attach_to=vehicle)

    phase = {"name": "warmup"}
    samples = []  # (phase, imu_gyro_z, phys_wz)

    def on_imu(data):
        phys_wz = vehicle.get_angular_velocity().z  # native unit; only relative magnitude used
        samples.append((phase["name"], data.gyroscope.z, phys_wz))

    imu.listen(on_imu)

    def drive(steer, seconds, name):
        phase["name"] = name
        vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=steer))
        time.sleep(seconds)

    try:
        drive(0.0, 2.0, "warmup")      # let it pick up speed; discarded
        drive(0.0, args.straight, "straight")
        drive(0.6, args.turn, "turn")
    finally:
        imu.stop()
        imu.destroy()
        vehicle.destroy()

    straight = [g for (p, g, _) in samples if p == "straight"]
    turning = [(g, w) for (p, g, w) in samples if p == "turn"]
    print(f"samples total={len(samples)} straight={len(straight)} turn={len(turning)}")
    if len(straight) < 8 or len(turning) < 8:
        print("INCONCLUSIVE: too few samples per phase; rerun (is the server ticking?)")
        sys.exit(2)

    straight_rms = rms(straight)
    turn_rms = rms([g for g, _ in turning])
    pos = sum(1 for g, w in turning if g * w > 0)
    sign_consistency = max(pos, len(turning) - pos) / len(turning)
    print(f"straight_rms(gyro.z)={straight_rms:.4f} turn_rms(gyro.z)={turn_rms:.4f} "
          f"sign_consistency={sign_consistency:.2f}")

    # Fixed IMU: gyro.z clearly moves while turning, driven by the turn, and is
    # consistently sign-correlated. Old IMU: turn_rms ~ straight_rms ~ 0.
    ok = (turn_rms > 0.03
          and turn_rms > 3.0 * (straight_rms + 1e-6)
          and sign_consistency > 0.8)
    print("RESULT=" + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
