// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/InertialMeasurementUnit.h"

#include <limits>

#include "InertialMeasurementUnitHighPrecision.generated.h"

/// High-precision IMU. Identical to AInertialMeasurementUnit except the
/// gyroscope is computed from the finite difference of the sensor's own world
/// rotation (physics-independent), which fixes the gyro≒0 behavior seen with
/// Chaos wheeled vehicles. Accelerometer, compass, noise, serialization and
/// ROS2 publishing are inherited unchanged.
UCLASS()
class CARLA_API AInertialMeasurementUnitHighPrecision : public AInertialMeasurementUnit
{
  GENERATED_BODY()

public:

  AInertialMeasurementUnitHighPrecision(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  virtual carla::geom::Vector3D ComputeGyroscope() override;

private:

  /// Sensor world rotation captured on the previous tick.
  FQuat PrevRotation = FQuat::Identity;

  /// World time (seconds) of the previous tick; NaN until first valid tick.
  float PrevGyroTime = std::numeric_limits<float>::quiet_NaN();
};
