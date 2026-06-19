// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/InertialMeasurementUnit.h"
#include "Carla/Sensor/IMUSubstepCallback.h"

#include <limits>

#include <util/disable-ue4-macros.h>
#include "carla/geom/Vector3D.h"
#include <util/enable-ue4-macros.h>

#include "InertialMeasurementUnitHighPrecision.generated.h"

/// High-precision IMU. When a valid Chaos vehicle physics proxy is available,
/// the gyroscope and accelerometer are computed and published once per physics
/// substep (each with its own timestamp), driven by a PostIntegrate sim
/// callback that captures the vehicle body kinematics on the physics thread.
/// If no proxy is available (or no substep samples arrive), it falls back to
/// the Phase 1 frame-rate path inherited from AInertialMeasurementUnit
/// (transform-difference gyro). Noise/bias model, compass, serialization and
/// ROS2 publishing are reused from the base class.
UCLASS()
class CARLA_API AInertialMeasurementUnitHighPrecision : public AInertialMeasurementUnit
{
  GENERATED_BODY()

public:

  AInertialMeasurementUnitHighPrecision(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  virtual carla::geom::Vector3D ComputeGyroscope() override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

protected:

  virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;

private:

  // ---- Phase 1 frame-rate gyro state (used by the fallback path) ----

  /// Sensor world rotation captured on the previous tick.
  FQuat PrevRotation = FQuat::Identity;

  /// World time (seconds) of the previous tick; NaN until first valid tick.
  float PrevGyroTime = std::numeric_limits<float>::quiet_NaN();

  // ---- Substep callback wiring ----

  FIMUSubstepCallback* SubstepCallback = nullptr;
  Chaos::FSingleParticlePhysicsProxy* VehicleProxy = nullptr;

  void RegisterSubstepCallback();
  void UnregisterSubstepCallback();

  // ---- Mount (sensor relative to vehicle body), captured once ----

  FQuat   MountRelRot = FQuat::Identity;
  FVector MountOffsetLocal = FVector::ZeroVector;
  bool    bMountCaptured = false;

  // ---- Game-thread per-substep IMU integration state ----

  /// Previous IMU world rotation (for the gyro finite difference).
  FQuat PrevImuRot = FQuat::Identity;
  bool  bSubstepPrevValid = false;

  /// Two previous IMU world positions (for the 2nd-derivative accel).
  FVector PrevImuPos[2] = {
    FVector::OneVector * std::numeric_limits<float>::quiet_NaN(),
    FVector::OneVector * std::numeric_limits<float>::quiet_NaN()
  };
  float PrevSubDt = std::numeric_limits<float>::quiet_NaN();

  /// Publish a single substep sample (ROS2 + streaming) with a per-substep
  /// timestamp offset (seconds before the frame stamp).
  void PublishSubstepSample(
      double TimeOffsetSeconds,
      const carla::geom::Vector3D &Accelerometer,
      const carla::geom::Vector3D &Gyroscope,
      float Compass);
};
