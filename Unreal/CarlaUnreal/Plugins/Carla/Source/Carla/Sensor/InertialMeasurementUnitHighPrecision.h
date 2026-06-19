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

/// Output mode for the high-precision IMU.
enum class EIMUOutputMode : uint8
{
  /// Use substep path when a valid physics proxy is available; else upsample.
  Auto,
  /// Always register the substep callback (falls back to frame-rate only when
  /// the proxy is absent).
  Substep,
  /// Never register the substep callback; emit N ZOH copies of the frame-rate
  /// value to maintain the target output_rate_hz.
  Upsample,
};

/// High-precision IMU. When substep mode is active and a valid Chaos vehicle
/// physics proxy is available, the gyroscope and accelerometer are computed and
/// published once per physics substep (each with its own timestamp), driven by
/// a PostIntegrate sim callback that captures the vehicle body kinematics on
/// the physics thread. In upsample mode (or when no proxy is available), the
/// frame-rate value is emitted N times per frame (zero-order hold) to maintain
/// the target output_rate_hz. Noise/bias model, compass, serialization and ROS2
/// publishing are reused from the base class.
UCLASS()
class CARLA_API AInertialMeasurementUnitHighPrecision : public AInertialMeasurementUnit
{
  GENERATED_BODY()

public:

  AInertialMeasurementUnitHighPrecision(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  virtual carla::geom::Vector3D ComputeGyroscope() override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  /// Publish a single sample (ROS2 + streaming) with a per-sample timestamp
  /// offset (seconds before the frame stamp). Public so that the file-local
  /// ZOH helper can call it without requiring a friend declaration.
  void PublishSubstepSample(
      double TimeOffsetSeconds,
      const carla::geom::Vector3D &Accelerometer,
      const carla::geom::Vector3D &Gyroscope,
      float Compass);

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

  // ---- Output mode and rate (set from blueprint attributes) ----

  /// Configured mode (from substep_mode attribute). Auto selects substep when
  /// a valid proxy is available, otherwise falls back to upsample.
  EIMUOutputMode ConfiguredMode = EIMUOutputMode::Auto;

  /// Target publish rate in Hz for the upsample (ZOH) path.
  float OutputRateHz = 200.0f;

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
};
