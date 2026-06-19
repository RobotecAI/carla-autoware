// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/InertialMeasurementUnitHighPrecision.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

AInertialMeasurementUnitHighPrecision::AInertialMeasurementUnitHighPrecision(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
}

FActorDefinition AInertialMeasurementUnitHighPrecision::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeIMUHighPrecisionDefinition();
}

// Angular velocity from the finite difference of the sensor's own world
// rotation, following AWSIM's ImuSensor.cs (physics-independent). The result
// is expressed in the sensor's local frame (CARLA: X-fwd, Y-right, Z-up) so
// that the reused CarlaIMUPublisher produces ROS output identical in
// convention to the standard IMU.
carla::geom::Vector3D AInertialMeasurementUnitHighPrecision::ComputeGyroscope()
{
  const float CurrentTime = GetWorld()->GetTimeSeconds();
  const FQuat CurrentRotation =
      GetRootComponent()->GetComponentTransform().GetRotation();

  // First valid frame: seed state and report zero.
  if (!FMath::IsFinite(PrevGyroTime))
  {
    PrevGyroTime = CurrentTime;
    PrevRotation = CurrentRotation;
    return carla::geom::Vector3D{};
  }

  const float DeltaTime = CurrentTime - PrevGyroTime;
  PrevGyroTime = CurrentTime;

  if (DeltaTime <= 0.0f)
  {
    PrevRotation = CurrentRotation;
    return carla::geom::Vector3D{};
  }

  const FQuat DeltaRotation = CurrentRotation * PrevRotation.Inverse();
  FVector Axis;
  float Angle;
  DeltaRotation.ToAxisAndAngle(Axis, Angle);

  // Take the short path: a tiny step must not be reported as ~2*pi the other way.
  if (Angle > PI)
  {
    Angle -= 2.0f * PI;
  }

  const FVector WorldAngularVelocity = (Angle / DeltaTime) * Axis;
  FVector LocalAngularVelocity =
      CurrentRotation.UnrotateVector(WorldAngularVelocity);

  PrevRotation = CurrentRotation;

  // Guard against NaN from a degenerate axis/angle decomposition.
  if (LocalAngularVelocity.ContainsNaN())
  {
    LocalAngularVelocity = FVector::ZeroVector;
  }

  // Reuse the base noise/bias model so behavior matches the standard IMU.
  return ComputeGyroscopeNoise(LocalAngularVelocity);
}
