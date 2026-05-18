// Copyright (c) 2026 TIER IV, Inc.

// TODO: Add gear-aware input (e.g. allow reverse when in reverse gear, keep forward-only in drive).

#include "VehicleAccelerationControl.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

UVehicleAccelerationControl::UVehicleAccelerationControl()
{
  PrimaryComponentTick.bCanEverTick = true;
}

void UVehicleAccelerationControl::BeginPlay()
{
  Super::BeginPlay();

  SetComponentTickEnabled(false);
  // Run after physics so our velocity override is applied after the vehicle simulation
  SetTickGroup(ETickingGroup::TG_PostPhysics);

  OwnerVehicle = GetOwner();
  ACarlaWheeledVehicle* CarlaVehicle = Cast<ACarlaWheeledVehicle>(OwnerVehicle);
  if (CarlaVehicle && CarlaVehicle->GetMesh())
  {
    PrimitiveComponent = Cast<UPrimitiveComponent>(CarlaVehicle->GetMesh());
  }
  if (PrimitiveComponent == nullptr)
  {
    PrimitiveComponent = Cast<UPrimitiveComponent>(OwnerVehicle->GetRootComponent());
  }
  ControlledForwardSpeed = 0.f;
}

void UVehicleAccelerationControl::Activate(bool bReset)
{
  Super::Activate(bReset);

  DesiredTargetAcceleration = FVector();
  FilteredTargetAcceleration = FVector();
  AppliedAcceleration = FVector();
  SetComponentTickEnabled(true);
}

void UVehicleAccelerationControl::Activate(FVector Acceleration, bool bReset)
{
  Super::Activate(bReset);

  DesiredTargetAcceleration = Acceleration;
  FilteredTargetAcceleration = Acceleration;
  AppliedAcceleration = Acceleration;
  if (PrimitiveComponent != nullptr)
  {
    const FVector Vel = PrimitiveComponent->GetPhysicsLinearVelocity();
    const FVector Forward = OwnerVehicle->GetActorTransform().TransformVectorNoScale(FVector(1, 0, 0));
    const FVector ForwardDir = Forward.GetSafeNormal();
    ControlledForwardSpeed = FVector::DotProduct(Vel, ForwardDir);
  }
  else
  {
    ControlledForwardSpeed = 0.f;
  }
  SetComponentTickEnabled(true);
}

void UVehicleAccelerationControl::SetTargetAcceleration(const FVector& Acceleration)
{
  DesiredTargetAcceleration = Acceleration;
  // Do not reset AppliedAcceleration or ControlledForwardSpeed here; jerk limiting
  // will move AppliedAcceleration towards the (possibly filtered) target over time.
}

void UVehicleAccelerationControl::SetJerkLimit(float InJerkLimitPosCmps3, float InJerkLimitNegCmps3)
{
  JerkLimitPosCmps3 = InJerkLimitPosCmps3;
  JerkLimitNegCmps3 = InJerkLimitNegCmps3;
}

void UVehicleAccelerationControl::Deactivate()
{
  SetComponentTickEnabled(false);
  Super::Deactivate();
}

void UVehicleAccelerationControl::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(UVehicleAccelerationControl::TickComponent);
  if (PrimitiveComponent == nullptr)
  {
    return;
  }
  PrimitiveComponent->WakeRigidBody(NAME_None);

  const FTransform Transf = OwnerVehicle->GetActorTransform();
  const FVector ForwardDir = Transf.TransformVectorNoScale(FVector(1, 0, 0)).GetSafeNormal();

  // First-order lag (exponential smoothing) on the target acceleration in vehicle-local space.
  // alpha = 1 - exp(-dt/tau)
  FVector TargetAcceleration = DesiredTargetAcceleration;
  if (DeltaTime > 0.0f && TargetLagTauS > 0.0f)
  {
    const float Alpha = 1.0f - FMath::Exp(-DeltaTime / TargetLagTauS);
    FilteredTargetAcceleration = FilteredTargetAcceleration + Alpha * (DesiredTargetAcceleration - FilteredTargetAcceleration);
    TargetAcceleration = FilteredTargetAcceleration;
  }
  else
  {
    FilteredTargetAcceleration = DesiredTargetAcceleration;
  }

  // Apply jerk limits (rate limit on acceleration command) in vehicle-local space.
  if (DeltaTime > 0.0f)
  {
    const FVector Diff = TargetAcceleration - AppliedAcceleration;
    const float MaxInc = (JerkLimitPosCmps3 > 0.0f) ? (JerkLimitPosCmps3 * DeltaTime) : TNumericLimits<float>::Max();
    const float MaxDec = (JerkLimitNegCmps3 > 0.0f) ? (JerkLimitNegCmps3 * DeltaTime) : TNumericLimits<float>::Max();
    AppliedAcceleration.X += FMath::Clamp(Diff.X, -MaxDec, MaxInc);
    AppliedAcceleration.Y += FMath::Clamp(Diff.Y, -MaxDec, MaxInc);
    AppliedAcceleration.Z += FMath::Clamp(Diff.Z, -MaxDec, MaxInc);
  }
  else
  {
    AppliedAcceleration = TargetAcceleration;
  }

  const FVector WorldAcceleration = Transf.TransformVector(AppliedAcceleration);
  const float ForwardAccel = FVector::DotProduct(WorldAcceleration, ForwardDir);

  // Integrate only forward speed; preserve lateral velocity from physics so tires can generate cornering force
  ControlledForwardSpeed += ForwardAccel * DeltaTime;
  // Clamp to non-negative so we never go into reverse
  // TODO: Add gear-aware input (e.g. allow reverse when in reverse gear, keep forward-only in drive).
  ControlledForwardSpeed = FMath::Max(0.f, ControlledForwardSpeed);

  const FVector CurrentVel = PrimitiveComponent->GetPhysicsLinearVelocity();
  const FVector LateralVel = CurrentVel - FVector::DotProduct(CurrentVel, ForwardDir) * ForwardDir;
  const FVector NewVel = (ControlledForwardSpeed * ForwardDir) + LateralVel;

  PrimitiveComponent->SetPhysicsLinearVelocity(NewVel, false, "None");
}
