// Copyright (c) 2026 TIER IV, Inc.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "VehicleAccelerationControl.generated.h"

/// Component that controls the actor with a constant acceleration (directly applied,
/// no pedal input). Similar to VehicleVelocityControl but for acceleration.
UCLASS(Blueprintable, BlueprintType, ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class CARLA_API UVehicleAccelerationControl : public UActorComponent
{
  GENERATED_BODY()

  // ===========================================================================
  /// @name Constructor and destructor
  // ===========================================================================
  /// @{
public:
  UVehicleAccelerationControl();

  /// @}
  // ===========================================================================
  /// @name Get functions
  // ===========================================================================
  /// @{
public:

  void BeginPlay() override;

  virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

  // Activate the component setting the target acceleration
  virtual void Activate(bool bReset=false) override;

  // Activate the component setting the target acceleration (in vehicle local space, e.g. X=forward)
  virtual void Activate(FVector Acceleration, bool bReset=false);

  // Update the target acceleration without resetting internal state.
  // Acceleration is in vehicle local space. Units: cm/s^2.
  void SetTargetAcceleration(const FVector& Acceleration);

  // Set jerk limits (rate-of-change of acceleration) for each component.
  // Units: cm/s^3. Use <= 0 to disable the corresponding limit.
  void SetJerkLimit(float InJerkLimitPosCmps3, float InJerkLimitNegCmps3);

  // Set first-order lag time constant on the acceleration target. Units: seconds. Use <= 0 to disable.
  void SetFirstOrderLagTau(float InTauS) { TargetLagTauS = InTauS; }

  // Deactivate the component
  virtual void Deactivate() override;

private:
  // Desired target acceleration in vehicle local space (e.g. X forward, Y right, Z up). Units: cm/s^2
  UPROPERTY(Category = "Vehicle Acceleration Control", VisibleAnywhere)
  FVector DesiredTargetAcceleration;

  // Filtered target acceleration after first-order lag. Units: cm/s^2.
  FVector FilteredTargetAcceleration;

  // Acceleration actually applied after jerk limiting. Units: cm/s^2.
  FVector AppliedAcceleration;

  // Maximum jerk (acceleration rate) limits. Units: cm/s^3.
  // Note: Neg limit is a positive magnitude used when the acceleration decreases (diff < 0).
  UPROPERTY(Category = "Vehicle Acceleration Control", EditAnywhere)
  float JerkLimitPosCmps3 = 300.0f; // default: 3.0 m/s^3

  UPROPERTY(Category = "Vehicle Acceleration Control", EditAnywhere)
  float JerkLimitNegCmps3 = 500.0f; // default: 5.0 m/s^3

  /// Integrated forward speed [cm/s] under our control; lateral component is left to physics for cornering
  float ControlledForwardSpeed;

  UPrimitiveComponent* PrimitiveComponent;
  AActor* OwnerVehicle;

  // First-order lag time constant for target acceleration. Units: seconds. Use <= 0 to disable.
  float TargetLagTauS = 0.0f;

};
