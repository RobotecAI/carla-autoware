// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"

// UE5 engine/Chaos headers use the (no-op) UE header guard in this repo, not
// the LibCarla disable-ue4-macros block (see CustomTerrainPhysicsComponent.cpp,
// MeshToLandscape.cpp). disable-ue4-macros is reserved for LibCarla / 3rd-party.
#include <util/ue-header-guard-begin.h>
#include "Chaos/SimCallbackObject.h"
#include "Chaos/SimCallbackInput.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include <util/ue-header-guard-end.h>

/// One per-substep raw kinematics sample of the vehicle body, captured on the
/// physics thread. Both the pre-integrate (X/R) and the post-integrate (P/Q)
/// quantities are kept so the game thread can confirm which pair carries the
/// up-to-date integrated transform at PostIntegrate time (see the diagnostic in
/// AInertialMeasurementUnitHighPrecision::PostPhysTick). IMU math is done on the
/// game thread; nothing here touches CARLA sensor state.
struct FIMUSubstepSample
{
  FVector PosX = FVector::ZeroVector;   // GetX() (pre-integrate position)
  FVector PosP = FVector::ZeroVector;   // GetP() (post-integrate position, expected up-to-date)
  FQuat   RotR = FQuat::Identity;       // GetR() (pre-integrate rotation)
  FQuat   RotQ = FQuat::Identity;       // GetQ() (post-integrate rotation, expected up-to-date)
  FVector LinVel = FVector::ZeroVector; // GetV()
  FVector AngVel = FVector::ZeroVector; // GetW()
  float   Dt = 0.0f;                    // substep dt (seconds)
};

/// Game -> physics input: the vehicle body proxy to sample each substep.
struct FIMUSubstepInput : public Chaos::FSimCallbackInput
{
  Chaos::FSingleParticlePhysicsProxy* Proxy = nullptr;
  void Reset() { Proxy = nullptr; }
};

/// Physics -> game output: kinematics accumulated over the substeps of ONE
/// physics step (one output instance per step; appended substep-by-substep).
struct FIMUSubstepOutput : public Chaos::FSimCallbackOutput
{
  TArray<FIMUSubstepSample> Samples;
  void Reset() { Samples.Reset(); }
};

/// Fires once per physics substep on the physics thread (PostIntegrate);
/// captures raw vehicle body kinematics into the per-step output array.
/// NOTE: the Presimulate option bit is REQUIRED in addition to PostIntegrate.
/// The Chaos solver only adds a callback to its SimCallbackObjects list when it
/// has the Presimulate option (PBDRigidsSolver.cpp: SimCallbackObjects.Add gated
/// on HasOption(Presimulate)); the per-substep PostIntegrate lambda then iterates
/// that same list. Without Presimulate the object is never registered and
/// OnPostIntegrate_Internal never fires (the sensor falls back to frame rate).
class FIMUSubstepCallback : public Chaos::TSimCallbackObject<
    FIMUSubstepInput,
    FIMUSubstepOutput,
    Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PostIntegrate>
{
public:
  virtual FName GetFNameForStatId() const override
  {
    const static FLazyName StaticName("FIMUSubstepCallback");
    return StaticName;
  }
private:
  virtual void OnPostIntegrate_Internal() override;
};
