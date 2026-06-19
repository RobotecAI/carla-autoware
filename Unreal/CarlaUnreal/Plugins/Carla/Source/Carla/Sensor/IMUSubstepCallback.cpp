// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/IMUSubstepCallback.h"

void FIMUSubstepCallback::OnPostIntegrate_Internal()
{
  const FIMUSubstepInput* In = GetConsumerInput_Internal();
  if (In == nullptr || In->Proxy == nullptr)
  {
    return;
  }

  // Gate on a valid physics-thread API (the proxy must have a live handle).
  if (In->Proxy->GetPhysicsThreadAPI() == nullptr)
  {
    return;
  }

  // Read the post-integrate kinematics from the low-level rigid particle
  // handle. GetP()/GetQ() (post-integrate, "predicted") and GetX()/GetR()
  // (pre-integrate) only exist on the rigid particle handle, not on the
  // FRigidBodyHandle_Internal proxy API; this mirrors how the engine reads
  // up-to-date P/Q during post-solve (PhysicsReplicationCache.cpp).
  Chaos::FGeometryParticleHandle* GtHandle = In->Proxy->GetHandle_LowLevel();
  Chaos::FPBDRigidParticleHandle* Rigid =
      (GtHandle != nullptr) ? GtHandle->CastToRigidParticle() : nullptr;
  if (Rigid == nullptr)
  {
    return;
  }

  FIMUSubstepOutput& Out = GetProducerOutputData_Internal();
  FIMUSubstepSample S;
  S.PosX   = FVector(Rigid->GetX());
  S.PosP   = FVector(Rigid->GetP());
  S.RotR   = FQuat(Rigid->GetR());
  S.RotQ   = FQuat(Rigid->GetQ());
  S.LinVel = FVector(Rigid->GetV());
  S.AngVel = FVector(Rigid->GetW());
  S.Dt     = static_cast<float>(GetDeltaTime_Internal());
  Out.Samples.Add(S);
}
