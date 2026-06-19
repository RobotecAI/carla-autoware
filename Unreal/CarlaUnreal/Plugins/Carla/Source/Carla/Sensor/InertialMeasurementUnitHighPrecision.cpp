// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/InertialMeasurementUnitHighPrecision.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaStatics.h"

// UE5 engine/physics headers: repo convention uses the (no-op) UE header guard.
#include <util/ue-header-guard-begin.h>
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PBDRigidsSolver.h" // complete Chaos::FPBDRigidsSolver for *SimCallbackObject_External
#include <util/ue-header-guard-end.h>

// LibCarla headers: disable-ue4-macros block.
#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <util/enable-ue4-macros.h>

AInertialMeasurementUnitHighPrecision::AInertialMeasurementUnitHighPrecision(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
}

FActorDefinition AInertialMeasurementUnitHighPrecision::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeIMUHighPrecisionDefinition();
}

void AInertialMeasurementUnitHighPrecision::Set(const FActorDescription &ActorDescription)
{
  // Base class reads noise/bias attributes.
  Super::Set(ActorDescription);

  // Read high-precision-specific attributes.
  const FString ModeStr = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("substep_mode"), ActorDescription.Variations, TEXT("auto"));
  if (ModeStr.Equals(TEXT("substep"), ESearchCase::IgnoreCase))
  {
    ConfiguredMode = EIMUOutputMode::Substep;
  }
  else if (ModeStr.Equals(TEXT("upsample"), ESearchCase::IgnoreCase))
  {
    ConfiguredMode = EIMUOutputMode::Upsample;
  }
  else
  {
    ConfiguredMode = EIMUOutputMode::Auto; // "auto" or unrecognised
  }

  OutputRateHz = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("output_rate_hz"), ActorDescription.Variations, 200.0f);
}

// =========================================================================
// Phase 1 frame-rate gyro (fallback path, used when substep is unavailable).
// =========================================================================

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

// =========================================================================
// Substep callback registration / teardown.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::RegisterSubstepCallback()
{
  // In upsample mode we intentionally skip registering the substep callback.
  if (ConfiguredMode == EIMUOutputMode::Upsample)
  {
    return;
  }

  UWorld* World = GetWorld();
  if (World == nullptr) return;
  FPhysScene_Chaos* Scene = static_cast<FPhysScene_Chaos*>(World->GetPhysicsScene());
  if (Scene == nullptr) return;
  Chaos::FPhysicsSolver* Solver = Scene->GetSolver();
  if (Solver == nullptr) return;

  // Owner vehicle root primitive -> physics proxy.
  AActor* Owner = GetOwner();
  UPrimitiveComponent* Root = (Owner != nullptr)
      ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
  FBodyInstance* BI = (Root != nullptr) ? Root->GetBodyInstance() : nullptr;
  VehicleProxy = (BI != nullptr)
      ? static_cast<Chaos::FSingleParticlePhysicsProxy*>(BI->GetPhysicsActorHandle())
      : nullptr;
  if (VehicleProxy == nullptr)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("imu_highprecision: no vehicle physics proxy; substep unavailable, "
             "falling back to upsample ZOH path"));
    return;
  }
  SubstepCallback = Solver->CreateAndRegisterSimCallbackObject_External<FIMUSubstepCallback>();
}

void AInertialMeasurementUnitHighPrecision::UnregisterSubstepCallback()
{
  if (SubstepCallback == nullptr) return;
  UWorld* World = GetWorld();
  FPhysScene_Chaos* Scene = (World != nullptr)
      ? static_cast<FPhysScene_Chaos*>(World->GetPhysicsScene()) : nullptr;
  if (Scene != nullptr && Scene->GetSolver() != nullptr)
  {
    Scene->GetSolver()->UnregisterAndFreeSimCallbackObject_External(SubstepCallback);
  }
  SubstepCallback = nullptr;
}

void AInertialMeasurementUnitHighPrecision::EndPlay(EEndPlayReason::Type EndPlayReason)
{
  UnregisterSubstepCallback();
  Super::EndPlay(EndPlayReason);
}

// =========================================================================
// Per-substep publish helper (ROS2 + streaming) with per-substep timestamp.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::PublishSubstepSample(
    double TimeOffsetSeconds,
    const carla::geom::Vector3D &Accelerometer,
    const carla::geom::Vector3D &Gyroscope,
    float Compass)
{
  auto DataStream = GetDataStream(*this);

  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent =
          GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromIMUStamped(
          TimeOffsetSeconds, DataStream.GetSensorType(), StreamId,
          LocalTransformRelativeToParent, Accelerometer, Gyroscope, Compass, this);
    }
    else
    {
      ROS2->ProcessDataFromIMUStamped(
          TimeOffsetSeconds, DataStream.GetSensorType(), StreamId,
          DataStream.GetSensorTransform(), Accelerometer, Gyroscope, Compass, this);
    }
  }
  #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnitHighPrecision::SerializeAndSend);
    DataStream.SerializeAndSend(*this, Accelerometer, Gyroscope, Compass);
  }
}

// =========================================================================
// ZOH upsample helper: emits N copies of a frame-rate IMU value.
// =========================================================================

static void EmitZOHSamples(
    AInertialMeasurementUnitHighPrecision* Sensor,
    float OutputRateHz,
    float DeltaTime)
{
  const int32 N = FMath::Max(1, FMath::RoundToInt(OutputRateHz * DeltaTime));
  const carla::geom::Vector3D G = Sensor->ComputeGyroscope();
  const carla::geom::Vector3D A = Sensor->ComputeAccelerometer(
      Sensor->GetWorld()->GetTimeSeconds());
  const float Compass = Sensor->ComputeCompass();
  for (int32 k = 0; k < N; ++k)
  {
    // Timestamps are evenly spaced across the frame ending at the frame stamp.
    // Sample k=0 is the oldest (furthest before the frame stamp),
    // sample k=N-1 is the most recent (at the frame stamp).
    const double tOffset =
        static_cast<double>(N - 1 - k) * static_cast<double>(DeltaTime) / N;
    Sensor->PublishSubstepSample(tOffset, A, G, Compass);
  }
}

// =========================================================================
// PostPhysTick: mode-dispatch to substep path, upsample ZOH path, or fallback.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::PostPhysTick(
    UWorld* World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnitHighPrecision::PostPhysTick);

  // Upsample-only mode: skip substep registration entirely; emit ZOH samples.
  if (ConfiguredMode == EIMUOutputMode::Upsample)
  {
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Auto / Substep mode: try to register the substep callback on first tick.
  if (SubstepCallback == nullptr && VehicleProxy == nullptr)
  {
    RegisterSubstepCallback();
  }

  // No callback registered (no valid proxy): fall back to ZOH upsample.
  // The base PrevLocation/PrevTime will be seeded by ComputeAccelerometer on
  // the first call; the base NaN guard handles the first-sample transition.
  if (SubstepCallback == nullptr)
  {
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Feed the current proxy to the physics thread for the next step.
  if (FIMUSubstepInput* InData = SubstepCallback->GetProducerInputData_External())
  {
    InData->Proxy = VehicleProxy;
  }

  // Drain all substep samples accumulated since the last frame.
  TArray<FIMUSubstepSample> Frame;
  while (Chaos::TSimCallbackOutputHandle<FIMUSubstepOutput> Out =
             SubstepCallback->PopFutureOutputData_External())
  {
    Frame.Append(Out->Samples);
  }

  const int32 N = Frame.Num();

  // No samples this frame (e.g. first frame before the physics thread has fired,
  // or a brief transition out of substep). The base PrevLocation/PrevTime become
  // stale when we were in the substep path last frame, which would cause a
  // one-frame spike in the base accelerometer. Fall back to ZOH upsample;
  // the base NaN/IsFinite guard will re-seed on the next valid call.
  if (N == 0)
  {
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Capture the mount (sensor relative to the vehicle body) once.
  if (!bMountCaptured)
  {
    const FTransform Rel = GetRootComponent()->GetRelativeTransform(); // parent = vehicle
    MountRelRot = Rel.GetRotation();
    MountOffsetLocal = Rel.GetLocation();
    bMountCaptured = true;
  }

  const ACarlaGameModeBase* GameMode = UCarlaStatics::GetGameMode(GetWorld());
  const float GRAVITY = (GameMode != nullptr) ? GameMode->IMUSensorGravity : -9.81f;
  // Convert UE's cm to meters (matches the base accelerometer convention).
  constexpr float TO_METERS = 1e-2f;

  // Compass uses the current frame value (low-rate orientation reference).
  const float Compass = ComputeCompass();

  // Precompute suffix sums of Dt for accurate per-substep timestamps.
  // SuffixDt[k] = sum of Frame[j].Dt for j = k+1 .. N-1.
  // This is the elapsed time from the END of substep k to the frame stamp,
  // which equals the per-substep time offset for sample k.
  TArray<double> SuffixDt;
  SuffixDt.SetNumUninitialized(N);
  SuffixDt[N - 1] = 0.0;
  for (int32 j = N - 2; j >= 0; --j)
  {
    SuffixDt[j] = SuffixDt[j + 1] + static_cast<double>(Frame[j + 1].Dt);
  }

  for (int32 k = 0; k < N; ++k)
  {
    const FIMUSubstepSample& S = Frame[k];

    // GetQ/GetP are the up-to-date post-integrate values at OnPostIntegrate_Internal time.
    const FQuat   BodyRot = S.RotQ;
    const FVector BodyPos = S.PosP;

    const FQuat   ImuRot = BodyRot * MountRelRot;
    const FVector ImuPos = BodyPos + BodyRot.RotateVector(MountOffsetLocal);

    carla::geom::Vector3D Gyro{};
    carla::geom::Vector3D Accel{};

    if (bSubstepPrevValid && S.Dt > 0.0f)
    {
      // Gyro: rotation finite difference (same scheme as the Phase 1 path).
      FQuat dq = ImuRot * PrevImuRot.Inverse();
      FVector Axis;
      float Angle;
      dq.ToAxisAndAngle(Axis, Angle);
      if (Angle > PI)
      {
        Angle -= 2.0f * PI;
      }
      FVector wWorld = (Angle / S.Dt) * Axis;
      FVector wLocal = ImuRot.UnrotateVector(wWorld);
      if (wLocal.ContainsNaN())
      {
        wLocal = FVector::ZeroVector;
      }
      Gyro = ComputeGyroscopeNoise(wLocal);

      // Accel: 2nd derivative (quadratic interpolation, same form as the base)
      // evaluated at the IMU point, plus gravity (cm -> m).
      if (!PrevImuPos[0].ContainsNaN() && !PrevImuPos[1].ContainsNaN() &&
          FMath::IsFinite(PrevSubDt) && PrevSubDt > 0.0f)
      {
        const float H1 = S.Dt;
        const float H2 = PrevSubDt;
        const float H12 = H1 + H2;
        const FVector A = PrevImuPos[1] / (H1 * H2);
        const FVector B = PrevImuPos[0] / (H2 * H12);
        const FVector C = ImuPos / (H1 * H12);
        FVector aWorld = TO_METERS * -2.0f * (A - B - C);
        aWorld.Z += GRAVITY;
        FVector aLocal = ImuRot.UnrotateVector(aWorld);
        Accel = ComputeAccelerometerNoise(aLocal);
      }
    }

    // Per-substep timestamp: use accumulated suffix dt for accuracy with
    // non-uniform substep sizes (robust to variable substep dt).
    PublishSubstepSample(SuffixDt[k], Accel, Gyro, Compass);

    // Advance integration state.
    PrevImuRot = ImuRot;
    bSubstepPrevValid = true;
    PrevImuPos[0] = PrevImuPos[1];
    PrevImuPos[1] = ImuPos;
    PrevSubDt = S.Dt;
  }

  // Substep path published each sample with its own timestamp; do NOT call
  // Super::PostPhysTick (avoids a duplicate frame-rate publish).
}
