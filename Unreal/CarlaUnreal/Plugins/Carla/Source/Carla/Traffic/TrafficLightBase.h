// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Traffic/TrafficSignBase.h"

#include "Traffic/TrafficLightState.h"
#include "Traffic/TrafficLightComponent.h"

#include "TrafficLightBase.generated.h"

class ACarlaWheeledVehicle;
class AWheeledVehicleAIController;

UCLASS()
class CARLA_API ATrafficLightBase : public ATrafficSignBase
{

  GENERATED_BODY()

public:

  ATrafficLightBase(const FObjectInitializer &ObjectInitializer);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  ETrafficLightState GetTrafficLightState() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetTrafficLightState(ETrafficLightState State);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void NotifyWheeledVehicle(ACarlaWheeledVehicle *Vehicle);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void UnNotifyWheeledVehicle(ACarlaWheeledVehicle *Vehicle);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetGreenTime(float InGreenTime);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  float GetGreenTime() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetYellowTime(float InYellowTime);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  float GetYellowTime() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetRedTime(float InRedTime);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  float GetRedTime() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  float GetElapsedTime() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetTimeIsFrozen(bool InTimeIsFrozen);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  bool GetTimeIsFrozen() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetPoleIndex(int InPoleIndex);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  int GetPoleIndex() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  TArray<ATrafficLightBase *> GetGroupTrafficLights() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetGroupTrafficLights(TArray<ATrafficLightBase *> InGroupTrafficLights);

  // --- Arrow sections (lanelet2-driven JP arrow lights) ----------------------
  // (color x direction) bitmask, see carla/rpc/TrafficLightArrowState.h.

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  void SetArrowState(int32 InArrowState);

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  int32 GetArrowState() const;

  UFUNCTION(Category = "Traffic Light", BlueprintCallable)
  int32 GetArrowCapabilities() const;

  // used from replayer
  void SetElapsedTime(float InElapsedTime);

  UFUNCTION(Category = "Traffic Light", BlueprintPure)
  UTrafficLightComponent* GetTrafficLightComponent();

  const UTrafficLightComponent* GetTrafficLightComponent() const;

  // Compatibility old traffic light system with traffic light components
  void LightChangedCompatibility(ETrafficLightState NewLightState);

  void AddTimeToRecorder();

protected:

  virtual void BeginPlay() override;

  UFUNCTION(Category = "Traffic Light", BlueprintImplementableEvent)
  void OnTrafficLightStateChanged(ETrafficLightState TrafficLightState);

  /// Mirrors OnTrafficLightStateChanged for the arrow mask.
  UFUNCTION(Category = "Traffic Light", BlueprintImplementableEvent)
  void OnArrowStateChanged(int32 NewArrowState);

private:

  /// (color x direction) bitmask, see carla/rpc/TrafficLightArrowState.h.
  /// Initial value is stamped by the lanelet2 editor placer (lanelet2 green
  /// arrows = the constant-on default look) and persists with the level.
  UPROPERTY(Category = "Traffic Light", EditAnywhere)
  int32 ArrowState = 0;

  /// Bitmask of arrow faces physically present on the mesh (placer-stamped).
  /// requested & ~capabilities = bits that cannot light (no face).
  UPROPERTY(Category = "Traffic Light", EditAnywhere)
  int32 ArrowCapabilities = 0;

  UPROPERTY(Category = "Traffic Light", VisibleAnywhere)
  TArray<TObjectPtr<AWheeledVehicleAIController>> Vehicles;

  UPROPERTY(Category = "Traffic Light", VisibleAnywhere)
  int PoleIndex = 0;

  UPROPERTY(Category = "Traffic Light", VisibleAnywhere)
  TArray<TObjectPtr<ATrafficLightBase>> GroupTrafficLights;

  UPROPERTY(Category = "Traffic Light", EditAnywhere)
  TObjectPtr<UTrafficLightComponent> TrafficLightComponent = nullptr;
};
