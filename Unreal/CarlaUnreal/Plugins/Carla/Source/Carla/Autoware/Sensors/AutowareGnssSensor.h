// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/GnssSensor.h"
#include "Autoware/Data/MgrsDataAsset.h"
#include <carla/geom/GeoProjection.h>
#include "AutowareGnssSensor.generated.h"

UCLASS()
class CARLA_API AAutowareGnssSensor : public ASensor
{
	GENERATED_BODY()

public:
	AAutowareGnssSensor(const FObjectInitializer& ObjectInitializer);
	
	static FActorDefinition GetSensorDefinition();
	void Set(const FActorDescription &ActorDescription);

	void SetLatitudeDeviation(float Value);
	void SetLongitudeDeviation(float Value);
	void SetAltitudeDeviation(float Value);

	void SetLatitudeBias(float Value);
	void SetLongitudeBias(float Value);
	void SetAltitudeBias(float Value);

	float GetLatitudeDeviation() const;
	float GetLongitudeDeviation() const;
	float GetAltitudeDeviation() const;

	float GetLatitudeBias() const;
	float GetLongitudeBias() const;
	float GetAltitudeBias() const;

	double GetLatitudeValue() const;
	double GetLongitudeValue() const;
	double GetAltitudeValue() const;

	UFUNCTION(BlueprintCallable)
	bool IsNoiseErrorEnabled() const;
	
	UFUNCTION(BlueprintCallable)
	void LoadMgrsData();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// virtual void PrePhysTick(float DeltaSeconds) override;
	virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;
	
	UFUNCTION(BlueprintCallable)
	void SetNoiseErrorActive(const bool bEnabled);

private:
	UPROPERTY()
	TObjectPtr<UMgrsDataAsset> MgrsDataAsset = nullptr;
	
	carla::geom::GeoProjection CurrentGeoProjection;

	bool bApplyNoiseError = false;

	float LatitudeDeviation;
	float LongitudeDeviation;
	float AltitudeDeviation;

	float LatitudeBias;
	float LongitudeBias;
	float AltitudeBias;

	double LatitudeValue;
	double LongitudeValue;
	double AltitudeValue;
};