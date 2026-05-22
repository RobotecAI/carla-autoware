// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowareGameModeBase.h"

#include "AutowareWorldSettings.h"

#include <carla/geom/GeoProjection.h>
#include <carla/geom/GeoProjectionsParams.h>

AAutowareGameModeBase::AAutowareGameModeBase(const FObjectInitializer& ObjectInitializer)
	:Super(ObjectInitializer)
{
}

void AAutowareGameModeBase::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
}

void AAutowareGameModeBase::LoadGeoReference()
{
	auto* WS = Cast<AAutowareWorldSettings>(GetWorld()->GetWorldSettings());

	if (!IsValid(WS))
	{
		Super::LoadGeoReference(); // Fallback to CarlaGameMode::LoadGeoReference default
		return;
	}
	
	UE_LOG(LogCarla, Warning, TEXT("Autoware Settings fetch succeded."));
	auto* Data = WS->MgrsDataAssetSoftPtr.LoadSynchronous();

	if (!IsValid(Data))
	{
		return;
	}
	
	// Construct a Transverse Mercator projection centred on the MGRS reference
	// point. This preserves the prior T4 behaviour of treating the configured
	// lat/lon/alt as the origin for local UE-coordinates → geographic
	// transformations (replaces the removed GeoLocation::Transform Mercator
	// path; see upstream commit 8b3f3b207).
	carla::geom::TransverseMercatorParams TmParams(
		/* lat_0 */ Data->GeoReference.Latitude,
		/* lon_0 */ Data->GeoReference.Longitude,
		/* k     */ 1.0,
		/* x_0   */ 0.0,
		/* y_0   */ 0.0,
		/* ellps */ carla::geom::Ellipsoid{6378137.0, 298.257223563} // WGS84
	);
	Episode->MapGeoProjection = carla::geom::GeoProjection::Make(TmParams);

	UE_LOG(LogCarla, Warning, TEXT("MGRS Offset loaded successfuly."));
	StoreSpawnPoints();
}
