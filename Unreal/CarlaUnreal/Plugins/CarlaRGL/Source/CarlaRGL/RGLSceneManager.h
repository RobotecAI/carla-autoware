// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization manager: extracts UE5 mesh geometry and
// uploads it to the RobotecGPULidar (RGL) scene for GPU ray tracing.
//
// Design:
//   - Singleton per UWorld (accessed via GetInstance)
//   - Caches UStaticMesh* -> rgl_mesh_t to avoid re-uploading identical meshes
//   - Creates rgl_entity_t for each UStaticMeshComponent instance
//   - Updates entity transforms each tick
//   - Handles actor creation/destruction via periodic world scans

#pragma once

#ifdef WITH_RGL

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include <util/ue-header-guard-end.h>

// Forward declarations
class UWorld;
class AActor;

/// Manages synchronization between the UE5 scene and the RGL scene.
/// Call Update() each tick before running the RGL raytrace graph.
class FRGLSceneManager
{
public:
    ~FRGLSceneManager();

    /// Get the singleton instance for the given world.
    /// Creates the instance on first call. Cleans up stale instances.
    static FRGLSceneManager& GetInstance(UWorld* World);

    /// Destroy the singleton instance (call on world teardown).
    static void DestroyInstance(UWorld* World);

    /// Get the RGL scene handle. Pass to rgl_node_raytrace().
    /// Returns nullptr which RGL treats as the default scene.
    rgl_scene_t GetScene() const { return Scene; }

    /// Synchronize UE5 world geometry with RGL scene.
    /// Should be called each frame before raytrace.
    /// @param SimulationTime Elapsed simulation time in seconds (from episode start).
    ///        Used for RGL scene time and ROS2 timestamp synchronization.
    ///        Pass -1.0 to fall back to World->GetTimeSeconds().
    void Update(UWorld* World, double SimulationTime = -1.0);

    /// Update ground plane position to follow the sensor.
    /// Uses a UE5 line trace to detect actual ground level.
    void UpdateGroundPlane(UWorld* World, const FTransform& SensorTransform);

    /// Register or refresh a sensor's culling sphere. Call each tick before Update().
    /// SensorId must be stable for the sensor's lifetime (use the session handle).
    /// Now = current simulation time (seconds), used for stale-sensor eviction.
    void RegisterSensor(const void* SensorId, const FVector& Position, float DistanceCm, double Now)
    {
        FSensorRegistration& S = RegisteredSensors.FindOrAdd(SensorId);
        S.Position = Position;
        S.RegistrationDistanceCm = FMath::Max(5000.0f, DistanceCm);
        S.LastUpdateTime = Now;
    }

    /// Remove a sensor when its session is destroyed.
    void UnregisterSensor(const void* SensorId)
    {
        RegisteredSensors.Remove(SensorId);
    }

    /// Set the world sync interval in simulation seconds. Default: 1.0s. Minimum: 0.1s.
    void SetSyncInterval(float Seconds) { SyncIntervalSeconds = FMath::Max(0.1f, Seconds); }

    /// Check if the scene manager has been initialized.
    bool IsInitialized() const { return bInitialized; }

private:
    FRGLSceneManager();

    // Non-copyable
    FRGLSceneManager(const FRGLSceneManager&) = delete;
    FRGLSceneManager& operator=(const FRGLSceneManager&) = delete;

    /// Initial scan of the world to upload all static meshes.
    void InitializeFromWorld(UWorld* World);

    /// Upload a UStaticMesh to RGL (vertex + index data).
    /// Returns nullptr on failure (mesh skipped silently).
    rgl_mesh_t UploadMesh(UStaticMesh* StaticMesh);

    /// Create an RGL entity for a static mesh component. Returns true on success.
    bool RegisterComponent(UStaticMeshComponent* Component);

    /// Filter components before uploading them to RGL.
    bool ShouldRegisterComponent(UStaticMeshComponent* Component, bool& bOutOfRange) const;

    /// Check whether an already registered component should stay in the active RGL scene.
    bool ShouldKeepRegisteredComponent(UStaticMeshComponent* Component) const;

    /// Union distance culling helpers (within ANY registered sensor's sphere).
    bool IsWithinAnySensor(const FVector& CenterCm, float BoundsRadiusCm, float ExtraCm) const;
    bool IsComponentWithinAnySensor(UStaticMeshComponent* Component, float ExtraCm) const;
    bool IsInstanceWithinAnySensor(
        UInstancedStaticMeshComponent* Component,
        UStaticMesh* StaticMesh,
        const FTransform& InstanceTransform,
        float ExtraCm) const;

    /// Remove an RGL entity for a destroyed component.
    void UnregisterComponent(UStaticMeshComponent* Component);

    /// Register all instances of an ISMC as separate RGL entities.
    bool RegisterISMComponent(UInstancedStaticMeshComponent* Component);

    /// Remove all RGL entities for an ISMC.
    void UnregisterISMComponent(UInstancedStaticMeshComponent* Component);

    /// Track active RGL entity references to cached meshes and free unused GPU meshes.
    void AddMeshReference(UStaticMesh* StaticMesh, int32 Count = 1);
    void ReleaseMeshReference(UStaticMesh* StaticMesh, int32 Count = 1);

    /// Update all entity transforms to match current UE5 state.
    void UpdateTransforms();

    /// Scan world for new/removed components since last update.
    void SyncWorldComponents(UWorld* World);

    /// Create a large ground plane in the RGL scene to represent the road surface.
    void CreateGroundPlane();

    // ---- Data ----

    /// RGL scene handle (nullptr = default scene).
    rgl_scene_t Scene = nullptr;

    /// Cache: UStaticMesh* -> rgl_mesh_t (avoids duplicate uploads).
    /// Uses TMap for UE5 compatibility and safe iteration.
    TMap<UStaticMesh*, rgl_mesh_t> MeshCache;

    /// Active entity references per cached mesh. Meshes are released when this reaches zero.
    TMap<UStaticMesh*, int32> MeshRefCounts;

    /// Mapping: UStaticMeshComponent* -> RGL entity + weak reference.
    struct FEntityInfo
    {
        rgl_entity_t Entity = nullptr;
        TWeakObjectPtr<UStaticMeshComponent> Component;
        UStaticMesh* StaticMesh = nullptr;     // Mesh referenced by Entity; used for cache ref-counting.
        bool bIsStatic = false;            // True if Mobility == Static (never moves)
        bool bTransformInitialized = false; // True after first transform set
        bool bInRange = true;              // Distance culling state
        FTransform LastTransform;           // Cached for change detection
    };
    TMap<UStaticMeshComponent*, FEntityInfo> EntityMap;

    /// ISMC entity group: one ISMC component -> N RGL entities (one per instance).
    struct FISMCEntityGroup
    {
        TArray<rgl_entity_t> Entities;
        TWeakObjectPtr<UInstancedStaticMeshComponent> Component;
        UStaticMesh* StaticMesh = nullptr; // Mesh referenced by Entities; used for cache ref-counting.
        int32 OriginalInstanceCount = 0;  // Instance count at registration time (may differ from Entities.Num() due to skipped instances)
        int32 MeshRefCount = 0;           // Number of live RGL entities referencing StaticMesh.
    };
    TMap<UInstancedStaticMeshComponent*, FISMCEntityGroup> ISMCEntityMap;

    bool bInitialized = false;

    /// Frame tracking — skip Update if already done this frame
    uint64 LastUpdateFrame = 0;

    /// Per-sensor culling registration. A static mesh is registered if it falls
    /// within ANY active sensor's sphere (union), so spatially separated sensors
    /// (multi-vehicle / roadside) are all served correctly by the single shared scene.
    struct FSensorRegistration
    {
        FVector Position = FVector::ZeroVector;
        float   RegistrationDistanceCm = 30000.0f; // per-sensor radius (UE5 cm)
        FVector LastSyncPosition = FVector::ZeroVector;
        bool    bLastSyncValid = false;
        double  LastUpdateTime = 0.0;              // sim time of last RegisterSensor()
    };
    /// Key = sensor id (session handle). Pointer key uses pointer hashing.
    TMap<const void*, FSensorRegistration> RegisteredSensors;

    /// Hysteresis for dynamic unloads to avoid add/remove churn around the boundary (UE5 cm).
    float UnregistrationHysteresisCm = 2000.0f; // 20m

    /// Evict sensors that stopped ticking (paused but not destroyed) to free their VRAM.
    float SensorStaleTimeoutSeconds = 5.0f;

    /// Latest simulation time seen by Update(); used for stale-sensor eviction.
    double CurrentSimTime = 0.0;

    /// Virtual ground plane (not tied to any UE5 component).
    rgl_mesh_t GroundMesh = nullptr;
    rgl_entity_t GroundEntity = nullptr;

    /// Time-based periodic sync interval (simulation seconds).
    float SyncIntervalSeconds = 1.0f;
    float TimeSinceLastSync = 0.0f;

    // ---- Static singleton storage ----
    static TMap<UWorld*, FRGLSceneManager*> Instances;
};

#endif // WITH_RGL
