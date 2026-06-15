// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of FRGLSceneManager: UE5 <-> RGL scene synchronization.

#ifdef WITH_RGL

#include "RGLSceneManager.h"
#include "RGLCoordinateUtils.h"
#include "CarlaRGLModule.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Rendering/PositionVertexBuffer.h"
#include "StaticMeshResources.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "EngineUtils.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/Logging.h>
#include <util/enable-ue4-macros.h>

// RGL_CHECK macro is defined in RGLCoordinateUtils.h

// [DEBUG] Global counters for mesh extraction path statistics (Development only)
#if !UE_BUILD_SHIPPING
int32 GRGLExtractRender = 0, GRGLExtractGPU = 0, GRGLExtractChaos = 0, GRGLExtractFailed = 0;
int64 GRGLTotalVertices = 0, GRGLTotalTriangles = 0;
#endif

// ============================================================================
// Static singleton storage
// ============================================================================

TMap<UWorld*, FRGLSceneManager*> FRGLSceneManager::Instances;

FRGLSceneManager::FRGLSceneManager()
{
    // Use the default scene (nullptr).
    // RGL treats nullptr as the implicit default scene.
    Scene = nullptr;
}

FRGLSceneManager::~FRGLSceneManager()
{
    // Destroy ground plane
    if (GroundEntity)
    {
        rgl_entity_destroy(GroundEntity);
        GroundEntity = nullptr;
    }
    if (GroundMesh)
    {
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
    }

    // Destroy all entities first (they reference meshes)
    for (auto& Pair : EntityMap)
    {
        if (Pair.Value.Entity)
        {
            rgl_entity_destroy(Pair.Value.Entity);
            Pair.Value.Entity = nullptr;
        }
    }
    EntityMap.Empty();

    // Destroy all ISMC entities
    for (auto& Pair : ISMCEntityMap)
    {
        for (rgl_entity_t Entity : Pair.Value.Entities)
        {
            if (Entity)
            {
                rgl_entity_destroy(Entity);
            }
        }
    }
    ISMCEntityMap.Empty();

    // Destroy all cached meshes
    for (auto& Pair : MeshCache)
    {
        if (Pair.Value)
        {
            rgl_mesh_destroy(Pair.Value);
        }
    }
    MeshCache.Empty();
    MeshRefCounts.Empty();
}

FRGLSceneManager& FRGLSceneManager::GetInstance(UWorld* World)
{
    // Clean up stale entries for worlds that are no longer valid
    TArray<UWorld*> StaleWorlds;
    for (auto& Pair : Instances)
    {
        if (!IsValid(Pair.Key))
        {
            StaleWorlds.Add(Pair.Key);
        }
    }
    for (UWorld* Stale : StaleWorlds)
    {
        RGLLog::Info("RGLSceneManager: Cleaning up stale instance for destroyed world");
        delete Instances[Stale];
        Instances.Remove(Stale);
    }

    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        return **Found;
    }

    FRGLSceneManager* NewInstance = new FRGLSceneManager();
    Instances.Add(World, NewInstance);
    return *NewInstance;
}

void FRGLSceneManager::DestroyInstance(UWorld* World)
{
    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        delete *Found;
        Instances.Remove(World);
    }
}

// ============================================================================
// Update: called each frame before raytrace
// ============================================================================

void FRGLSceneManager::Update(UWorld* World, double SimulationTime)
{
    if (!World)
    {
        return;
    }

    // Frame skip: avoid redundant updates if already called this frame
    const uint64 CurrentFrame = GFrameCounter;
    if (bInitialized && CurrentFrame == LastUpdateFrame)
    {
        return;
    }
    LastUpdateFrame = CurrentFrame;

    if (!bInitialized)
    {
        InitializeFromWorld(World);
        bInitialized = true;
    }

    // Periodic full scan for new/removed actors (time-based)
    TimeSinceLastSync += World->GetDeltaSeconds();
    if (TimeSinceLastSync >= SyncIntervalSeconds)
    {
        SyncWorldComponents(World);
        TimeSinceLastSync = 0.0f;
    }

    // Update scene time for velocity computation and ROS2 timestamp synchronization.
    // Use SimulationTime (elapsed since episode start) if provided, otherwise fall back
    // to World->GetTimeSeconds() which includes UE5 engine startup time (~7s offset).
    const double SceneTime = (SimulationTime >= 0.0) ? SimulationTime : World->GetTimeSeconds();
    const uint64 TimeNs = static_cast<uint64>(SceneTime * 1e9);
    RGL_CHECK(rgl_scene_set_time(Scene, TimeNs));

    // Update only dynamic entity transforms (static entities skip after first set)
    UpdateTransforms();
}

// ============================================================================
// InitializeFromWorld: first-time scan
// ============================================================================

void FRGLSceneManager::InitializeFromWorld(UWorld* World)
{
    RGLLog::Info("RGLSceneManager: Initializing from world...");
    RGLLog::Info("RGLSceneManager: Static registration radius=",
                 RegistrationDistanceCm * RGLCoord::UE_TO_RGL, "m",
                 " sensorValid=", bSensorPositionValid ? "true" : "false");

    int32 EntityCount = 0;
    int32 SkippedCount = 0;
    int32 RangeSkippedCount = 0;
    int32 ActorCount = 0;
    int32 CompCount = 0;

    // [DEBUG] Diagnose what types of collision-bearing actors exist in the world
#if !UE_BUILD_SHIPPING
    {
        int32 LandscapeCount = 0;
        int32 StaticMeshActorCount = 0;
        int32 CollisionCompCount = 0;
        int32 NoStaticMeshCompCount = 0;

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* Actor = *It;
            if (!IsValid(Actor)) continue;

            FString ClassName = Actor->GetClass()->GetName();

            if (ClassName.Contains(TEXT("Landscape")))
            {
                ++LandscapeCount;
            }

            TArray<UStaticMeshComponent*> SMComps;
            Actor->GetComponents<UStaticMeshComponent>(SMComps);
            if (SMComps.Num() > 0)
            {
                ++StaticMeshActorCount;
            }
            else
            {
                // Check if this actor has any primitive component with collision
                TArray<UPrimitiveComponent*> PrimComps;
                Actor->GetComponents<UPrimitiveComponent>(PrimComps);
                for (UPrimitiveComponent* Prim : PrimComps)
                {
                    if (Prim && Prim->IsCollisionEnabled())
                    {
                        ++CollisionCompCount;
                        // Log first few non-StaticMesh collision actors
                        static int32 sNonSMLog = 0;
                        if (sNonSMLog < 5)
                        {
                            RGLLog::Info("[DEBUG] Non-SM collision actor:",
                                         TCHAR_TO_UTF8(*Actor->GetName()),
                                         "class:", TCHAR_TO_UTF8(*ClassName),
                                         "comp:", TCHAR_TO_UTF8(*Prim->GetClass()->GetName()));
                            ++sNonSMLog;
                        }
                        break;
                    }
                }
                ++NoStaticMeshCompCount;
            }
        }

        RGLLog::Info("[DEBUG] World actor types: Landscape=", LandscapeCount,
                     "WithStaticMesh=", StaticMeshActorCount,
                     "WithoutStaticMesh=", NoStaticMeshCompCount,
                     "NonSM-Collision=", CollisionCompCount);
    }
#endif

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }
        ++ActorCount;

        TArray<UStaticMeshComponent*> Components;
        Actor->GetComponents<UStaticMeshComponent>(Components);

        for (UStaticMeshComponent* Comp : Components)
        {
            if (IsValid(Comp) && Comp->GetStaticMesh())
            {
                ++CompCount;
            }

            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++RangeSkippedCount;
                }
                continue;
            }

            if (RegisterComponent(Comp))
            {
                ++EntityCount;
            }
            else
            {
                ++SkippedCount;
            }
        }
    }

    const int32 MeshCount = MeshCache.Num();
    RGLLog::Info("RGLSceneManager: Scanned", ActorCount, "actors,", CompCount,
                 "mesh components. Uploaded", MeshCount, "unique meshes, created",
                 EntityCount, "entities (skipped", SkippedCount,
                 ", range-skipped", RangeSkippedCount, ")");

    const int32 ISMCCompCount = ISMCEntityMap.Num();
    int32 ISMCEntityTotal = 0;
    for (const auto& Pair : ISMCEntityMap)
    {
        ISMCEntityTotal += Pair.Value.Entities.Num();
    }
    RGLLog::Info("RGLSceneManager: ISMC components=", ISMCCompCount,
                 " total ISMC instances=", ISMCEntityTotal);

#if !UE_BUILD_SHIPPING
    // Print extraction path statistics
    {
        extern int32 GRGLExtractRender, GRGLExtractGPU, GRGLExtractChaos, GRGLExtractFailed;
        extern int64 GRGLTotalVertices, GRGLTotalTriangles;
        RGLLog::Info("[DEBUG] Mesh extraction paths: RenderData=", GRGLExtractRender,
                     "GPUBuffer=", GRGLExtractGPU,
                     "ChaosCollision=", GRGLExtractChaos, "Failed=", GRGLExtractFailed);
        RGLLog::Info("[DEBUG] Total scene geometry: vertices=", GRGLTotalVertices,
                     "triangles=", GRGLTotalTriangles);

        int32 StaticCount = 0, DynamicCount = 0;
        for (const auto& Pair : EntityMap)
        {
            if (Pair.Value.bIsStatic) ++StaticCount; else ++DynamicCount;
        }
        RGLLog::Info("[DEBUG] Entity mobility: Static=", StaticCount, "Dynamic=", DynamicCount);
        RGLLog::Info("[DEBUG] ISMC entity map: components=", ISMCEntityMap.Num());
    }
#endif
}

// ============================================================================
// Mesh upload: extract vertex/index data from UStaticMesh
// ============================================================================

/// Try to extract geometry from Chaos TriMeshes (complex collision, full triangle detail).
/// This is the primary path — preserves concave geometry unlike ConvexElems.
static bool ExtractFromChaosTriMeshes(UStaticMesh* StaticMesh,
                                      TArray<rgl_vec3f>& OutVertices,
                                      TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    if (BodySetup->ChaosTriMeshes.Num() > 0)
    {
        // Merge all tri-meshes into one vertex/index set
        int32 VertexOffset = 0;

        for (const auto& TriMeshPtr : BodySetup->ChaosTriMeshes)
        {
            if (!TriMeshPtr.IsValid())
            {
                continue;
            }

            const Chaos::FTriangleMeshImplicitObject& TriMesh = *TriMeshPtr;
            const auto& Particles = TriMesh.Particles();
            const auto& Elements = TriMesh.Elements();
            const int32 NumVerts = static_cast<int32>(Particles.Size());
            const int32 NumTris = Elements.GetNumTriangles();

            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            // Append vertices (in local space, cm -> m)
            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const Chaos::FVec3 P = Particles.GetX(i);
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(P[0]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[1]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[2]) * RGLCoord::UE_TO_RGL
                } };
            }

            // Append indices (offset by accumulated vertex count)
            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);

            if (Elements.RequiresLargeIndices())
            {
                const auto& IdxBuf = Elements.GetLargeIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }
            else
            {
                const auto& IdxBuf = Elements.GetSmallIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Fallback: extract from ConvexElems or BoxElems (simplified convex geometry).
/// Only used as last resort — convex hulls lose concave features.
static bool ExtractFromChaosConvexOrBox(UStaticMesh* StaticMesh,
                                        TArray<rgl_vec3f>& OutVertices,
                                        TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    const FKAggregateGeom& AggGeom = BodySetup->AggGeom;
    if (AggGeom.ConvexElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKConvexElem& Convex : AggGeom.ConvexElems)
        {
            const int32 NumVerts = Convex.VertexData.Num();
            const int32 NumIdx = Convex.IndexData.Num();
            const int32 NumTris = NumIdx / 3;
            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const FVector& V = Convex.VertexData[i];
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(V.X) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Y) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Z) * RGLCoord::UE_TO_RGL
                } };
            }

            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);
            for (int32 i = 0; i < NumTris; ++i)
            {
                OutIndices[PrevTriCount + i] = { {
                    static_cast<int32>(Convex.IndexData[i * 3 + 0]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 1]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 2]) + VertexOffset
                } };
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    // Fallback: try box collision elements (common for walls, floors, building shells)
    if (AggGeom.BoxElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKBoxElem& Box : AggGeom.BoxElems)
        {
            // Generate 8 vertices and 12 triangles for each box
            const float HX = static_cast<float>(Box.X) * 0.5f * RGLCoord::UE_TO_RGL; // half-extent in meters
            const float HY = static_cast<float>(Box.Y) * 0.5f * RGLCoord::UE_TO_RGL;
            const float HZ = static_cast<float>(Box.Z) * 0.5f * RGLCoord::UE_TO_RGL;

            // Box center offset (in meters)
            const FVector C = Box.Center;
            const float CX = static_cast<float>(C.X) * RGLCoord::UE_TO_RGL;
            const float CY = static_cast<float>(C.Y) * RGLCoord::UE_TO_RGL;
            const float CZ = static_cast<float>(C.Z) * RGLCoord::UE_TO_RGL;

            const int32 PrevVert = OutVertices.Num();
            OutVertices.AddUninitialized(8);
            OutVertices[PrevVert + 0] = {{ CX - HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 1] = {{ CX + HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 2] = {{ CX + HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 3] = {{ CX - HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 4] = {{ CX - HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 5] = {{ CX + HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 6] = {{ CX + HX, CY + HY, CZ + HZ }};
            OutVertices[PrevVert + 7] = {{ CX - HX, CY + HY, CZ + HZ }};

            const int32 V = VertexOffset;
            const int32 PrevTri = OutIndices.Num();
            OutIndices.AddUninitialized(12);
            // Bottom face
            OutIndices[PrevTri + 0]  = {{ V+0, V+2, V+1 }};
            OutIndices[PrevTri + 1]  = {{ V+0, V+3, V+2 }};
            // Top face
            OutIndices[PrevTri + 2]  = {{ V+4, V+5, V+6 }};
            OutIndices[PrevTri + 3]  = {{ V+4, V+6, V+7 }};
            // Front face
            OutIndices[PrevTri + 4]  = {{ V+0, V+1, V+5 }};
            OutIndices[PrevTri + 5]  = {{ V+0, V+5, V+4 }};
            // Back face
            OutIndices[PrevTri + 6]  = {{ V+2, V+3, V+7 }};
            OutIndices[PrevTri + 7]  = {{ V+2, V+7, V+6 }};
            // Left face
            OutIndices[PrevTri + 8]  = {{ V+0, V+4, V+7 }};
            OutIndices[PrevTri + 9]  = {{ V+0, V+7, V+3 }};
            // Right face
            OutIndices[PrevTri + 10] = {{ V+1, V+2, V+6 }};
            OutIndices[PrevTri + 11] = {{ V+1, V+6, V+5 }};

            VertexOffset += 8;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Try to extract geometry from render vertex/index buffers (requires CPU access).
static bool ExtractFromRenderData(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();

    if (NumVertices == 0 || VB.GetStride() == 0)
    {
        return false;
    }

    // CPU data must be accessible (otherwise VertexPosition() dereferences null -> SIGSEGV).
    // This check is required in ALL build configurations — even Development packaged builds
    // release CPU-side vertex data after GPU upload.
    if (!StaticMesh->bAllowCPUAccess && !VB.GetAllowCPUAccess())
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f& Pos = VB.VertexPosition(i);
        OutVertices[i] = { {
            Pos.X * RGLCoord::UE_TO_RGL,
            Pos.Y * RGLCoord::UE_TO_RGL,
            Pos.Z * RGLCoord::UE_TO_RGL
        } };
    }

    FIndexArrayView IndexView = LOD.IndexBuffer.GetArrayView();
    const int32 NumTriangles = IndexView.Num() / 3;
    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (int32 i = 0; i < NumTriangles; ++i)
    {
        const int32 Idx0 = static_cast<int32>(IndexView[i * 3 + 0]);
        const int32 Idx1 = static_cast<int32>(IndexView[i * 3 + 1]);
        const int32 Idx2 = static_cast<int32>(IndexView[i * 3 + 2]);

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    return true;
}

/// Extract geometry by reading GPU vertex/index buffers via RHI readback.
/// Works in ALL build configurations (Shipping included) since render data is always on GPU.
/// Must be called from game thread (uses FlushRenderingCommands).
/// Uses the coarsest available LOD to minimize polygon count for raytrace performance.
static bool ExtractFromGPUBuffer(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];

    // --- Vertex buffer readback ---
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();
    if (NumVertices == 0)
    {
        return false;
    }

    const FBufferRHIRef& VBRef = VB.VertexBufferRHI;
    if (!VBRef.IsValid())
    {
        return false;
    }

    // Flush rendering to ensure the buffer is up to date
    FlushRenderingCommands();

    const uint32 VBSize = VBRef->GetSize();
    const uint32 Stride = VBSize / NumVertices;
    if (Stride < sizeof(FVector3f))
    {
        return false;
    }

    // Lock the GPU buffer for reading
    void* LockedVB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedVB = RHICmdList.LockBuffer(VBRef, 0, VBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedVB)
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f* Pos = reinterpret_cast<const FVector3f*>(
            static_cast<const uint8*>(LockedVB) + i * Stride);
        OutVertices[i] = { {
            Pos->X * RGLCoord::UE_TO_RGL,
            Pos->Y * RGLCoord::UE_TO_RGL,
            Pos->Z * RGLCoord::UE_TO_RGL
        } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(VBRef);
    });
    FlushRenderingCommands();

    // --- Index buffer readback ---
    const FRawStaticIndexBuffer& IB = LOD.IndexBuffer;
    const FBufferRHIRef& IBRef = IB.IndexBufferRHI;
    if (!IBRef.IsValid())
    {
        OutVertices.Empty();
        return false;
    }

    const uint32 IBSize = IBRef->GetSize();
    const bool bIs32Bit = (IB.Is32Bit());
    const uint32 IndexStride = bIs32Bit ? 4 : 2;
    const uint32 NumIndices = IBSize / IndexStride;
    const uint32 NumTriangles = NumIndices / 3;

    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    void* LockedIB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedIB = RHICmdList.LockBuffer(IBRef, 0, IBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedIB)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (uint32 i = 0; i < NumTriangles; ++i)
    {
        int32 Idx0, Idx1, Idx2;
        if (bIs32Bit)
        {
            const uint32* Indices32 = static_cast<const uint32*>(LockedIB);
            Idx0 = static_cast<int32>(Indices32[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices32[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices32[i * 3 + 2]);
        }
        else
        {
            const uint16* Indices16 = static_cast<const uint16*>(LockedIB);
            Idx0 = static_cast<int32>(Indices16[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices16[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices16[i * 3 + 2]);
        }

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.UnlockBuffer(IBRef);
            });
            FlushRenderingCommands();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(IBRef);
    });
    FlushRenderingCommands();

    return true;
}

rgl_mesh_t FRGLSceneManager::UploadMesh(UStaticMesh* StaticMesh)
{
    if (!StaticMesh)
    {
        return nullptr;
    }

    // Check cache first
    rgl_mesh_t* CachedMesh = MeshCache.Find(StaticMesh);
    if (CachedMesh)
    {
        return *CachedMesh;
    }

    TArray<rgl_vec3f> Vertices;
    TArray<rgl_vec3i> Indices;

    // Extraction priority:
    // 1. Chaos TriMeshes — complex collision with full triangle detail (fast raytrace)
    // 2. Render data (CPU access) — if bAllowCPUAccess is set
    // 3. GPU buffer readback — render mesh from GPU (full fidelity, heavy)
    // 4. Chaos ConvexElems/BoxElems — simplified convex hulls (last resort)
    //
    // Note: AWSIM uses render meshes or direct collision trimeshes, never convex hulls.
    // ConvexElems lose concave features (e.g., spiral towers become simple polyhedra).
    // GPU readback preserves full visual fidelity, so it is preferred over ConvexElems.
    if (ExtractFromChaosTriMeshes(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;
#endif
    }
    else if (ExtractFromRenderData(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractRender;
#endif
    }
    else if (ExtractFromGPUBuffer(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractGPU;
#endif
    }
    else if (ExtractFromChaosConvexOrBox(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;  // Still count as Chaos path
#endif
    }
    else
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractFailed;
#endif
        return nullptr;
    }

    if (Vertices.Num() == 0 || Indices.Num() == 0)
    {
        return nullptr;
    }

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first mesh's vertex bounds
    {
        static int32 sMeshLogCount = 0;
        if (sMeshLogCount < 2)
        {
            float minX=1e9, minY=1e9, minZ=1e9, maxX=-1e9, maxY=-1e9, maxZ=-1e9;
            for (int32 i = 0; i < Vertices.Num(); ++i)
            {
                minX = FMath::Min(minX, Vertices[i].value[0]);
                minY = FMath::Min(minY, Vertices[i].value[1]);
                minZ = FMath::Min(minZ, Vertices[i].value[2]);
                maxX = FMath::Max(maxX, Vertices[i].value[0]);
                maxY = FMath::Max(maxY, Vertices[i].value[1]);
                maxZ = FMath::Max(maxZ, Vertices[i].value[2]);
            }
            RGLLog::Info("[DEBUG] Mesh", sMeshLogCount,
                         "name:", TCHAR_TO_UTF8(*StaticMesh->GetName()),
                         "verts:", Vertices.Num(), "tris:", Indices.Num(),
                         "bounds(m): min(", minX, minY, minZ, ") max(", maxX, maxY, maxZ, ")");
            ++sMeshLogCount;
        }
    }
#endif

#if !UE_BUILD_SHIPPING
    // Track total scene geometry
    extern int64 GRGLTotalVertices, GRGLTotalTriangles;
    GRGLTotalVertices += Vertices.Num();
    GRGLTotalTriangles += Indices.Num();
#endif

    // Upload to RGL
    rgl_mesh_t RGLMesh = nullptr;
    rgl_status_t Status = rgl_mesh_create(
        &RGLMesh,
        Vertices.GetData(),
        static_cast<int32_t>(Vertices.Num()),
        Indices.GetData(),
        static_cast<int32_t>(Indices.Num()));

    if (Status != RGL_SUCCESS || !RGLMesh)
    {
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to upload mesh '%s' (%d verts, %d tris): %s"),
               *StaticMesh->GetName(),
               Vertices.Num(), Indices.Num(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        return nullptr;
    }

    // Cache the mesh
    MeshCache.Add(StaticMesh, RGLMesh);

    return RGLMesh;
}

void FRGLSceneManager::AddMeshReference(UStaticMesh* StaticMesh, int32 Count)
{
    if (!StaticMesh || Count <= 0)
    {
        return;
    }

    int32& RefCount = MeshRefCounts.FindOrAdd(StaticMesh);
    RefCount += Count;
}

void FRGLSceneManager::ReleaseMeshReference(UStaticMesh* StaticMesh, int32 Count)
{
    if (!StaticMesh || Count <= 0)
    {
        return;
    }

    int32* RefCount = MeshRefCounts.Find(StaticMesh);
    if (!RefCount)
    {
        return;
    }

    *RefCount -= Count;
    if (*RefCount > 0)
    {
        return;
    }

    MeshRefCounts.Remove(StaticMesh);

    rgl_mesh_t* RGLMesh = MeshCache.Find(StaticMesh);
    if (RGLMesh && *RGLMesh)
    {
        rgl_mesh_destroy(*RGLMesh);
    }
    MeshCache.Remove(StaticMesh);
}

// ============================================================================
// Entity management
// ============================================================================

bool FRGLSceneManager::IsComponentWithinDistance(UStaticMeshComponent* Component, float DistanceCm) const
{
    if (!bSensorPositionValid || !IsValid(Component))
    {
        return false;
    }

    const float BoundsRadiusCm = Component->Bounds.SphereRadius;
    const FVector BoundsCenter = Component->Bounds.Origin;
    const float EffectiveDistanceCm = DistanceCm + BoundsRadiusCm;
    return FVector::DistSquared(BoundsCenter, SensorPosition) <= FMath::Square(EffectiveDistanceCm);
}

bool FRGLSceneManager::IsInstanceWithinDistance(
    UInstancedStaticMeshComponent* Component,
    UStaticMesh* StaticMesh,
    const FTransform& InstanceTransform,
    float DistanceCm) const
{
    if (!bSensorPositionValid || !IsValid(Component) || !StaticMesh)
    {
        return false;
    }

    const FVector Scale = InstanceTransform.GetScale3D();
    const float MaxScale = FMath::Max(FMath::Max(FMath::Abs(Scale.X), FMath::Abs(Scale.Y)), FMath::Abs(Scale.Z));
    const float BoundsRadiusCm = StaticMesh->GetBounds().SphereRadius * MaxScale;
    const float EffectiveDistanceCm = DistanceCm + BoundsRadiusCm;
    return FVector::DistSquared(InstanceTransform.GetLocation(), SensorPosition) <=
           FMath::Square(EffectiveDistanceCm);
}

bool FRGLSceneManager::ShouldRegisterComponent(UStaticMeshComponent* Component, bool& bOutOfRange) const
{
    bOutOfRange = false;

    if (!IsValid(Component) || !Component->GetStaticMesh() || !Component->IsVisible())
    {
        return false;
    }

    const FTransform WorldTransform = Component->GetComponentTransform();
    const FVector Scale = WorldTransform.GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
    {
        return false;
    }

    AActor* Owner = Component->GetOwner();
    const FString OwnerName = Owner ? Owner->GetName() : FString();
    const FString ComponentName = Component->GetName();
    const FString MeshName = Component->GetStaticMesh()->GetName();

    // Sky domes are valid static meshes but should never participate in LiDAR ray tracing.
    if (OwnerName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase) ||
        ComponentName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase) ||
        MeshName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase))
    {
        return false;
    }

    // Movable actors are kept even if the sensor position has not been set yet.
    if (Component->Mobility != EComponentMobility::Static)
    {
        return true;
    }

    // Static geometry on large maps is large enough to exhaust OptiX if uploaded wholesale.
    // Register only meshes whose bounds intersect the sensor-centered radius.
    if (!bSensorPositionValid)
    {
        bOutOfRange = true;
        return false;
    }

    if (!IsComponentWithinDistance(Component, RegistrationDistanceCm))
    {
        bOutOfRange = true;
        return false;
    }

    return true;
}

bool FRGLSceneManager::ShouldKeepRegisteredComponent(UStaticMeshComponent* Component) const
{
    bool bOutOfRange = false;
    if (ShouldRegisterComponent(Component, bOutOfRange))
    {
        return true;
    }

    if (!bOutOfRange || !IsValid(Component))
    {
        return false;
    }

    // Keep a wider unload radius than the load radius to prevent repeated
    // destroy/recreate near the boundary while the ego vehicle moves.
    return IsComponentWithinDistance(Component, RegistrationDistanceCm + UnregistrationHysteresisCm);
}

bool FRGLSceneManager::RegisterComponent(UStaticMeshComponent* Component)
{
    if (!IsValid(Component) || EntityMap.Contains(Component))
    {
        return false;
    }

    // ISMC branch: expand all instances into separate RGL entities
    if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Component))
    {
        if (ISMCEntityMap.Contains(ISMComp))
        {
            return false;
        }
        return RegisterISMComponent(ISMComp);
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    AddMeshReference(StaticMesh);

    rgl_entity_t Entity = nullptr;
    rgl_status_t Status = rgl_entity_create(&Entity, Scene, RGLMesh);
    if (Status != RGL_SUCCESS || !Entity)
    {
        ReleaseMeshReference(StaticMesh);
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to create entity for '%s': %s"),
               *Component->GetName(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        return false;
    }

    // Set initial transform
    const FTransform WorldTransform = Component->GetComponentTransform();
    rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
    RGL_CHECK(rgl_entity_set_transform(Entity, &RGLTransform));

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first 3 entities' positions
    {
        static int32 sEntityLogCount = 0;
        if (sEntityLogCount < 3)
        {
            const FVector Pos = WorldTransform.GetLocation();
            RGLLog::Info("[DEBUG] Entity", sEntityLogCount,
                         "name:", TCHAR_TO_UTF8(*Component->GetOwner()->GetName()),
                         "UE5pos(cm):", Pos.X, Pos.Y, Pos.Z,
                         "RGLpos(m):", RGLTransform.value[0][3], RGLTransform.value[1][3], RGLTransform.value[2][3]);
            ++sEntityLogCount;
        }
    }
#endif

    // Store mapping with static/dynamic classification
    FEntityInfo Info;
    Info.Entity = Entity;
    Info.Component = Component;
    Info.StaticMesh = StaticMesh;
    Info.bIsStatic = (Component->Mobility == EComponentMobility::Static);
    Info.bTransformInitialized = true;
    Info.LastTransform = WorldTransform;
    EntityMap.Add(Component, Info);

    return true;
}

void FRGLSceneManager::UnregisterComponent(UStaticMeshComponent* Component)
{
    FEntityInfo* Info = EntityMap.Find(Component);
    if (!Info)
    {
        return;
    }

    if (Info->Entity)
    {
        rgl_entity_destroy(Info->Entity);
        Info->Entity = nullptr;
    }
    ReleaseMeshReference(Info->StaticMesh);
    EntityMap.Remove(Component);
}

bool FRGLSceneManager::RegisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    if (!IsValid(Component))
    {
        return false;
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    AddMeshReference(StaticMesh); // Temporary reference until at least one entity owns the mesh.

    const int32 NumInstances = Component->GetInstanceCount();
    if (NumInstances <= 0)
    {
        ReleaseMeshReference(StaticMesh);
        return false;
    }

    FISMCEntityGroup Group;
    Group.Component = Component;
    Group.StaticMesh = StaticMesh;
    Group.OriginalInstanceCount = NumInstances;
    Group.Entities.Reserve(NumInstances);

    int32 SuccessCount = 0;
    int32 RangeSkippedCount = 0;
    for (int32 i = 0; i < NumInstances; ++i)
    {
        FTransform InstanceTransform;
        if (!Component->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/true))
        {
            continue;
        }

        // Skip instances with zero/near-zero scale
        const FVector Scale = InstanceTransform.GetScale3D();
        if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
        {
            continue;
        }

        if (Component->Mobility == EComponentMobility::Static &&
            !IsInstanceWithinDistance(Component, StaticMesh, InstanceTransform, RegistrationDistanceCm))
        {
            ++RangeSkippedCount;
            continue;
        }

        rgl_entity_t Entity = nullptr;
        rgl_status_t Status = rgl_entity_create(&Entity, Scene, RGLMesh);
        if (Status != RGL_SUCCESS || !Entity)
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(InstanceTransform);
        RGL_CHECK(rgl_entity_set_transform(Entity, &RGLTransform));

        Group.Entities.Add(Entity);
        ++SuccessCount;
    }

    if (SuccessCount == 0)
    {
        ReleaseMeshReference(StaticMesh);
        return false;
    }

    Group.MeshRefCount = SuccessCount;
    ISMCEntityMap.Add(Component, MoveTemp(Group));
    AddMeshReference(StaticMesh, SuccessCount);
    ReleaseMeshReference(StaticMesh); // Drop the temporary upload reference.

    RGLLog::Info("RGLSceneManager: Registered ISMC '",
                 TCHAR_TO_UTF8(*Component->GetName()),
                 "' instances=", NumInstances,
                 " entities=", SuccessCount,
                 " range-skipped=", RangeSkippedCount,
                 " mesh='", TCHAR_TO_UTF8(*StaticMesh->GetName()), "'");

    return true;
}

void FRGLSceneManager::UnregisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Component);
    if (!Group)
    {
        return;
    }

    for (rgl_entity_t Entity : Group->Entities)
    {
        if (Entity)
        {
            rgl_entity_destroy(Entity);
        }
    }
    ReleaseMeshReference(Group->StaticMesh, Group->MeshRefCount);
    ISMCEntityMap.Remove(Component);
}

// ============================================================================
// Transform update
// ============================================================================

void FRGLSceneManager::UpdateTransforms()
{
    TArray<UStaticMeshComponent*> ToRemove;

    for (auto& Pair : EntityMap)
    {
        FEntityInfo& Info = Pair.Value;

        // Use weak pointer to safely detect destroyed components
        if (!Info.Component.IsValid())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        UStaticMeshComponent* Comp = Info.Component.Get();
        if (!IsValid(Comp) || !Comp->IsRegistered())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        // Optimization: skip static entities after initial transform is set
        if (Info.bIsStatic && Info.bTransformInitialized)
        {
            continue;
        }

        const FTransform WorldTransform = Comp->GetComponentTransform();

        // Skip entities with zero/near-zero scale to avoid NaN in transforms
        const FVector Scale = WorldTransform.GetScale3D();
        if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
        {
            continue;
        }

        // Optimization: skip dynamic entities whose transform hasn't changed
        if (Info.bTransformInitialized && WorldTransform.Equals(Info.LastTransform, 0.01f))
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
        RGL_CHECK(rgl_entity_set_transform(Info.Entity, &RGLTransform));

        Info.LastTransform = WorldTransform;
        Info.bTransformInitialized = true;
    }

    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
    }
}

// ============================================================================
// Periodic world sync: detect new/removed components
// ============================================================================

void FRGLSceneManager::SyncWorldComponents(UWorld* World)
{
    TSet<UStaticMeshComponent*> CurrentComponents;
    TSet<UInstancedStaticMeshComponent*> CurrentISMComponents;
    int32 AddedRegular = 0;
    int32 AddedISM = 0;
    int32 RemovedRegular = 0;
    int32 RemovedISM = 0;
    int32 RangeSkipped = 0;

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }

        TArray<UStaticMeshComponent*> Components;
        Actor->GetComponents<UStaticMeshComponent>(Components);

        for (UStaticMeshComponent* Comp : Components)
        {
            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++RangeSkipped;
                }
                continue;
            }

            // Classify: ISMC or regular
            if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Comp))
            {
                CurrentISMComponents.Add(ISMComp);
                if (!ISMCEntityMap.Contains(ISMComp))
                {
                    if (RegisterISMComponent(ISMComp))
                    {
                        ++AddedISM;
                    }
                }
            }
            else
            {
                CurrentComponents.Add(Comp);
                if (!EntityMap.Contains(Comp))
                {
                    if (RegisterComponent(Comp))
                    {
                        ++AddedRegular;
                    }
                }
            }
        }
    }

    // Remove regular entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    TArray<UStaticMeshComponent*> ToRemove;
    for (auto& Pair : EntityMap)
    {
        if (!CurrentComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ToRemove.Add(Pair.Key);
        }
    }
    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
        ++RemovedRegular;
    }

    // Remove ISMC entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    TArray<UInstancedStaticMeshComponent*> ISMCToRemove;
    for (auto& Pair : ISMCEntityMap)
    {
        if (!CurrentISMComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ISMCToRemove.Add(Pair.Key);
        }
    }
    for (UInstancedStaticMeshComponent* Comp : ISMCToRemove)
    {
        UnregisterISMComponent(Comp);
        ++RemovedISM;
    }

    if (AddedRegular || AddedISM || RemovedRegular || RemovedISM)
    {
        RGLLog::Info("RGLSceneManager: Dynamic sync active regular=", EntityMap.Num(),
                     " ISMC=", ISMCEntityMap.Num(),
                     " cachedMeshes=", MeshCache.Num(),
                     " added=", AddedRegular, "+", AddedISM,
                     " removed=", RemovedRegular, "+", RemovedISM,
                     " range-skipped=", RangeSkipped,
                     " radius(m)=", RegistrationDistanceCm * RGLCoord::UE_TO_RGL);
    }
}

// ============================================================================
// Ground plane: a large flat mesh at Z=0 to represent road surface
// ============================================================================

void FRGLSceneManager::CreateGroundPlane()
{
    // Create a 2km × 2km quad centered at the origin, at Z=0.
    // This covers the typical LiDAR range (100m) around any sensor position.
    // The actual ground height varies, but Z=0 is a reasonable approximation
    // for flat urban roads in CARLA.
    static constexpr float HALF_SIZE = 1000.0f; // 1000m = 1km half-extent

    rgl_vec3f Vertices[4] = {
        {{ -HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE,  HALF_SIZE, 0.0f }},
        {{ -HALF_SIZE,  HALF_SIZE, 0.0f }}
    };

    // Two triangles forming the quad (both face upward)
    rgl_vec3i Indices[2] = {
        {{ 0, 1, 2 }},
        {{ 0, 2, 3 }}
    };

    rgl_status_t Status = rgl_mesh_create(&GroundMesh, Vertices, 4, Indices, 2);
    if (Status != RGL_SUCCESS || !GroundMesh)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane mesh");
        return;
    }

    Status = rgl_entity_create(&GroundEntity, Scene, GroundMesh);
    if (Status != RGL_SUCCESS || !GroundEntity)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane entity");
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
        return;
    }

    // Place at origin with identity rotation (already in meters, Z=0).
    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));

    RGLLog::Info("RGLSceneManager: Ground plane created (2km x 2km)");
}

void FRGLSceneManager::UpdateGroundPlane(UWorld* World, const FTransform& SensorTransform)
{
    if (!GroundEntity || !World)
    {
        return;
    }

    // Trace a ray straight down from the sensor to find the actual ground level.
    const FVector SensorPos = SensorTransform.GetLocation();
    const FVector TraceStart = SensorPos;
    const FVector TraceEnd = SensorPos - FVector(0, 0, 10000.0f); // 100m downward

    FHitResult HitResult;
    FCollisionQueryParams TraceParams;
    TraceParams.bTraceComplex = false;

    float GroundZ;
    if (World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd,
                                          ECC_WorldStatic, TraceParams))
    {
        GroundZ = static_cast<float>(HitResult.ImpactPoint.Z) * RGLCoord::UE_TO_RGL;
    }
    else
    {
        // Fallback: assume ground is 2m below sensor
        GroundZ = static_cast<float>(SensorPos.Z - 200.0f) * RGLCoord::UE_TO_RGL;
    }

    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    GroundTf.value[0][3] = static_cast<float>(SensorPos.X) * RGLCoord::UE_TO_RGL;
    GroundTf.value[1][3] = static_cast<float>(SensorPos.Y) * RGLCoord::UE_TO_RGL;
    GroundTf.value[2][3] = GroundZ;

    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));
}

#endif // WITH_RGL
