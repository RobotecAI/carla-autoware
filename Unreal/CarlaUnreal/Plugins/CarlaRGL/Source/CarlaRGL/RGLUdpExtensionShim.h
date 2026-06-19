// SPDX-License-Identifier: MIT
//
// Forward declarations for the optional RGL UDP extension.
//
// These types and the rgl_node_points_udp_publish function are defined in the
// private RobotecAI/RGL-extension-udp repository, which is *not* vendored in
// the CARLA RGL fork. The values mirror AWSIM's RGLNativeTypes.cs so that the
// CarlaRGL plugin can build (and link via dlsym graceful fallback) regardless
// of whether the runtime libRobotecGPULidar.so contains the extension.
//
// At runtime, callers must gate use of rgl_node_points_udp_publish on
// rgl_get_extension_info(RGL_EXTENSION_UDP, ...) returning available=1.

#pragma once

#include <cstdint>

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <util/enable-ue4-macros.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RGL_VELODYNE_VLP16      = 1,
    RGL_VELODYNE_VLP32C     = 2,
    RGL_VELODYNE_VLS128     = 3,
    RGL_HESAI_PANDAR_40P    = 4,
    RGL_HESAI_PANDAR_QT64   = 5,
    RGL_HESAI_QT128C2X      = 6,
    RGL_HESAI_PANDAR_128E4X = 7,
    RGL_HESAI_PANDAR_XT32   = 8,
} rgl_lidar_model_t;

typedef enum
{
    RGL_UDP_NO_ADDITIONAL_OPTIONS           = 0,
    RGL_UDP_ENABLE_HESAI_UDP_SEQUENCE       = 1 << 0,
    RGL_UDP_HIGH_RESOLUTION_MODE            = 1 << 1,
    RGL_UDP_UP_CLOSE_BLOCKAGE_DETECTION     = 1 << 2,
    RGL_UDP_FIT_QT64_TO_HESAI_PANDAR_DRIVER = 1 << 3,
} rgl_udp_options_t;

// Forward declaration: actual symbol lives in libRobotecGPULidar.so when the
// UDP extension was compiled in (RGL_BUILD_UDP_EXTENSION=ON). When absent,
// the dlsym lookup in RGLDynLoader.cpp returns null and the wrapper returns
// RGL_INVALID_STATE without ever invoking this function.
rgl_status_t rgl_node_points_udp_publish(
    rgl_node_t* node,
    rgl_lidar_model_t lidar_model,
    rgl_udp_options_t udp_options,
    const char* device_ip,
    const char* dest_ip,
    int32_t dest_port);

#ifdef __cplusplus
}
#endif
