// Dynamic loader for libRobotecGPULidar.so
// Replaces link-time dependency with dlopen/dlsym at runtime.
// When the library is unavailable, wrapper functions return RGL_INVALID_STATE
// (or do nothing for void functions), allowing graceful degradation.

#include "RGLDynLoader.h"

#include <dlfcn.h>
#include <cstdio>

// Include RGL headers for type definitions (rgl_status_t, rgl_node_t, rgl_mesh_t, etc.)
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>

// ---------------------------------------------------------------------------
// Library handle
// ---------------------------------------------------------------------------
static void* Handle = nullptr;

// ---------------------------------------------------------------------------
// Function pointer types and static variables
// ---------------------------------------------------------------------------

// 1. rgl_get_last_error_string
typedef void (*fn_rgl_get_last_error_string_t)(const char**);
static fn_rgl_get_last_error_string_t fn_rgl_get_last_error_string = nullptr;

// 2. rgl_mesh_create
typedef rgl_status_t (*fn_rgl_mesh_create_t)(rgl_mesh_t*, const rgl_vec3f*, int32_t, const rgl_vec3i*, int32_t);
static fn_rgl_mesh_create_t fn_rgl_mesh_create = nullptr;

// 3. rgl_mesh_destroy
typedef rgl_status_t (*fn_rgl_mesh_destroy_t)(rgl_mesh_t);
static fn_rgl_mesh_destroy_t fn_rgl_mesh_destroy = nullptr;

// 4. rgl_entity_create
typedef rgl_status_t (*fn_rgl_entity_create_t)(rgl_entity_t*, rgl_scene_t, rgl_mesh_t);
static fn_rgl_entity_create_t fn_rgl_entity_create = nullptr;

// 5. rgl_entity_destroy
typedef rgl_status_t (*fn_rgl_entity_destroy_t)(rgl_entity_t);
static fn_rgl_entity_destroy_t fn_rgl_entity_destroy = nullptr;

// 6. rgl_entity_set_transform
typedef rgl_status_t (*fn_rgl_entity_set_transform_t)(rgl_entity_t, const rgl_mat3x4f*);
static fn_rgl_entity_set_transform_t fn_rgl_entity_set_transform = nullptr;

// 7. rgl_scene_set_time
typedef rgl_status_t (*fn_rgl_scene_set_time_t)(rgl_scene_t, uint64_t);
static fn_rgl_scene_set_time_t fn_rgl_scene_set_time = nullptr;

// 8. rgl_node_rays_from_mat3x4f
typedef rgl_status_t (*fn_rgl_node_rays_from_mat3x4f_t)(rgl_node_t*, const rgl_mat3x4f*, int32_t);
static fn_rgl_node_rays_from_mat3x4f_t fn_rgl_node_rays_from_mat3x4f = nullptr;

// 9. rgl_node_rays_set_ring_ids
typedef rgl_status_t (*fn_rgl_node_rays_set_ring_ids_t)(rgl_node_t*, const int32_t*, int32_t);
static fn_rgl_node_rays_set_ring_ids_t fn_rgl_node_rays_set_ring_ids = nullptr;

// 10. rgl_node_rays_set_range
typedef rgl_status_t (*fn_rgl_node_rays_set_range_t)(rgl_node_t*, const rgl_vec2f*, int32_t);
static fn_rgl_node_rays_set_range_t fn_rgl_node_rays_set_range = nullptr;

// 11. rgl_node_rays_transform
typedef rgl_status_t (*fn_rgl_node_rays_transform_t)(rgl_node_t*, const rgl_mat3x4f*);
static fn_rgl_node_rays_transform_t fn_rgl_node_rays_transform = nullptr;

// 12. rgl_node_raytrace
typedef rgl_status_t (*fn_rgl_node_raytrace_t)(rgl_node_t*, rgl_scene_t);
static fn_rgl_node_raytrace_t fn_rgl_node_raytrace = nullptr;

// 13. rgl_node_raytrace_configure_non_hits
typedef rgl_status_t (*fn_rgl_node_raytrace_configure_non_hits_t)(rgl_node_t, float, float);
static fn_rgl_node_raytrace_configure_non_hits_t fn_rgl_node_raytrace_configure_non_hits = nullptr;

// 13b. rgl_node_raytrace_configure_mask
typedef rgl_status_t (*fn_rgl_node_raytrace_configure_mask_t)(rgl_node_t, const int8_t*, int32_t);
static fn_rgl_node_raytrace_configure_mask_t fn_rgl_node_raytrace_configure_mask = nullptr;

// 14. rgl_node_points_compact_by_field
typedef rgl_status_t (*fn_rgl_node_points_compact_by_field_t)(rgl_node_t*, rgl_field_t);
static fn_rgl_node_points_compact_by_field_t fn_rgl_node_points_compact_by_field = nullptr;

// 15. rgl_node_points_transform
typedef rgl_status_t (*fn_rgl_node_points_transform_t)(rgl_node_t*, const rgl_mat3x4f*);
static fn_rgl_node_points_transform_t fn_rgl_node_points_transform = nullptr;

// 16. rgl_node_points_yield
typedef rgl_status_t (*fn_rgl_node_points_yield_t)(rgl_node_t*, const rgl_field_t*, int32_t);
static fn_rgl_node_points_yield_t fn_rgl_node_points_yield = nullptr;

// 17. rgl_node_points_format
typedef rgl_status_t (*fn_rgl_node_points_format_t)(rgl_node_t*, const rgl_field_t*, int32_t);
static fn_rgl_node_points_format_t fn_rgl_node_points_format = nullptr;

// 18. rgl_graph_destroy
typedef rgl_status_t (*fn_rgl_graph_destroy_t)(rgl_node_t);
static fn_rgl_graph_destroy_t fn_rgl_graph_destroy = nullptr;

// 19. rgl_graph_run
typedef rgl_status_t (*fn_rgl_graph_run_t)(rgl_node_t);
static fn_rgl_graph_run_t fn_rgl_graph_run = nullptr;

// 20. rgl_graph_get_result_size
typedef rgl_status_t (*fn_rgl_graph_get_result_size_t)(rgl_node_t, rgl_field_t, int32_t*, int32_t*);
static fn_rgl_graph_get_result_size_t fn_rgl_graph_get_result_size = nullptr;

// 21. rgl_graph_get_result_data
typedef rgl_status_t (*fn_rgl_graph_get_result_data_t)(rgl_node_t, rgl_field_t, void*);
static fn_rgl_graph_get_result_data_t fn_rgl_graph_get_result_data = nullptr;

// 22. rgl_graph_node_add_child
typedef rgl_status_t (*fn_rgl_graph_node_add_child_t)(rgl_node_t, rgl_node_t);
static fn_rgl_graph_node_add_child_t fn_rgl_graph_node_add_child = nullptr;

// 23. rgl_node_points_ros2_publish_with_qos
typedef rgl_status_t (*fn_rgl_node_points_ros2_publish_with_qos_t)(
    rgl_node_t*, const char*, const char*,
    rgl_qos_policy_reliability_t, rgl_qos_policy_durability_t,
    rgl_qos_policy_history_t, int32_t);
static fn_rgl_node_points_ros2_publish_with_qos_t fn_rgl_node_points_ros2_publish_with_qos = nullptr;

// 24. rgl_node_gaussian_noise_angular_ray
typedef rgl_status_t (*fn_rgl_node_gaussian_noise_angular_ray_t)(
    rgl_node_t*, float, float, rgl_axis_t);
static fn_rgl_node_gaussian_noise_angular_ray_t fn_rgl_node_gaussian_noise_angular_ray = nullptr;

// 25. rgl_node_gaussian_noise_angular_hitpoint
typedef rgl_status_t (*fn_rgl_node_gaussian_noise_angular_hitpoint_t)(
    rgl_node_t*, float, float, rgl_axis_t);
static fn_rgl_node_gaussian_noise_angular_hitpoint_t fn_rgl_node_gaussian_noise_angular_hitpoint = nullptr;

// 26. rgl_node_gaussian_noise_distance
typedef rgl_status_t (*fn_rgl_node_gaussian_noise_distance_t)(
    rgl_node_t*, float, float, float);
static fn_rgl_node_gaussian_noise_distance_t fn_rgl_node_gaussian_noise_distance = nullptr;

// 27. rgl_node_raytrace_configure_beam_divergence
typedef rgl_status_t (*fn_rgl_node_raytrace_configure_beam_divergence_t)(
    rgl_node_t, float, float);
static fn_rgl_node_raytrace_configure_beam_divergence_t fn_rgl_node_raytrace_configure_beam_divergence = nullptr;

// 28. rgl_node_raytrace_configure_return_mode
typedef rgl_status_t (*fn_rgl_node_raytrace_configure_return_mode_t)(
    rgl_node_t, rgl_return_mode_t);
static fn_rgl_node_raytrace_configure_return_mode_t fn_rgl_node_raytrace_configure_return_mode = nullptr;

// 29. rgl_node_points_udp_publish (UDP extension)
typedef rgl_status_t (*fn_rgl_node_points_udp_publish_t)(
    rgl_node_t*, rgl_lidar_model_t, rgl_udp_options_t,
    const char*, const char*, int32_t);
static fn_rgl_node_points_udp_publish_t fn_rgl_node_points_udp_publish = nullptr;

// 30. rgl_get_extension_info (always available)
typedef rgl_status_t (*fn_rgl_get_extension_info_t)(rgl_extension_t, int32_t*);
static fn_rgl_get_extension_info_t fn_rgl_get_extension_info = nullptr;

// ---------------------------------------------------------------------------
// Wrapper function definitions
// These provide the symbols the linker resolves to, replacing the .so linkage.
// The RGL headers declare these with extern "C" linkage (via RGL_API = NO_MANGLING RGL_VISIBLE).
// Since we do not link the .so, our definitions here become the active symbols.
// ---------------------------------------------------------------------------

// 1. rgl_get_last_error_string (void return)
void rgl_get_last_error_string(const char** out_error_string)
{
    if (!fn_rgl_get_last_error_string)
    {
        if (out_error_string) *out_error_string = "RGL library not loaded";
        return;
    }
    fn_rgl_get_last_error_string(out_error_string);
}

// 2. rgl_mesh_create
rgl_status_t rgl_mesh_create(rgl_mesh_t* out_mesh, const rgl_vec3f* vertices, int32_t vertex_count,
                             const rgl_vec3i* indices, int32_t index_count)
{
    if (!fn_rgl_mesh_create) return RGL_INVALID_STATE;
    return fn_rgl_mesh_create(out_mesh, vertices, vertex_count, indices, index_count);
}

// 3. rgl_mesh_destroy
rgl_status_t rgl_mesh_destroy(rgl_mesh_t mesh)
{
    if (!fn_rgl_mesh_destroy) return RGL_INVALID_STATE;
    return fn_rgl_mesh_destroy(mesh);
}

// 4. rgl_entity_create
rgl_status_t rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
    if (!fn_rgl_entity_create) return RGL_INVALID_STATE;
    return fn_rgl_entity_create(out_entity, scene, mesh);
}

// 5. rgl_entity_destroy
rgl_status_t rgl_entity_destroy(rgl_entity_t entity)
{
    if (!fn_rgl_entity_destroy) return RGL_INVALID_STATE;
    return fn_rgl_entity_destroy(entity);
}

// 6. rgl_entity_set_transform
rgl_status_t rgl_entity_set_transform(rgl_entity_t entity, const rgl_mat3x4f* transform)
{
    if (!fn_rgl_entity_set_transform) return RGL_INVALID_STATE;
    return fn_rgl_entity_set_transform(entity, transform);
}

// 7. rgl_scene_set_time
rgl_status_t rgl_scene_set_time(rgl_scene_t scene, uint64_t nanoseconds)
{
    if (!fn_rgl_scene_set_time) return RGL_INVALID_STATE;
    return fn_rgl_scene_set_time(scene, nanoseconds);
}

// 8. rgl_node_rays_from_mat3x4f
rgl_status_t rgl_node_rays_from_mat3x4f(rgl_node_t* node, const rgl_mat3x4f* rays, int32_t ray_count)
{
    if (!fn_rgl_node_rays_from_mat3x4f) return RGL_INVALID_STATE;
    return fn_rgl_node_rays_from_mat3x4f(node, rays, ray_count);
}

// 9. rgl_node_rays_set_ring_ids
rgl_status_t rgl_node_rays_set_ring_ids(rgl_node_t* node, const int32_t* ring_ids, int32_t ring_ids_count)
{
    if (!fn_rgl_node_rays_set_ring_ids) return RGL_INVALID_STATE;
    return fn_rgl_node_rays_set_ring_ids(node, ring_ids, ring_ids_count);
}

// 10. rgl_node_rays_set_range
rgl_status_t rgl_node_rays_set_range(rgl_node_t* node, const rgl_vec2f* ranges, int32_t ranges_count)
{
    if (!fn_rgl_node_rays_set_range) return RGL_INVALID_STATE;
    return fn_rgl_node_rays_set_range(node, ranges, ranges_count);
}

// 11. rgl_node_rays_transform
rgl_status_t rgl_node_rays_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
    if (!fn_rgl_node_rays_transform) return RGL_INVALID_STATE;
    return fn_rgl_node_rays_transform(node, transform);
}

// 12. rgl_node_raytrace
rgl_status_t rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene)
{
    if (!fn_rgl_node_raytrace) return RGL_INVALID_STATE;
    return fn_rgl_node_raytrace(node, scene);
}

// 13. rgl_node_raytrace_configure_non_hits
rgl_status_t rgl_node_raytrace_configure_non_hits(rgl_node_t node, float nearDistance, float farDistance)
{
    if (!fn_rgl_node_raytrace_configure_non_hits) return RGL_INVALID_STATE;
    return fn_rgl_node_raytrace_configure_non_hits(node, nearDistance, farDistance);
}

// 13b. rgl_node_raytrace_configure_mask
rgl_status_t rgl_node_raytrace_configure_mask(rgl_node_t node, const int8_t* rays_mask, int32_t rays_count)
{
    if (!fn_rgl_node_raytrace_configure_mask) return RGL_INVALID_STATE;
    return fn_rgl_node_raytrace_configure_mask(node, rays_mask, rays_count);
}

// 14. rgl_node_points_compact_by_field
rgl_status_t rgl_node_points_compact_by_field(rgl_node_t* node, rgl_field_t field)
{
    if (!fn_rgl_node_points_compact_by_field) return RGL_INVALID_STATE;
    return fn_rgl_node_points_compact_by_field(node, field);
}

// 15. rgl_node_points_transform
rgl_status_t rgl_node_points_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
    if (!fn_rgl_node_points_transform) return RGL_INVALID_STATE;
    return fn_rgl_node_points_transform(node, transform);
}

// 16. rgl_node_points_yield
rgl_status_t rgl_node_points_yield(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
    if (!fn_rgl_node_points_yield) return RGL_INVALID_STATE;
    return fn_rgl_node_points_yield(node, fields, field_count);
}

// 17. rgl_node_points_format
rgl_status_t rgl_node_points_format(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
    if (!fn_rgl_node_points_format) return RGL_INVALID_STATE;
    return fn_rgl_node_points_format(node, fields, field_count);
}

// 18. rgl_graph_destroy
rgl_status_t rgl_graph_destroy(rgl_node_t node)
{
    if (!fn_rgl_graph_destroy) return RGL_INVALID_STATE;
    return fn_rgl_graph_destroy(node);
}

// 19. rgl_graph_run
rgl_status_t rgl_graph_run(rgl_node_t node)
{
    if (!fn_rgl_graph_run) return RGL_INVALID_STATE;
    return fn_rgl_graph_run(node);
}

// 20. rgl_graph_get_result_size
rgl_status_t rgl_graph_get_result_size(rgl_node_t node, rgl_field_t field, int32_t* out_count, int32_t* out_size_of)
{
    if (!fn_rgl_graph_get_result_size) return RGL_INVALID_STATE;
    return fn_rgl_graph_get_result_size(node, field, out_count, out_size_of);
}

// 21. rgl_graph_get_result_data
rgl_status_t rgl_graph_get_result_data(rgl_node_t node, rgl_field_t field, void* data)
{
    if (!fn_rgl_graph_get_result_data) return RGL_INVALID_STATE;
    return fn_rgl_graph_get_result_data(node, field, data);
}

// 22. rgl_graph_node_add_child
rgl_status_t rgl_graph_node_add_child(rgl_node_t parent, rgl_node_t child)
{
    if (!fn_rgl_graph_node_add_child) return RGL_INVALID_STATE;
    return fn_rgl_graph_node_add_child(parent, child);
}

// 23. rgl_node_points_ros2_publish_with_qos
rgl_status_t rgl_node_points_ros2_publish_with_qos(
    rgl_node_t* node, const char* topic_name, const char* frame_id,
    rgl_qos_policy_reliability_t qos_reliability,
    rgl_qos_policy_durability_t qos_durability,
    rgl_qos_policy_history_t qos_history, int32_t qos_history_depth)
{
    if (!fn_rgl_node_points_ros2_publish_with_qos) return RGL_INVALID_STATE;
    return fn_rgl_node_points_ros2_publish_with_qos(
        node, topic_name, frame_id,
        qos_reliability, qos_durability,
        qos_history, qos_history_depth);
}

// 24. rgl_node_gaussian_noise_angular_ray
rgl_status_t rgl_node_gaussian_noise_angular_ray(
    rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis)
{
    if (!fn_rgl_node_gaussian_noise_angular_ray) return RGL_INVALID_STATE;
    return fn_rgl_node_gaussian_noise_angular_ray(node, mean, st_dev, rotation_axis);
}

// 25. rgl_node_gaussian_noise_angular_hitpoint
rgl_status_t rgl_node_gaussian_noise_angular_hitpoint(
    rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis)
{
    if (!fn_rgl_node_gaussian_noise_angular_hitpoint) return RGL_INVALID_STATE;
    return fn_rgl_node_gaussian_noise_angular_hitpoint(node, mean, st_dev, rotation_axis);
}

// 26. rgl_node_gaussian_noise_distance
rgl_status_t rgl_node_gaussian_noise_distance(
    rgl_node_t* node, float mean, float st_dev_base, float st_dev_rise_per_meter)
{
    if (!fn_rgl_node_gaussian_noise_distance) return RGL_INVALID_STATE;
    return fn_rgl_node_gaussian_noise_distance(node, mean, st_dev_base, st_dev_rise_per_meter);
}

// 27. rgl_node_raytrace_configure_beam_divergence
rgl_status_t rgl_node_raytrace_configure_beam_divergence(
    rgl_node_t node, float horizontal_beam_divergence, float vertical_beam_divergence)
{
    if (!fn_rgl_node_raytrace_configure_beam_divergence) return RGL_INVALID_STATE;
    return fn_rgl_node_raytrace_configure_beam_divergence(node, horizontal_beam_divergence, vertical_beam_divergence);
}

// 28. rgl_node_raytrace_configure_return_mode
rgl_status_t rgl_node_raytrace_configure_return_mode(
    rgl_node_t node, rgl_return_mode_t return_mode)
{
    if (!fn_rgl_node_raytrace_configure_return_mode) return RGL_INVALID_STATE;
    return fn_rgl_node_raytrace_configure_return_mode(node, return_mode);
}

// 29. rgl_node_points_udp_publish
rgl_status_t rgl_node_points_udp_publish(
    rgl_node_t* node, rgl_lidar_model_t lidar_model, rgl_udp_options_t udp_options,
    const char* device_ip, const char* dest_ip, int32_t dest_port)
{
    if (!fn_rgl_node_points_udp_publish) return RGL_INVALID_STATE;
    return fn_rgl_node_points_udp_publish(
        node, lidar_model, udp_options, device_ip, dest_ip, dest_port);
}

// 30. rgl_get_extension_info
rgl_status_t rgl_get_extension_info(rgl_extension_t extension, int32_t* out_available)
{
    if (!fn_rgl_get_extension_info)
    {
        if (out_available) *out_available = 0;
        return RGL_INVALID_STATE;
    }
    return fn_rgl_get_extension_info(extension, out_available);
}

// ---------------------------------------------------------------------------
// RGLDynLoader implementation
// ---------------------------------------------------------------------------

#define LOAD_FN(name) \
    fn_##name = (fn_##name##_t)dlsym(Handle, #name); \
    if (!fn_##name) { fprintf(stderr, "RGLDynLoader: warning: failed to resolve %s: %s\n", #name, dlerror()); allResolved = false; }

bool RGLDynLoader::Load(const char* LibPath)
{
    if (Handle)
    {
        return true; // Already loaded
    }

    Handle = dlopen(LibPath, RTLD_NOW | RTLD_LOCAL);
    if (!Handle)
    {
        fprintf(stderr, "RGLDynLoader: dlopen failed: %s\n", dlerror());
        return false;
    }

    bool allResolved = true;

    LOAD_FN(rgl_get_last_error_string)
    LOAD_FN(rgl_mesh_create)
    LOAD_FN(rgl_mesh_destroy)
    LOAD_FN(rgl_entity_create)
    LOAD_FN(rgl_entity_destroy)
    LOAD_FN(rgl_entity_set_transform)
    LOAD_FN(rgl_scene_set_time)
    LOAD_FN(rgl_node_rays_from_mat3x4f)
    LOAD_FN(rgl_node_rays_set_ring_ids)
    LOAD_FN(rgl_node_rays_set_range)
    LOAD_FN(rgl_node_rays_transform)
    LOAD_FN(rgl_node_raytrace)
    LOAD_FN(rgl_node_raytrace_configure_non_hits)
    LOAD_FN(rgl_node_raytrace_configure_mask)
    LOAD_FN(rgl_node_points_compact_by_field)
    LOAD_FN(rgl_node_points_transform)
    LOAD_FN(rgl_node_points_yield)
    LOAD_FN(rgl_node_points_format)
    LOAD_FN(rgl_graph_destroy)
    LOAD_FN(rgl_graph_run)
    LOAD_FN(rgl_graph_get_result_size)
    LOAD_FN(rgl_graph_get_result_data)
    LOAD_FN(rgl_graph_node_add_child)
    LOAD_FN(rgl_node_points_ros2_publish_with_qos)
    LOAD_FN(rgl_node_gaussian_noise_angular_ray)
    LOAD_FN(rgl_node_gaussian_noise_angular_hitpoint)
    LOAD_FN(rgl_node_gaussian_noise_distance)
    LOAD_FN(rgl_node_raytrace_configure_beam_divergence)
    LOAD_FN(rgl_node_raytrace_configure_return_mode)
    LOAD_FN(rgl_node_points_udp_publish)
    LOAD_FN(rgl_get_extension_info)

    if (!allResolved)
    {
        fprintf(stderr, "RGLDynLoader: some symbols could not be resolved, but library is loaded.\n");
    }

    return true;
}

#undef LOAD_FN

void RGLDynLoader::Unload()
{
    if (Handle)
    {
        dlclose(Handle);
        Handle = nullptr;
    }

    // Reset all function pointers
    fn_rgl_get_last_error_string = nullptr;
    fn_rgl_mesh_create = nullptr;
    fn_rgl_mesh_destroy = nullptr;
    fn_rgl_entity_create = nullptr;
    fn_rgl_entity_destroy = nullptr;
    fn_rgl_entity_set_transform = nullptr;
    fn_rgl_scene_set_time = nullptr;
    fn_rgl_node_rays_from_mat3x4f = nullptr;
    fn_rgl_node_rays_set_ring_ids = nullptr;
    fn_rgl_node_rays_set_range = nullptr;
    fn_rgl_node_rays_transform = nullptr;
    fn_rgl_node_raytrace = nullptr;
    fn_rgl_node_raytrace_configure_non_hits = nullptr;
    fn_rgl_node_raytrace_configure_mask = nullptr;
    fn_rgl_node_points_compact_by_field = nullptr;
    fn_rgl_node_points_transform = nullptr;
    fn_rgl_node_points_yield = nullptr;
    fn_rgl_node_points_format = nullptr;
    fn_rgl_graph_destroy = nullptr;
    fn_rgl_graph_run = nullptr;
    fn_rgl_graph_get_result_size = nullptr;
    fn_rgl_graph_get_result_data = nullptr;
    fn_rgl_graph_node_add_child = nullptr;
    fn_rgl_node_points_ros2_publish_with_qos = nullptr;
    fn_rgl_node_gaussian_noise_angular_ray = nullptr;
    fn_rgl_node_gaussian_noise_angular_hitpoint = nullptr;
    fn_rgl_node_gaussian_noise_distance = nullptr;
    fn_rgl_node_raytrace_configure_beam_divergence = nullptr;
    fn_rgl_node_raytrace_configure_return_mode = nullptr;
    fn_rgl_node_points_udp_publish = nullptr;
    fn_rgl_get_extension_info = nullptr;
}

bool RGLDynLoader::IsLoaded()
{
    return Handle != nullptr;
}

// --- RclcppBridge dynamic loader ---

static void* GRclcppBridgeHandle = nullptr;
typedef bool (*fn_rclcpp_bridge_init_t)(void);
typedef bool (*fn_rclcpp_bridge_ok_t)(void);
typedef void (*fn_rclcpp_bridge_shutdown_t)(void);
static fn_rclcpp_bridge_init_t fn_rclcpp_bridge_init = nullptr;
static fn_rclcpp_bridge_ok_t fn_rclcpp_bridge_ok = nullptr;
static fn_rclcpp_bridge_shutdown_t fn_rclcpp_bridge_shutdown = nullptr;

bool RclcppBridge::Load(const char* LibPath)
{
    if (GRclcppBridgeHandle) return true;
    GRclcppBridgeHandle = dlopen(LibPath, RTLD_NOW | RTLD_GLOBAL);
    if (!GRclcppBridgeHandle) return false;

    fn_rclcpp_bridge_init = (fn_rclcpp_bridge_init_t)dlsym(GRclcppBridgeHandle, "rclcpp_bridge_init");
    fn_rclcpp_bridge_ok = (fn_rclcpp_bridge_ok_t)dlsym(GRclcppBridgeHandle, "rclcpp_bridge_ok");
    fn_rclcpp_bridge_shutdown = (fn_rclcpp_bridge_shutdown_t)dlsym(GRclcppBridgeHandle, "rclcpp_bridge_shutdown");
    return fn_rclcpp_bridge_init != nullptr;
}

bool RclcppBridge::Init()
{
    if (!fn_rclcpp_bridge_init) return false;
    return fn_rclcpp_bridge_init();
}

void RclcppBridge::Shutdown()
{
    if (fn_rclcpp_bridge_shutdown) fn_rclcpp_bridge_shutdown();
}

void RclcppBridge::Unload()
{
    fn_rclcpp_bridge_init = nullptr;
    fn_rclcpp_bridge_ok = nullptr;
    fn_rclcpp_bridge_shutdown = nullptr;
    if (GRclcppBridgeHandle) { dlclose(GRclcppBridgeHandle); GRclcppBridgeHandle = nullptr; }
}

bool RclcppBridge::IsLoaded()
{
    return GRclcppBridgeHandle != nullptr;
}
