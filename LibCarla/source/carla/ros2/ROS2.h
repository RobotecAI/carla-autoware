// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Buffer.h"
#include "carla/BufferView.h"
#include "carla/geom/Transform.h"
#include "carla/ros2/ROS2CallbackData.h"
#include "carla/streaming/detail/Types.h"

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <vector>

// forward declarations
class AActor;
namespace carla {
  namespace geom {
    class GeoLocation;
    struct Vector3D;
  }
  namespace sensor {
    namespace data {
      struct DVSEvent;
      class LidarData;
      class SemanticLidarData;
      class RadarData;
      class VehicleStatusEvent;
    }
    namespace s11n{
      struct VehicleStatusData;
    }
  }
}

namespace carla {
namespace ros2 {

  class CarlaPublisher;
  class CarlaTransformPublisher;
  class CarlaClockPublisher;
  class CarlaEgoVehicleControlSubscriber;
  class AutowareController;
  class AutowarePublisher;
  class BasicSubscriber;
  class BasicPublisher;

class ROS2
{
  public:

  // deleting copy constructor for singleton
  ROS2(const ROS2& obj) = delete;
  static std::shared_ptr<ROS2> GetInstance() {
    if (!_instance)
      _instance = std::shared_ptr<ROS2>(new ROS2);
    return _instance;
  }

  // utility
  static std::pair<int32_t, uint32_t> Carla2RosTime(double timestamp) {
    double integral;
    const double fractional = modf(timestamp, &integral);
    const double multiplier = 1000000000.0;
    const auto seconds = static_cast<int32_t>(integral);
    const auto nanoseconds = static_cast<uint32_t>(fractional * multiplier);

    return std::make_pair(seconds, nanoseconds);
  }

  // general
  void Enable(bool enable);
  void Shutdown();
  bool IsEnabled() { return _enabled; }
  void SetFrame(uint64_t frame);
  void SetTimestamp(double timestamp);

  // TF publishing control
  void SetPublishTF(bool publish_tf);
  bool GetPublishTF() const { return _publish_tf; }

  // ros_name managing
  void AddActorRosName(void *actor, std::string ros_name);
  void AddActorParentRosName(void *actor, void* parent);
  void RemoveActorRosName(void *actor);
  void UpdateActorRosName(void *actor, std::string ros_name);
  std::string GetActorRosName(void *actor);
  std::string GetActorParentRosName(void *actor);

  // ros_topic_name managing
  void AddActorRosTopicName(void *actor, std::string ros_topic_name);
  void RemoveActorRosTopicName(void *actor);
  void UpdateActorRosTopicName(void *actor, std::string ros_name);
  std::string GetActorRosTopicName(void *actor);

  // callbacks
  void AddActorCallback(void* actor, std::string ros_name, ActorCallback callback);
  void RemoveActorCallback(void* actor);
  void RemoveBasicSubscriberCallback(void* actor);
  void AddBasicSubscriberCallback(void* actor, std::string ros_name, ActorMessageCallback callback);

  // enabling streams to publish
  void EnableStream(carla::streaming::detail::stream_id_type id) { _publish_stream.insert(id); }
  bool IsStreamEnabled(carla::streaming::detail::stream_id_type id) { return _publish_stream.count(id) > 0; }
  void ResetStreams() { _publish_stream.clear(); }

  // receiving data to publish
  void ProcessDataFromCamera(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      int W, int H, float Fov,
      const carla::SharedBufferView buffer,
      void *actor = nullptr);
  void ProcessDataFromGNSS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::geom::GeoLocation &data,
      const carla::geom::Transform &sensor_world_transform,
      void *actor = nullptr);
  // ue5-dev compatible overload (without sensor_world_transform)
  void ProcessDataFromGNSS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::geom::GeoLocation &data,
      void *actor = nullptr) {
    ProcessDataFromGNSS(sensor_type, stream_id, sensor_transform, data, sensor_transform, actor);
  }
  void ProcessDataFromAutowareGNSS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::geom::GeoLocation &data,
      const carla::geom::Transform &sensor_world_transform,
      const double mgrs_offset_position[3],
      void *actor = nullptr);
  void ProcessDataFromIMU(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::geom::Vector3D accelerometer,
      carla::geom::Vector3D gyroscope,
      float compass,
      void *actor = nullptr);
  void ProcessDataFromIMUStamped(
      double time_offset_seconds,
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::geom::Vector3D accelerometer,
      carla::geom::Vector3D gyroscope,
      float compass,
      void *actor = nullptr);
  void ProcessDataFromDVS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::SharedBufferView buffer,
      int W, int H, float Fov,
      void *actor = nullptr);
  void ProcessDataFromLidar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      uint32_t channel_count,
      float upper_fov_limit,
      float lower_fov_limit,
      carla::sensor::data::LidarData &data,
      void *actor = nullptr);
  // ue5-dev compatible overload (without channel_count, fov)
  void ProcessDataFromLidar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::LidarData &data,
      void *actor = nullptr) {
    ProcessDataFromLidar(sensor_type, stream_id, sensor_transform, 0, 0.f, 0.f, data, actor);
  }
  void ProcessDataFromSemanticLidar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::SemanticLidarData &data,
      void *actor = nullptr);
  void ProcessDataFromRadar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::sensor::data::RadarData &data,
      void *actor = nullptr);
  void ProcessDataFromObstacleDetection(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      AActor *first_actor,
      AActor *second_actor,
      float distance,
      void *actor = nullptr);
  void ProcessDataFromCollisionSensor(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      uint32_t other_actor,
      carla::geom::Vector3D impulse,
      void* actor);
  void ProcessDataFromStatusSensor(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const sensor::s11n::VehicleStatusData &data,
      void *vehicle_actor,
      void *actor);

    uint32_t GetDomainId() const noexcept { return _domain_id; }

  private:
  std::pair<std::shared_ptr<CarlaPublisher>, std::shared_ptr<CarlaTransformPublisher>> GetOrCreateSensor(int type, carla::streaming::detail::stream_id_type id, void* actor);

  // sigleton
  ROS2() {};

  bool ObtainDomainId() noexcept;

  static std::shared_ptr<ROS2> _instance;

  bool _enabled { false };
  bool _publish_tf { true };
  uint64_t _frame { 0 };
  int32_t _seconds { 0 };
  uint32_t _nanoseconds { 0 };
  uint32_t _domain_id { 0U };
  std::unordered_map<void *, std::string> _actor_ros_name;
  std::unordered_map<void *, std::string> _actor_ros_topic_name;
  std::unordered_map<void *, std::vector<void*> > _actor_parent_ros_name;
  std::shared_ptr<CarlaEgoVehicleControlSubscriber> _controller;
  std::shared_ptr<AutowareController> _autoware_controller;
  std::shared_ptr<AutowarePublisher> _autoware_publisher;
  std::shared_ptr<CarlaClockPublisher> _clock_publisher;
  std::unordered_map<void *, std::shared_ptr<CarlaPublisher>> _publishers;
  std::unordered_map<void *, std::shared_ptr<CarlaTransformPublisher>> _transforms;
  std::unordered_set<carla::streaming::detail::stream_id_type> _publish_stream;
  std::unordered_map<void *, ActorCallback> _actor_callbacks;
#if defined(WITH_ROS2_DEMO)
  std::shared_ptr<BasicSubscriber> _basic_subscriber;
  std::shared_ptr<BasicPublisher> _basic_publisher;
  std::unordered_map<void *, ActorMessageCallback> _actor_message_callbacks;
#endif
};

} // namespace ros2
} // namespace carla
