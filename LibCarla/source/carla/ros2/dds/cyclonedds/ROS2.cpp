// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
//
// CycloneDDS-specific ROS2 implementation with full Autoware integration.
// Existing (ue5-dev) publishers use Init() with no args.
// Autoware publishers/subscribers use their own Init patterns (DomainId).

#include <charconv>
#include <cstdlib>
#include <cstring>
#include "carla/Logging.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/GeoLocation.h"
#include "carla/geom/Vector3D.h"
#include "carla/sensor/data/DVSEvent.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/RadarData.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/VehicleStatusEvent.h"
#include "carla/sensor/s11n/ImageSerializer.h"
#include "carla/sensor/s11n/SensorHeaderSerializer.h"

#include "carla/ros2/data_types.h"
#include "carla/ros2/publishers/AutowarePublisher.h"
#include "carla/ros2/publishers/AutowareGNSSPublisher.h"
#include "carla/ros2/publishers/CarlaPublisher.h"
#include "carla/ros2/publishers/CarlaClockPublisher.h"
#include "carla/ros2/publishers/CarlaRGBCameraPublisher.h"
#include "carla/ros2/publishers/CarlaDepthCameraPublisher.h"
#include "carla/ros2/publishers/CarlaNormalsCameraPublisher.h"
#include "carla/ros2/publishers/CarlaOpticalFlowCameraPublisher.h"
#include "carla/ros2/publishers/CarlaSSCameraPublisher.h"
#include "carla/ros2/publishers/CarlaISCameraPublisher.h"
#include "carla/ros2/publishers/CarlaDVSCameraPublisher.h"
#include "carla/ros2/publishers/CarlaLidarPublisher.h"
#include "carla/ros2/publishers/CarlaSemanticLidarPublisher.h"
#include "carla/ros2/publishers/CarlaRadarPublisher.h"
#include "carla/ros2/publishers/CarlaIMUPublisher.h"
#include "carla/ros2/publishers/CarlaGNSSPublisher.h"
#include "carla/ros2/publishers/CarlaMapSensorPublisher.h"
#include "carla/ros2/publishers/CarlaSpeedometerSensor.h"
#include "carla/ros2/publishers/CarlaTransformPublisher.h"
#include "carla/ros2/publishers/CarlaCollisionPublisher.h"
#include "carla/ros2/publishers/CarlaLineInvasionPublisher.h"
#include "carla/ros2/publishers/BasicPublisher.h"

#include "carla/ros2/subscribers/CarlaSubscriber.h"
#include "carla/ros2/subscribers/AutowareController.h"
#include "carla/ros2/subscribers/CarlaEgoVehicleControlSubscriber.h"
#if defined(WITH_ROS2_DEMO)
  #include "carla/ros2/subscribers/BasicSubscriber.h"
#endif

#include <vector>

namespace carla {
namespace ros2 {

// static fields
std::shared_ptr<ROS2> ROS2::_instance;

// list of sensors (should be equal to the list of SensorsRegistry
enum ESensors {
  CollisionSensor,
  DepthCamera,
  NormalsCamera,
  DVSCamera,
  GnssSensor,
  InertialMeasurementUnit,
  LaneInvasionSensor,
  ObstacleDetectionSensor,
  OpticalFlowCamera,
  Radar,
  RayCastSemanticLidar,
  RayCastLidar,
  RssSensor,
  SceneCaptureCamera,
  SemanticSegmentationCamera,
  InstanceSegmentationCamera,
  WorldObserver,
  CameraGBufferUint8,
  CameraGBufferFloat
};

void ROS2::Enable(bool enable) {
  _enabled = enable;
  ObtainDomainId();
  log_info("ROS2 enabled: ", _enabled);
  _clock_publisher = std::make_shared<CarlaClockPublisher>("clock", "");
  _clock_publisher->Init();
#if defined(WITH_ROS2_DEMO)
  _basic_publisher = std::make_shared<BasicPublisher>("basic_publisher", "");
  _basic_publisher->Init();
#endif
}

void ROS2::SetFrame(uint64_t frame) {
  _frame = frame;
   //log_info("ROS2 new frame: ", _frame);
   if (_controller) {
    void* actor = _controller->GetVehicle();
    if (_controller->IsAlive()) {
      if (_controller->HasNewMessage()) {
        auto it = _actor_callbacks.find(actor);
        if (it != _actor_callbacks.end()) {
          VehicleControl control = _controller->GetMessage();
          it->second(actor, control);
        }
      }
    } else {
      RemoveActorCallback(actor);
    }
   }
  if (_autoware_controller) {  // Autoware input has priority (acceleration control from /control/command/control_cmd)
    void* actor = _autoware_controller->GetVehicle();
    if (_autoware_controller->HasNewControl()) {
      auto it = _actor_callbacks.find(actor);
      if (it != _actor_callbacks.cend()) {
        const auto control = _autoware_controller->GetControl();
        it->second(actor, ROS2CallbackData(control));
      }
    }
  }
#if defined(WITH_ROS2_DEMO)
   if (_basic_subscriber)
   {
    void* actor = _basic_subscriber->GetActor();
    if (!_basic_subscriber->IsAlive()){
        RemoveBasicSubscriberCallback(actor);
    }
    if (actor&& _basic_subscriber->HasNewMessage())
    {
      auto it = _actor_message_callbacks.find(actor);
      if (it != _actor_message_callbacks.end()) {
        MessageControl control;
        control.message = _basic_subscriber->GetMessage();
        it->second(actor, control);
      }
    }
   }
#endif
}

void ROS2::SetTimestamp(double timestamp) {
  std::tie(_seconds, _nanoseconds) = Carla2RosTime(timestamp);
  _clock_publisher->SetData(_seconds, _nanoseconds);
  _clock_publisher->Publish();
#if defined(WITH_ROS2_DEMO)
  _basic_publisher->SetData("Hello from Carla!");
  _basic_publisher->Publish();
#endif
}

void ROS2::SetPublishTF(bool publish_tf) {
  _publish_tf = publish_tf;
}

void ROS2::AddActorRosName(void *actor, std::string ros_name) {
  _actor_ros_name.insert({actor, ros_name});
}

void ROS2::AddActorParentRosName(void *actor, void* parent) {
  auto it = _actor_parent_ros_name.find(actor);
  if (it != _actor_parent_ros_name.end()) {
    it->second.push_back(parent);
  } else {
    _actor_parent_ros_name.insert({actor, {parent}});
  }
}

void ROS2::RemoveActorRosName(void *actor) {
  _actor_ros_name.erase(actor);
  _actor_parent_ros_name.erase(actor);

  _publishers.erase(actor);
  _transforms.erase(actor);
}

void ROS2::UpdateActorRosName(void *actor, std::string ros_name) {
  auto it = _actor_ros_name.find(actor);
  if (it != _actor_ros_name.end()) {
    it->second = ros_name;
  }
}

std::string ROS2::GetActorRosName(void *actor) {
  auto it = _actor_ros_name.find(actor);
  if (it != _actor_ros_name.end()) {
    return it->second;
  } else {
    return std::string("");
  }
}

std::string ROS2::GetActorParentRosName(void *actor) {
  auto it = _actor_parent_ros_name.find(actor);
  if (it != _actor_parent_ros_name.end())
  {
    const std::string current_actor_name = GetActorRosName(actor);
    std::string parent_name;
    for (auto parent_it = it->second.cbegin(); parent_it != it->second.cend(); ++parent_it)
    {
      const std::string name = GetActorRosName(*parent_it);
      if (name == current_actor_name)
      {
        continue;
      }
      if (name.empty())
      {
        continue;
      }
      parent_name = name + '/' + parent_name;
    }
    if (parent_name.back() == '/')
      parent_name.pop_back();
    return parent_name;
  }
  else
    return std::string("");
}

void ROS2::AddActorRosTopicName(void *actor, std::string ros_topic_name) {
  _actor_ros_topic_name.insert({actor, ros_topic_name});
}

void ROS2::RemoveActorRosTopicName(void *actor) {
  _actor_ros_topic_name.erase(actor);

  _publishers.erase(actor);
  _transforms.erase(actor);
}

void ROS2::UpdateActorRosTopicName(void *actor, std::string ros_topic_name) {
  auto it = _actor_ros_topic_name.find(actor);
  if (it != _actor_ros_topic_name.end()) {
    it->second = ros_topic_name;
  }
}

std::string ROS2::GetActorRosTopicName(void *actor) {
  auto it = _actor_ros_topic_name.find(actor);
  if (it != _actor_ros_topic_name.end()) {
    return it->second;
  } else {
    return std::string("");
  }
}

void ROS2::AddBasicSubscriberCallback(void* actor, std::string ros_name, ActorMessageCallback callback) {
  #if defined(WITH_ROS2_DEMO)
  _actor_message_callbacks.insert({actor, std::move(callback)});

  _basic_subscriber.reset();
  _basic_subscriber = std::make_shared<BasicSubscriber>(actor, ros_name.c_str());
  _basic_subscriber->Init();
  #endif
}

void ROS2::RemoveBasicSubscriberCallback(void* actor) {
  #if defined(WITH_ROS2_DEMO)
  _basic_subscriber.reset();
  _actor_message_callbacks.erase(actor);
  #endif
}

void ROS2::AddActorCallback(void* actor, std::string ros_name, ActorCallback callback) {
  _actor_callbacks.insert({actor, std::move(callback)});

  _controller.reset();
  _controller = std::make_shared<CarlaEgoVehicleControlSubscriber>(actor, ros_name.c_str());
  _controller->Init();

  _autoware_controller.reset();
  _autoware_controller = std::make_shared<AutowareController>(actor, _domain_id);

  _autoware_publisher.reset();
  _autoware_publisher = std::make_shared<AutowarePublisher>(actor, _domain_id);
}

void ROS2::RemoveActorCallback(void* actor) {
  _controller.reset();
  _autoware_controller.reset();
  _autoware_publisher.reset();
  _actor_callbacks.erase(actor);
}

bool ROS2::ObtainDomainId() noexcept
{
  const char* domain_id = std::getenv("ROS_DOMAIN_ID");
  if (!domain_id) {
    return false; // environment variable not set
  }

  int new_domain_id{};
  const char* end = domain_id + std::strlen(domain_id);

  // parse directly with from_chars
  auto [ptr, ec] = std::from_chars(domain_id, end, new_domain_id);

  if (ec != std::errc{} || ptr != end) {
    return false; // parse error, overflow, or leftover garbage
  }

  if (new_domain_id < 0) {
    return false; // negative IDs are invalid
  }

  _domain_id = new_domain_id;
  return true;
}

std::pair<std::shared_ptr<CarlaPublisher>, std::shared_ptr<CarlaTransformPublisher>> ROS2::GetOrCreateSensor(int type, carla::streaming::detail::stream_id_type id, void* actor) {
  auto it_publishers = _publishers.find(actor);
  auto it_transforms = _transforms.find(actor);
  std::shared_ptr<CarlaPublisher> publisher {};
  std::shared_ptr<CarlaTransformPublisher> transform {};
  if (it_publishers != _publishers.end()) {
    publisher = it_publishers->second;
    if (it_transforms != _transforms.end()) {
      transform = it_transforms->second;
    }
  } else {
    //Sensor not found, creating one of the given type
    const std::string string_id = std::to_string(id);
    std::string ros_name = GetActorRosName(actor);
    std::string parent_ros_name = GetActorParentRosName(actor);
    std::string ros_topic_name = GetActorRosTopicName(actor);
    switch(type) {
      case ESensors::CollisionSensor: {
        if (ros_name == "collision__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaCollisionPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        if (new_publisher->Init()) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::DepthCamera: {
        if (ros_name == "depth__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaDepthCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::NormalsCamera: {
        if (ros_name == "normals__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaNormalsCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::DVSCamera: {
        if (ros_name == "dvs__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaDVSCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::GnssSensor: {
        if (ros_name == "gnss__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        if (false) {  // TODO: Add some configurable switch whether the sensor is Autoware or Carla
          auto new_publisher = std::make_shared<CarlaGNSSPublisher>(ros_name.c_str(), parent_ros_name.c_str());
          if (!ros_topic_name.empty())
            new_publisher->topic_name(std::string(ros_topic_name));
          if (new_publisher->Init()) {
            _publishers.insert({actor, new_publisher});
            publisher = new_publisher;
          }
        } else {
          auto new_publisher = std::make_shared<AutowareGNSSPublisher>(ros_name.c_str(), parent_ros_name.c_str(), ros_topic_name.c_str());
          if (new_publisher->Init(_domain_id)) {
            _publishers.insert({actor, new_publisher});
            publisher = new_publisher;
          }
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::InertialMeasurementUnit: {
        if (ros_name == "imu__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaIMUPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig imu_config;
        imu_config.domain_id = _domain_id;
        imu_config.reliability_qos = ReliabilityQoS::RELIABLE;
        imu_config.durability_qos = DurabilityQoS::VOLATILE;
        imu_config.history_qos = HistoryQoS::KEEP_LAST;
        imu_config.history_qos_depth = 1000;
        if (new_publisher->Init(imu_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::LaneInvasionSensor: {
        if (ros_name == "lane_invasion__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaLineInvasionPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        if (new_publisher->Init()) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::ObstacleDetectionSensor: {
        std::cerr << "Obstacle detection sensor does not have an available publisher" << std::endl;
      } break;
      case ESensors::OpticalFlowCamera: {
        if (ros_name == "optical_flow__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaOpticalFlowCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::Radar: {
        if (ros_name == "radar__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaRadarPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        if (new_publisher->Init()) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::RayCastSemanticLidar: {
        if (ros_name == "ray_cast_semantic__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaSemanticLidarPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        if (new_publisher->Init()) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::RayCastLidar: {
        if (ros_name == "ray_cast__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaLidarPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig lidar_config;
        lidar_config.domain_id = _domain_id;
        lidar_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        lidar_config.durability_qos = DurabilityQoS::VOLATILE;
        lidar_config.history_qos = HistoryQoS::KEEP_LAST;
        lidar_config.history_qos_depth = 5;
        if (new_publisher->Init(lidar_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::RssSensor: {
        std::cerr << "RSS sensor does not have an available publisher" << std::endl;
      } break;
      case ESensors::SceneCaptureCamera: {
        if (ros_name == "rgb__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaRGBCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::SemanticSegmentationCamera: {
        if (ros_name == "semantic_segmentation__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaSSCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::InstanceSegmentationCamera: {
        if (ros_name == "instance_segmentation__") {
          ros_name.pop_back();
          ros_name.pop_back();
          ros_name += string_id;
          UpdateActorRosName(actor, ros_name);
        }
        auto new_publisher = std::make_shared<CarlaISCameraPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (!ros_topic_name.empty())
          new_publisher->topic_name(std::string(ros_topic_name));
        TopicConfig cam_config;
        cam_config.domain_id = _domain_id;
        cam_config.reliability_qos = ReliabilityQoS::BEST_EFFORT;
        cam_config.durability_qos = DurabilityQoS::VOLATILE;
        cam_config.history_qos = HistoryQoS::KEEP_LAST;
        cam_config.history_qos_depth = 1;
        cam_config.suffix = "/image_raw";
        if (new_publisher->Init(cam_config)) {
          _publishers.insert({actor, new_publisher});
          publisher = new_publisher;
        }
        auto new_transform = std::make_shared<CarlaTransformPublisher>(ros_name.c_str(), parent_ros_name.c_str());
        if (new_transform->Init()) {
          _transforms.insert({actor, new_transform});
          transform = new_transform;
        }
      } break;
      case ESensors::WorldObserver: {
        std::cerr << "World obserser does not have an available publisher" << std::endl;
      } break;
      case ESensors::CameraGBufferUint8: {
        std::cerr << "Camera GBuffer uint8 does not have an available publisher" << std::endl;
      } break;
      case ESensors::CameraGBufferFloat: {
        std::cerr << "Camera GBuffer float does not have an available publisher" << std::endl;
      } break;
      default: {
        std::cerr << "Unknown sensor type" << std::endl;
      }
    }
  }
  return { publisher, transform };
}

void ROS2::ProcessDataFromCamera(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    int W, int H, float Fov,
    const carla::SharedBufferView buffer,
    void *actor) {

  switch (sensor_type) {
    case ESensors::CollisionSensor:
      log_info("Sensor Collision to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      break;
    case ESensors::DepthCamera:
      {
        log_info("Sensor DepthCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
        auto sensors = GetOrCreateSensor(ESensors::DepthCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaDepthCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaDepthCameraPublisher>(sensors.first);
          const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::NormalsCamera:
      log_info("Sensor NormalsCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::NormalsCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaNormalsCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaNormalsCameraPublisher>(sensors.first);
          const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::LaneInvasionSensor:
      log_info("Sensor LaneInvasionSensor to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::LaneInvasionSensor, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaLineInvasionPublisher> publisher = std::dynamic_pointer_cast<CarlaLineInvasionPublisher>(sensors.first);
          publisher->SetData(_seconds, _nanoseconds, (const int32_t*) buffer->data());
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::OpticalFlowCamera:
      log_info("Sensor OpticalFlowCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::OpticalFlowCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaOpticalFlowCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaOpticalFlowCameraPublisher>(sensors.first);
          const carla::sensor::s11n::OpticalFlowImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::OpticalFlowImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const float*) (buffer->data() + carla::sensor::s11n::OpticalFlowImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::RssSensor:
      log_info("Sensor RssSensor to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      break;
    case ESensors::SceneCaptureCamera:
    {
      log_info("Sensor SceneCaptureCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::SceneCaptureCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaRGBCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaRGBCameraPublisher>(sensors.first);
          const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    }
    case ESensors::SemanticSegmentationCamera:
      log_info("Sensor SemanticSegmentationCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::SemanticSegmentationCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaSSCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaSSCameraPublisher>(sensors.first);
          const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::InstanceSegmentationCamera:
      log_info("Sensor InstanceSegmentationCamera to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      {
        auto sensors = GetOrCreateSensor(ESensors::InstanceSegmentationCamera, stream_id, actor);
        if (sensors.first) {
          std::shared_ptr<CarlaISCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaISCameraPublisher>(sensors.first);
          const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
            reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
          if (!header)
            return;
          if (!publisher->HasBeenInitialized())
            publisher->InitInfoData(0, 0, H, W, Fov, true);
          publisher->SetImageData(_seconds, _nanoseconds, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
          publisher->SetCameraInfoData(_seconds, _nanoseconds);
          publisher->Publish();
        }
        if (sensors.second && _publish_tf) {
          std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
          publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
          publisher->Publish();
        }
      }
      break;
    case ESensors::WorldObserver:
      log_info("Sensor WorldObserver to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      break;
    case ESensors::CameraGBufferUint8:
      log_info("Sensor CameraGBufferUint8 to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      break;
    case ESensors::CameraGBufferFloat:
      log_info("Sensor CameraGBufferFloat to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
      break;
    default:
      log_info("Sensor to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "buffer.", buffer->size());
  }
}

// ue5-dev compatible overload is provided as inline in ROS2.h


void ROS2::ProcessDataFromGNSS(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    const carla::geom::GeoLocation &data,
    const carla::geom::Transform &sensor_world_transform,
    void *actor) {
  log_info("Sensor GnssSensor to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "geo.", data.latitude, data.longitude, data.altitude);
  auto sensors = GetOrCreateSensor(ESensors::GnssSensor, stream_id, actor);
  if (sensors.first) {
    if (std::shared_ptr<CarlaGNSSPublisher> carla_publisher = std::dynamic_pointer_cast<CarlaGNSSPublisher>(sensors.first)) {
      carla_publisher->SetData(_seconds, _nanoseconds, reinterpret_cast<const double*>(&data));
      carla_publisher->Publish();
    } else if (const auto autoware_publisher = std::dynamic_pointer_cast<AutowareGNSSPublisher>(sensors.first)) {
      autoware_publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_world_transform.location, (const float*)&sensor_world_transform.rotation);
      autoware_publisher->Publish();
    }
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromAutowareGNSS(
  uint64_t sensor_type,
  carla::streaming::detail::stream_id_type stream_id,
  const carla::geom::Transform sensor_transform,
  const carla::geom::GeoLocation &data,
  const carla::geom::Transform &sensor_world_transform,
  const double mgrs_offset_position[3],
  void *actor) {
  log_info("Sensor GnssSensor to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "geo.", data.latitude, data.longitude, data.altitude);
  auto sensors = GetOrCreateSensor(ESensors::GnssSensor, stream_id, actor);
  if (sensors.first) {
    if (const auto autoware_publisher = std::dynamic_pointer_cast<AutowareGNSSPublisher>(sensors.first)) {
      autoware_publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_world_transform.location, (const float*)&sensor_world_transform.rotation, mgrs_offset_position);
      autoware_publisher->Publish();
    }
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromIMU(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    carla::geom::Vector3D accelerometer,
    carla::geom::Vector3D gyroscope,
    float compass,
    void *actor) {
  log_info("Sensor InertialMeasurementUnit to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "imu.", accelerometer.x, gyroscope.x, compass);
  auto sensors = GetOrCreateSensor(ESensors::InertialMeasurementUnit, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaIMUPublisher> publisher = std::dynamic_pointer_cast<CarlaIMUPublisher>(sensors.first);
    publisher->SetData(_seconds, _nanoseconds, reinterpret_cast<float*>(&accelerometer), reinterpret_cast<float*>(&gyroscope), compass);
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromIMUStamped(
    double time_offset_seconds,
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    carla::geom::Vector3D accelerometer,
    carla::geom::Vector3D gyroscope,
    float compass,
    void *actor) {
  // Per-substep stamp = current frame stamp minus the sample's offset from frame end.
  double base = static_cast<double>(_seconds) + 1e-9 * static_cast<double>(_nanoseconds);
  double t = base - time_offset_seconds;
  if (t < 0.0) t = 0.0;
  int32_t sec = static_cast<int32_t>(t);
  uint32_t nsec = static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);
  auto sensors = GetOrCreateSensor(ESensors::InertialMeasurementUnit, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaIMUPublisher> publisher = std::dynamic_pointer_cast<CarlaIMUPublisher>(sensors.first);
    publisher->SetData(sec, nsec, reinterpret_cast<float*>(&accelerometer), reinterpret_cast<float*>(&gyroscope), compass);
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(sec, nsec, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromDVS(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    const carla::SharedBufferView buffer,
    int W, int H, float Fov,
    void *actor) {
  log_info("Sensor DVS to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id);
  auto sensors = GetOrCreateSensor(ESensors::DVSCamera, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaDVSCameraPublisher> publisher = std::dynamic_pointer_cast<CarlaDVSCameraPublisher>(sensors.first);
    const carla::sensor::s11n::ImageSerializer::ImageHeader *header =
      reinterpret_cast<const carla::sensor::s11n::ImageSerializer::ImageHeader *>(buffer->data());
    if (!header)
      return;
    if (!publisher->HasBeenInitialized())
      publisher->InitInfoData(0, 0, H, W, Fov, true);
    size_t elements = (buffer->size() - carla::sensor::s11n::ImageSerializer::header_offset) / sizeof(carla::sensor::data::DVSEvent);
    publisher->SetImageData(_seconds, _nanoseconds, elements, header->height, header->width, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
    publisher->SetCameraInfoData(_seconds, _nanoseconds);
    publisher->SetPointCloudData(1, elements * sizeof(carla::sensor::data::DVSEvent), elements, (const uint8_t*) (buffer->data() + carla::sensor::s11n::ImageSerializer::header_offset));
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

// ue5-dev compatible overload is provided as inline in ROS2.h
void ROS2::ProcessDataFromLidar(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    uint32_t channel_count,
    float upper_fov_limit,
    float lower_fov_limit,
    carla::sensor::data::LidarData &data,
    void *actor) {
  log_info("Sensor Lidar to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "points.", data._points.size());
  auto sensors = GetOrCreateSensor(ESensors::RayCastLidar, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaLidarPublisher> publisher = std::dynamic_pointer_cast<CarlaLidarPublisher>(sensors.first);

    // Convert to radians
    upper_fov_limit = upper_fov_limit * M_PI / 180.0f;
    lower_fov_limit = lower_fov_limit * M_PI / 180.0f;

    std::vector<float> vertical_angles;
    const float vertical_step = (channel_count > 1U)
        ? (upper_fov_limit - lower_fov_limit) / static_cast<float>(channel_count - 1U)
        : 0.f;
    vertical_angles.reserve(channel_count);
    for (uint32_t i = 0; i < channel_count; ++i) {
      vertical_angles.push_back(upper_fov_limit - static_cast<float>(i) * vertical_step);
    }

    size_t width = data._points.size();
    size_t height = 1;
    publisher->SetDataEx(_seconds, _nanoseconds, height, width, (float*)data._points.data(),
      data._header.size() - carla::sensor::data::LidarData::Index::SIZE,
      data._header.data() + carla::sensor::data::LidarData::Index::SIZE,
      vertical_angles);
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromSemanticLidar(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::SemanticLidarData &data,
    void *actor) {
  static_assert(sizeof(float) == sizeof(uint32_t), "Invalid float size");
  log_info("Sensor SemanticLidar to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "points.", data._ser_points.size());
  auto sensors = GetOrCreateSensor(ESensors::RayCastSemanticLidar, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaSemanticLidarPublisher> publisher = std::dynamic_pointer_cast<CarlaSemanticLidarPublisher>(sensors.first);
    size_t width = data._ser_points.size();
    size_t height = 1;
    publisher->SetData(_seconds, _nanoseconds, 6, height, width, (float*)data._ser_points.data());
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromRadar(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    const carla::sensor::data::RadarData &data,
    void *actor) {
  log_info("Sensor Radar to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "points.", data._detections.size());
  auto sensors = GetOrCreateSensor(ESensors::Radar, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaRadarPublisher> publisher = std::dynamic_pointer_cast<CarlaRadarPublisher>(sensors.first);
    size_t elements = data.GetDetectionCount();
    size_t width = elements * sizeof(carla::sensor::data::RadarDetection);
    size_t height = 1;
    publisher->SetData(_seconds, _nanoseconds, height, width, elements, (const uint8_t*)data._detections.data());
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromObstacleDetection(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    AActor *first_ctor,
    AActor *second_actor,
    float distance,
    void *actor) {
  log_info("Sensor ObstacleDetector to ROS data: frame.", _frame, "sensor.", sensor_type, "stream.", stream_id, "distance.", distance);
}

void ROS2::ProcessDataFromCollisionSensor(
    uint64_t sensor_type,
    carla::streaming::detail::stream_id_type stream_id,
    const carla::geom::Transform sensor_transform,
    uint32_t other_actor,
    carla::geom::Vector3D impulse,
    void* actor) {
  auto sensors = GetOrCreateSensor(ESensors::CollisionSensor, stream_id, actor);
  if (sensors.first) {
    std::shared_ptr<CarlaCollisionPublisher> publisher = std::dynamic_pointer_cast<CarlaCollisionPublisher>(sensors.first);
    publisher->SetData(_seconds, _nanoseconds, other_actor, impulse.x, impulse.y, impulse.z);
    publisher->Publish();
  }
  if (sensors.second && _publish_tf) {
    std::shared_ptr<CarlaTransformPublisher> publisher = std::dynamic_pointer_cast<CarlaTransformPublisher>(sensors.second);
    publisher->SetData(_seconds, _nanoseconds, (const float*)&sensor_transform.location, (const float*)&sensor_transform.rotation);
    publisher->Publish();
  }
}

void ROS2::ProcessDataFromStatusSensor(
  uint64_t sensor_type,
  carla::streaming::detail::stream_id_type stream_id,
  const carla::geom::Transform sensor_transform,
  const sensor::s11n::VehicleStatusData &data,
  void *vehicle_actor,
  void *actor)
{
  // This callback is only for the Ego vehicle
  if (!_autoware_controller || vehicle_actor != _autoware_controller->GetVehicle()) {
    log_warning("There is no Ego vehicle or this Vehicle Status Sensor is attached to a different vehicle than Ego. This is not allowed - not publishing the data!");
    return;
  }

  if (!_autoware_publisher) {
    log_error("Encountered an unexpected error when publishing Vehicle Status Sensor data (", __FILE__, ":", __LINE__, ")");
    return;
  }

  constexpr uint8_t reverse_mask     = 0b00000001;
  constexpr uint8_t manual_gear_mask = 0b00000010;

  constexpr uint8_t left_blinker_mask  = 0b00000001;
  constexpr uint8_t right_blinker_mask = 0b00000010;
  constexpr uint8_t hazard_lights_mask = 0b00000100;

  // Decode data
  const bool is_reverse = data.control_flags & reverse_mask;
  const bool is_manual_gear = data.control_flags & manual_gear_mask;

  const bool is_left_blinker_on = data.turn_mask & left_blinker_mask;
  const bool is_right_blinker_on = data.turn_mask & right_blinker_mask;
  const bool is_hazard_lights_on = data.turn_mask & hazard_lights_mask;

  // TODO: Verify whether any of the fields should be inverted
  // Now all positive values mean forward and left (if going forward)
  _autoware_publisher->SetVelocity(data.vel_x_mps, -data.vel_y_mps, -data.angVel_z_mps);

  // TODO: Check if steering should be set reversed (it is set reversed because control had to be reversed (this is an educated guess))
  _autoware_publisher->SetSteering(-data.steer);

  // TODO: Add logic to use the input of control mode and base don that set output
  // NOTE: Control mode command is a service, so no easy way to get it as of now (27.08.2025)
  _autoware_publisher->SetControlMode(ControlMode::AUTONOMOUS);

  // TODO: Verify what is the incoming gear from data.GetGear() and whether is_reverse should be used here
  _autoware_publisher->SetGear(Gear::NONE);
  switch (data.gear) {
    #define CASE(GEAR_VALUE, GEAR_ENUM)          \
      case GEAR_VALUE:                           \
        _autoware_publisher->SetGear(GEAR_ENUM); \
        break;                                   \
        static_assert(true, "")

    CASE(-2,  Gear::REVERSE_2);
    CASE(-1,  Gear::REVERSE  );
    CASE( 0,  Gear::NEUTRAL  );
    CASE( 1,  Gear::DRIVE    );
    CASE( 2,  Gear::DRIVE_2  );
    CASE( 3,  Gear::DRIVE_3  );
    CASE( 4,  Gear::DRIVE_4  );
    CASE( 5,  Gear::DRIVE_5  );
    CASE( 6,  Gear::DRIVE_6  );
    CASE( 7,  Gear::DRIVE_7  );
    CASE( 8,  Gear::DRIVE_8  );
    CASE( 9,  Gear::DRIVE_9  );
    CASE( 10, Gear::DRIVE_10 );
    CASE( 11, Gear::DRIVE_11 );
    CASE( 12, Gear::DRIVE_12 );
    CASE( 13, Gear::DRIVE_13 );
    CASE( 14, Gear::DRIVE_14 );
    CASE( 15, Gear::DRIVE_15 );
    CASE( 16, Gear::DRIVE_16 );
    CASE( 17, Gear::DRIVE_17 );
    CASE( 18, Gear::DRIVE_18 );

    #undef CASE
  }

  // Turn indicators
  //
  // CARLA does not actuate the ego's blinker from Autoware's turn_indicators_cmd
  // (AutowareController only reads the command), so turn_mask stays OFF and the
  // status would be permanently DISABLE. The Autoware planner consumes
  // turn_indicators_status as an input, so a stuck-DISABLE status starves it of
  // turn / lane-change context (the vehicle then fails to turn into the correct
  // lane and never commits lane changes). Echo the command back as status while
  // Autoware is driving, and fall back to the vehicle light state only before any
  // command has been received.
  bool turn_indicators_set_from_command = false;
  if (_autoware_controller) {
    switch (_autoware_controller->GetTurnIndicatorCommand()) {
      case 2:  // TurnIndicatorsCommand::ENABLE_LEFT
        _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::LEFT);
        turn_indicators_set_from_command = true;
        break;
      case 3:  // TurnIndicatorsCommand::ENABLE_RIGHT
        _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::RIGHT);
        turn_indicators_set_from_command = true;
        break;
      case 1:  // TurnIndicatorsCommand::DISABLE
        _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::OFF);
        turn_indicators_set_from_command = true;
        break;
      default: // 0 == NO_COMMAND / nothing received yet -> fall back below
        break;
    }
  }
  if (!turn_indicators_set_from_command) {
    if (!is_left_blinker_on && !is_right_blinker_on) {
      _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::OFF);
    } else if (is_left_blinker_on && !is_right_blinker_on) {
      _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::LEFT);
    } else if (is_right_blinker_on && !is_left_blinker_on) {
      _autoware_publisher->SetTurnIndicators(TurnIndicatorsStatus::RIGHT);
    } else {
      log_error("Both left and right blinkers are on. This should not happen!");
    }
  }

  // Hazard lights: same rationale as turn indicators -- echo the Autoware command
  // back as status; fall back to the vehicle light state before any command.
  bool hazard_on = is_hazard_lights_on;
  if (_autoware_controller) {
    switch (_autoware_controller->GetHazardLightsCommand()) {
      case 2:  // HazardLightsCommand::ENABLE
        hazard_on = true;
        break;
      case 1:  // HazardLightsCommand::DISABLE
        hazard_on = false;
        break;
      default: // 0 == NO_COMMAND -> keep light-state fallback
        break;
    }
  }
  _autoware_publisher->SetHazardLights(hazard_on);

  _autoware_publisher->Publish(_seconds, _nanoseconds);

  // Debug
  if constexpr (false) {
    std::cerr << "========== NEW STATUS ==========" << '\n'
              << "    RAW DATA:" << '\n'
              << "Timestamp: "     << data.timestamp << '\n'
              << "Speed: "         << data.speed_mps << '\n'
              << "VelX: "          << data.vel_x_mps << '\n'
              << "VelY: "          << data.vel_y_mps << '\n'
              << "VelZ: "          << data.vel_z_mps << '\n'
              << "AngVelX: "       << data.angVel_x_mps << '\n'
              << "AngVelY: "       << data.angVel_y_mps << '\n'
              << "AngVelZ: "       << data.angVel_z_mps << '\n'
              << "RotrPitch: "     << data.rot_pitch << '\n'
              << "RotrYaw: "       << data.rot_yaw << '\n'
              << "RotrRoll: "      << data.rot_roll << '\n'
              << "Steering: "      << data.steer << '\n'
              << "Gear: "          << data.gear << '\n'
              << "Turn mask: "     << data.turn_mask << '\n'
              << "Control flags: " << data.control_flags << '\n'
              << "    PROCESSED:" << '\n'
              << "Is reverse: "          << is_reverse << '\n'
              << "Is manual gear: "      << is_manual_gear << '\n'
              << "Is left blinker on: "  << is_left_blinker_on << '\n'
              << "Is right blinker on: " << is_right_blinker_on << '\n'
              << "Is hazard lights on: " << is_hazard_lights_on << '\n'
              << std::flush;
  }
}

void ROS2::Shutdown() {
  for (auto& element : _publishers) {
    element.second.reset();
  }
  for (auto& element : _transforms) {
    element.second.reset();
  }
  _clock_publisher.reset();
  _controller.reset();
  _autoware_controller.reset();
  _autoware_publisher.reset();
  _enabled = false;
#if defined(WITH_ROS2_DEMO)
  _basic_publisher.reset();
  _basic_subscriber.reset();
#endif
}

} // namespace ros2
} // namespace carla
