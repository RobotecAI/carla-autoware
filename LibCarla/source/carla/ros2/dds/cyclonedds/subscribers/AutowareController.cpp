// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowareController.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <mutex>

#include "dds/dds.h"
#include "Control.h"
#include "GearCommand.h"
#include "TurnIndicatorsCommand.h"
#include "HazardLightsCommand.h"
#include "VehicleEmergencyStamped.h"
#include "Engage.h"

#include "carla/ros2/AutowareSteeringCompensation.h"

namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Internal subscriber: one per Autoware command topic
// ---------------------------------------------------------------------------
template <typename Message>
struct AutowareSubReader {
  dds_entity_t participant { 0 };
  dds_entity_t topic { 0 };
  dds_entity_t reader { 0 };

  std::mutex mutex;
  Message data {};
  bool data_changed { false };

  bool HasNewMessage() {
    std::lock_guard<std::mutex> lock(mutex);
    return data_changed;
  }

  Message GetMessage() {
    std::lock_guard<std::mutex> lock(mutex);
    data_changed = false;
    return data;
  }

  // Read the last received message without clearing the "changed" flag, so a
  // separate consumer (e.g. status echo) can sample it independently of the
  // control loop's HasNewMessage()/GetMessage() handshake.
  Message PeekMessage() {
    std::lock_guard<std::mutex> lock(mutex);
    return data;
  }

  void OnData(const Message& msg) {
    std::lock_guard<std::mutex> lock(mutex);
    data = msg;
    data_changed = true;
  }

  ~AutowareSubReader() {
    if (participant > 0) dds_delete(participant);
  }
};

// ---------------------------------------------------------------------------
// Callback trampolines - CycloneDDS listener callbacks
// ---------------------------------------------------------------------------
template <typename Message>
static void on_data_available(dds_entity_t reader_entity, void* arg) {
  auto* sub = static_cast<AutowareSubReader<Message>*>(arg);
  if (!sub) return;

  Message msg {};
  void* samples[1] = { &msg };
  dds_sample_info_t infos[1];

  int32_t n = dds_take(reader_entity, samples, infos, 1, 1);
  if (n > 0 && infos[0].valid_data) {
    sub->OnData(msg);
  }
}

// ---------------------------------------------------------------------------
// Helper: create subscriber with CycloneDDS C API
// ---------------------------------------------------------------------------
template <typename Message>
static bool InitSubReader(AutowareSubReader<Message>& sub, DomainId domain_id,
                          const dds_topic_descriptor_t* desc, const char* topic_name,
                          const char* human_name) {
  sub.participant = dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr);
  if (sub.participant < 0) {
    std::cerr << "CycloneDDS: Failed to create DomainParticipant for " << human_name << std::endl;
    return false;
  }

  sub.topic = dds_create_topic(sub.participant, desc, topic_name, nullptr, nullptr);
  if (sub.topic < 0) {
    std::cerr << "CycloneDDS: Failed to create Topic for " << human_name << ": " << dds_strretcode(-sub.topic) << std::endl;
    dds_delete(sub.participant);
    sub.participant = 0;
    return false;
  }

  // RELIABLE + TRANSIENT_LOCAL QoS (matches Autoware default for commands)
  dds_qos_t* qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

  // Create listener
  dds_listener_t* listener = dds_create_listener(&sub);
  dds_lset_data_available(listener, on_data_available<Message>);

  sub.reader = dds_create_reader(sub.participant, sub.topic, qos, listener);
  dds_delete_listener(listener);
  dds_delete_qos(qos);

  if (sub.reader < 0) {
    std::cerr << "CycloneDDS: Failed to create DataReader for " << human_name << std::endl;
    dds_delete(sub.participant);
    sub.participant = 0;
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------
class AutowareController::Implementation {
public:
  AutowareSubReader<autoware_control_msgs_msg_Control>               _control_subscriber;
  AutowareSubReader<autoware_vehicle_msgs_msg_GearCommand>           _gear_subscriber;
  AutowareSubReader<autoware_vehicle_msgs_msg_TurnIndicatorsCommand> _turn_indicator_subscriber;
  AutowareSubReader<autoware_vehicle_msgs_msg_HazardLightsCommand>   _hazard_lights_subscriber;
  AutowareSubReader<tier4_vehicle_msgs_msg_VehicleEmergencyStamped>  _emergency_subscriber;
  AutowareSubReader<autoware_vehicle_msgs_msg_Engage>                _engage_subscriber;

  void* _vehicle { nullptr };
};

AutowareController::AutowareController(void* actor, const DomainId domain_id)
: _impl(std::make_shared<Implementation>())
{
  _impl->_vehicle = actor;

  InitSubReader(_impl->_control_subscriber, domain_id,
    &autoware_control_msgs_msg_Control_desc,
    "rt/control/command/control_cmd", "Autoware vehicle control");

  InitSubReader(_impl->_gear_subscriber, domain_id,
    &autoware_vehicle_msgs_msg_GearCommand_desc,
    "rt/control/command/gear_cmd", "Autoware gear command");

  InitSubReader(_impl->_turn_indicator_subscriber, domain_id,
    &autoware_vehicle_msgs_msg_TurnIndicatorsCommand_desc,
    "rt/control/command/turn_indicators_cmd", "Autoware turn indicator command");

  InitSubReader(_impl->_hazard_lights_subscriber, domain_id,
    &autoware_vehicle_msgs_msg_HazardLightsCommand_desc,
    "rt/control/command/hazard_lights_cmd", "Autoware hazard lights command");

  InitSubReader(_impl->_emergency_subscriber, domain_id,
    &tier4_vehicle_msgs_msg_VehicleEmergencyStamped_desc,
    "rt/control/command/emergency_cmd", "Autoware emergency");

  InitSubReader(_impl->_engage_subscriber, domain_id,
    &autoware_vehicle_msgs_msg_Engage_desc,
    "rt/vehicle/engage", "Autoware engage");
}

AutowareController::~AutowareController() = default;

bool AutowareController::HasNewControl() const {
  // Debug
  if constexpr (false) {
#define PRINT(SUBSCRIBER)                                         \
  if (_impl->SUBSCRIBER.HasNewMessage())                          \
    std::cerr << #SUBSCRIBER << " Has new message!" << std::endl; \
    static_assert(true, "")

    PRINT(_control_subscriber);
    PRINT(_gear_subscriber);
    PRINT(_turn_indicator_subscriber);
    PRINT(_hazard_lights_subscriber);
    PRINT(_emergency_subscriber);
    PRINT(_engage_subscriber);

#undef PRINT
  }

  return _impl->_control_subscriber.HasNewMessage() ||
         _impl->_gear_subscriber.HasNewMessage() ||
         _impl->_turn_indicator_subscriber.HasNewMessage() ||
         _impl->_hazard_lights_subscriber.HasNewMessage() ||
         _impl->_emergency_subscriber.HasNewMessage() ||
         _impl->_engage_subscriber.HasNewMessage();
}

VehicleAccelerationControl AutowareController::GetControl() {
  VehicleAccelerationControl control_out{};

  const auto control_in = _impl->_control_subscriber.GetMessage();
  const auto gear_in = _impl->_gear_subscriber.GetMessage();
  const auto turn_indicator_in = _impl->_turn_indicator_subscriber.GetMessage();
  const auto hazard_lights_in = _impl->_hazard_lights_subscriber.GetMessage();
  const auto emergency_in = _impl->_emergency_subscriber.GetMessage();
  const auto engage_in = _impl->_engage_subscriber.GetMessage();

  // Debug
  if constexpr (false) {
    std::cerr << "=== NEW CONTROL ===" << std::endl;

    std::cerr << "Control"
      << " " << control_in.longitudinal.velocity
      << " " << control_in.longitudinal.acceleration
      << " " << control_in.longitudinal.jerk
      << " " << (control_in.longitudinal.is_defined_acceleration ? "True" : "False")
      << " " << (control_in.longitudinal.is_defined_jerk ? "True" : "False")
      << " " << control_in.lateral.steering_tire_angle
      << " " << control_in.lateral.steering_tire_rotation_rate
      << " " << (control_in.lateral.is_defined_steering_tire_rotation_rate ? "True" : "False")
      << std::endl;

    std::cerr << "Gear"
      << " " << static_cast<int>(gear_in.command)
      << std::endl;

    std::cerr << "Turn indicator"
      << " " << static_cast<int>(turn_indicator_in.command)
      << std::endl;

    std::cerr << "Hazard lights"
      << " " << static_cast<int>(hazard_lights_in.command)
      << std::endl;

    std::cerr << "Emergency"
      << " " << (emergency_in.emergency ? "True" : "False")
      << std::endl;

    std::cerr << "Engage"
      << " " << (engage_in.engage ? "True" : "False")
      << std::endl;
  }

  // Longitudinal: use ONLY acceleration from /control/command/control_cmd to drive the vehicle.
  // Velocity and jerk from the message are intentionally ignored to keep pure acceleration control.
  control_out.acceleration = control_in.longitudinal.acceleration;

  /// @note Set lateral negative, because Carla treats positive as right and Autoware expects positive to represent left (all when moving forward)
  const auto raw_steering = -control_in.lateral.steering_tire_angle;

  // Apply steering compensation lookup table (measured on Lincoln MKZ).
  control_out.steer = autoware_steering_compensation::GetSteeringInput(raw_steering);
  control_out.steer_speed = 0.0f;
  if (control_in.lateral.is_defined_steering_tire_rotation_rate) {
    control_out.steer_speed = -control_in.lateral.steering_tire_rotation_rate;
  }

  // TODO: Use input from all subscribers to perform actions in simulation

  return control_out;
}

void* AutowareController::GetVehicle() {
  return _impl->_vehicle;
}

uint8_t AutowareController::GetTurnIndicatorCommand() const {
  return _impl->_turn_indicator_subscriber.PeekMessage().command;
}

uint8_t AutowareController::GetHazardLightsCommand() const {
  return _impl->_hazard_lights_subscriber.PeekMessage().command;
}

}  // namespace ros2
}  // namespace carla
