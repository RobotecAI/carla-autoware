# carla_v2i -- V2I traffic signal publisher for CARLA (AWSIM port)

Publishes the states of lanelet2-placed CARLA traffic lights as
`autoware_perception_msgs/TrafficLightGroupArray` (default topic
`/v2x/traffic_signals`, 10 Hz, QoS Reliable/Volatile/KeepLast(1)), so that
Autoware can obey signals without camera-based recognition via
`autoware_traffic_light_arbiter` (`external_traffic_signals` input).

## Usage

    source <autoware_ws>/install/setup.bash   # rclpy + autoware_perception_msgs
    cd <repo>/PythonAPI/util
    python3 -m carla_v2i --osm-path /path/to/lanelet2_map.osm [--radius-m 0]

Group ids are lanelet2 regulatory-element relation ids by default
(`--id-mode way` switches to way ids, matching AWSIM's TrafficSignalId).

## DO NOT run together with scenario_simulator_v2 V2I traffic lights

ss2's `V2ITrafficLights` publishes the same topics itself. Running both
produces conflicting publishers. This tool is for CARLA-standalone use
(CARLA autonomous cycling or PythonAPI control + Autoware direct).

## Notes

- Read-only towards CARLA (never modifies world or actor state).
- Lights without a regulatory-element relation are not published
  (AWSIM parity).
