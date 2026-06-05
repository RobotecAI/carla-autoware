// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>

namespace carla {
namespace rpc {

  /// Bitmask of traffic-light arrow sections, (color x direction).
  /// v1 implements the green row visually; yellow (tram) and red rows are
  /// reserved so adding them later keeps the wire format unchanged.
  class TrafficLightArrowState {
  public:

    using flag_type = uint32_t;

    /// Can be used as flags.
    enum class ArrowState : flag_type {
      None           = 0,
      GreenLeft      = 0x1,
      GreenStraight  = 0x1 << 1,
      GreenRight     = 0x1 << 2,
      YellowLeft     = 0x1 << 3,
      YellowStraight = 0x1 << 4,
      YellowRight    = 0x1 << 5,
      RedLeft        = 0x1 << 6,
      RedStraight    = 0x1 << 7,
      RedRight       = 0x1 << 8,
    };
  };

} // namespace rpc
} // namespace carla
