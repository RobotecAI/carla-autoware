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
  ///
  /// Layout (FROZEN: never renumber -- baked map content stamps these bits.
  /// Extend via optional kwargs or a new separately-named mask):
  ///   direction index: 0=Left 1=Straight 2=Right 3=UpLeft 4=UpRight
  ///                    5=Down (reserved, lane-use control) 6=DownLeft
  ///                    7=DownRight
  ///   bits  0-7  green row  (the only row implemented visually)
  ///   bits  8-15 yellow row (tram; reserved)
  ///   bits 16-23 red row    (reserved)
  ///   bits 24-30 user-defined sections (special arrows, e.g. U-turn)
  ///   bit  31    unused (int32 sign bit on the UE side)
  class TrafficLightArrowState {
  public:

    using flag_type = uint32_t;

    /// OR-composable bitmask: combine sections with |, test with &.
    enum class ArrowState : flag_type {
      None            = 0,
      GreenLeft       = 0x1,
      GreenStraight   = 0x1 << 1,
      GreenRight      = 0x1 << 2,
      GreenUpLeft     = 0x1 << 3,
      GreenUpRight    = 0x1 << 4,
      GreenDown       = 0x1 << 5,
      GreenDownLeft   = 0x1 << 6,
      GreenDownRight  = 0x1 << 7,
      YellowLeft      = 0x1 << 8,
      YellowStraight  = 0x1 << 9,
      YellowRight     = 0x1 << 10,
      YellowUpLeft    = 0x1 << 11,
      YellowUpRight   = 0x1 << 12,
      YellowDown      = 0x1 << 13,
      YellowDownLeft  = 0x1 << 14,
      YellowDownRight = 0x1 << 15,
      RedLeft         = 0x1 << 16,
      RedStraight     = 0x1 << 17,
      RedRight        = 0x1 << 18,
      RedUpLeft       = 0x1 << 19,
      RedUpRight      = 0x1 << 20,
      RedDown         = 0x1 << 21,
      RedDownLeft     = 0x1 << 22,
      RedDownRight    = 0x1 << 23,
      User1           = 0x1 << 24,
      User2           = 0x1 << 25,
      User3           = 0x1 << 26,
      User4           = 0x1 << 27,
      User5           = 0x1 << 28,
      User6           = 0x1 << 29,
      User7           = 0x1 << 30,
    };
  };

} // namespace rpc
} // namespace carla
