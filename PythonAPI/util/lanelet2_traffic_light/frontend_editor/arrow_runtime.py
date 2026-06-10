"""(color x direction) arrow bitmask helpers shared by the placer (pure).

Bit layout mirrors carla/rpc/TrafficLightArrowState.h: the green row occupies
bits 0..7 (direction index 0=left 1=straight 2=right 3=up_left 4=up_right
5=down 6=down_left 7=down_right). "down" (bit 5) is a reserved slot for
lane-use control devices and is intentionally absent from the placer table.
Yellow/red rows (bits 8-15 / 16-23) are reserved; the placer only ever stamps
green bits (the only faces present in the maps).
"""

ARROW_FLAG_BY_DIR = {
    "left": 0x1,
    "straight": 0x2,
    "right": 0x4,
    "up_left": 0x8,
    "up_right": 0x10,
    "down_left": 0x40,
    "down_right": 0x80,
}


def initial_arrow_mask(green_dirs):
    """Bitmask for the lanelet2 green arrows (the constant-on initial state)."""
    return sum(f for d, f in ARROW_FLAG_BY_DIR.items() if d in green_dirs)


def capabilities_mask(slot_index_by_dir):
    """Bitmask of physically present arrow faces (material slot index >= 0)."""
    return sum(f for d, f in ARROW_FLAG_BY_DIR.items()
               if slot_index_by_dir.get(d, -1) >= 0)
