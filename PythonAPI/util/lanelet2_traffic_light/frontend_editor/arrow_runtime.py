"""(color x direction) arrow bitmask helpers shared by the placer (pure).

Bit layout mirrors carla/rpc/TrafficLightArrowState.h: the green row occupies
bits 0..2 (left/straight/right). Yellow/red rows are reserved there; the
placer only ever stamps green bits in v1 (the only faces present in the maps).
"""

ARROW_FLAG_BY_DIR = {
    "left": 0x1,
    "straight": 0x2,
    "right": 0x4,
}


def initial_arrow_mask(green_dirs):
    """Bitmask for the lanelet2 green arrows (the constant-on initial state)."""
    return sum(f for d, f in ARROW_FLAG_BY_DIR.items() if d in green_dirs)


def capabilities_mask(slot_index_by_dir):
    """Bitmask of physically present arrow faces (material slot index >= 0)."""
    return sum(f for d, f in ARROW_FLAG_BY_DIR.items()
               if slot_index_by_dir.get(d, -1) >= 0)
