"""Module for converting MGRS local_x/local_y (m) to Unreal world coordinates (cm).

Implementation confirmed in Phase 0 validation:
- Unit of MgrsDataAsset.MgrsOffsetPosition is m (not cm)
- Standard CARLA Z-up convention; only Y is inverted, X/Z share the same sign
- Unreal Z height of a traffic light is derived directly from pole_height_m (offset.Z is 0 by default)

Coordinate conversion formulas:
    Unreal_X_cm = x_sign * (local_x_m - offset_x_m) * 100
    Unreal_Y_cm = y_sign * (local_y_m - offset_y_m) * 100
    Unreal_Z_cm = (pole_height_m - offset_z_m) * 100
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class MgrsTransformer:
    """Convert MGRS local coordinates (m) to Unreal world coordinates (cm).

    Args:
        offset_x_m: MgrsOffsetPosition.X from WorldSettings (m).
        offset_y_m: Same for Y (m).
        offset_z_m: Same for Z (m). 0 for Odaiba.
        x_sign: Sign for the lanelet2->Unreal X axis (Odaiba: +1).
        y_sign: Sign for the lanelet2->Unreal Y axis (Odaiba: -1, north->south inversion).
    """
    offset_x_m: float
    offset_y_m: float
    offset_z_m: float
    x_sign: int = +1
    y_sign: int = -1   # lanelet2 ENU -> Unreal world Y inversion

    def local_to_unreal_cm(self, local_x_m: float, local_y_m: float, height_m: float) -> tuple[float, float, float]:
        x = self.x_sign * (local_x_m - self.offset_x_m) * 100.0
        y = self.y_sign * (local_y_m - self.offset_y_m) * 100.0
        z = (height_m - self.offset_z_m) * 100.0
        return (x, y, z)

    def unreal_cm_to_local(self, x_cm: float, y_cm: float, z_cm: float) -> tuple[float, float, float]:
        """Inverse of local_to_unreal_cm: Unreal cm -> lanelet2 local (m).

        Args:
            x_cm: Unreal world X coordinate in cm.
            y_cm: Unreal world Y coordinate in cm.
            z_cm: Unreal world Z coordinate (height) in cm.

        Returns:
            Tuple of (local_x_m, local_y_m, height_m) in lanelet2 local coordinates.
        """
        local_x_m = x_cm / (self.x_sign * 100.0) + self.offset_x_m
        local_y_m = y_cm / (self.y_sign * 100.0) + self.offset_y_m
        height_m = z_cm / 100.0 + self.offset_z_m
        return (local_x_m, local_y_m, height_m)
