"""MGRS local_x/local_y (m) を Unreal world 座標 (cm) に変換するモジュール。

Phase 0 検証で確定した実装：
- MgrsDataAsset.MgrsOffsetPosition の単位は m (cm ではない)
- 標準 CARLA Z-up convention、Y のみ反転、X/Z は同符号
- 信号機の Unreal Z 高さは pole_height_m から直接算出 (offset.Z は 0 が標準)

座標変換式:
    Unreal_X_cm = x_sign * (local_x_m - offset_x_m) * 100
    Unreal_Y_cm = y_sign * (local_y_m - offset_y_m) * 100
    Unreal_Z_cm = (pole_height_m - offset_z_m) * 100
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class MgrsTransformer:
    """MGRS 局所座標 (m) を Unreal world 座標 (cm) に変換。

    Args:
        offset_x_m: WorldSettings の MgrsOffsetPosition.X (m)。
        offset_y_m: 同 Y (m)。
        offset_z_m: 同 Z (m)。Odaiba では 0。
        x_sign: lanelet2→Unreal X の符号 (Odaiba: +1)。
        y_sign: lanelet2→Unreal Y の符号 (Odaiba: -1、北→南反転)。
    """
    offset_x_m: float
    offset_y_m: float
    offset_z_m: float
    x_sign: int = +1
    y_sign: int = -1   # lanelet2 ENU → Unreal world Y 反転

    def local_to_unreal_cm(self, local_x_m: float, local_y_m: float, height_m: float) -> tuple[float, float, float]:
        x = self.x_sign * (local_x_m - self.offset_x_m) * 100.0
        y = self.y_sign * (local_y_m - self.offset_y_m) * 100.0
        z = (height_m - self.offset_z_m) * 100.0
        return (x, y, z)
