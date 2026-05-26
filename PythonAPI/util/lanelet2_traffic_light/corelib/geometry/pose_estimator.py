"""traffic_light way の2点 + yaw_offset から信号機の中心位置(local 座標 m)と
yaw(度)を推定するモジュール。Z は別途 mgrs_transform で height_m から算出。

Phase 0 で確定した CARLA-Odaiba 変換:
- lanelet2 の Y 軸 (Northing) は Unreal Y 軸とは符号が反転している
  (mgrs_transform の y_sign=-1 と同じ理由)
- そのため Yaw 計算でも dy の符号を反転して `atan2(-dy, dx)` とする
"""
import math
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import TrafficLightSpec


def estimate_center_and_yaw(spec: TrafficLightSpec, yaw_offset_deg: float = -90.0) -> tuple[float, float, float]:
    """way の2点から中点と Unreal world での Yaw を計算。

    Returns:
        (center_x_m, center_y_m, yaw_deg)
        - center は lanelet2 の local_x/local_y 単位 (m)。Unreal 変換は呼び出し側。
        - yaw は Unreal world の Z 軸まわり (度)。
          内部式: math.degrees(atan2(-dy, dx)) + yaw_offset_deg
          - lanelet2 Y 反転を考慮 (-dy)
          - yaw_offset_deg は信号面の正面方向の補正
    """
    cx = (spec.p0.local_x + spec.p1.local_x) / 2.0
    cy = (spec.p0.local_y + spec.p1.local_y) / 2.0
    dx = spec.p1.local_x - spec.p0.local_x
    dy = spec.p1.local_y - spec.p0.local_y
    # lanelet2 -> Unreal で Y 反転されるので、yaw 計算も -dy を使う
    way_yaw_deg = math.degrees(math.atan2(-dy, dx))
    return (cx, cy, way_yaw_deg + yaw_offset_deg)
