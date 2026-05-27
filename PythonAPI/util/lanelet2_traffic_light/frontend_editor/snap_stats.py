"""Data structures and computation for the Z-coordinate distribution statistics of existing meshes used in snap mode.

No dependency on the unreal module (pure Python). Testable from pytest.

Added in Phase 4.2: at the start of a Full Run, World Z values of existing meshes are
collected and summarized to derive an outlier-free range (z_low, z_high) using
Tukey's 1.5 IQR rule, enabling map-independent Z range detection.

Benefits:
- Auto-adapts even when the Z reference differs between maps (e.g. sea level / ellipsoid offset)
- Eliminates the need for per-subtype fixed pole_height values (profile values remain as fallback)
"""
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MeshZStats:
    """Statistics of the World Z coordinate distribution for existing meshes.

    `tukey_low` / `tukey_high` is the range computed by the standard outlier
    detection method (Tukey's 1.5 IQR rule). Meshes outside this range are
    treated as outliers (e.g. traffic lights on top of a multi-story car park)
    and excluded from snap targets.
    """
    n: int
    z_min: float
    z_max: float
    z_median: float
    z_q25: float
    z_q75: float

    @property
    def iqr(self) -> float:
        return self.z_q75 - self.z_q25

    @property
    def tukey_low(self) -> float:
        return self.z_q25 - 1.5 * self.iqr

    @property
    def tukey_high(self) -> float:
        return self.z_q75 + 1.5 * self.iqr


def compute_z_stats(zs) -> Optional[MeshZStats]:
    """Compute MeshZStats from an iterable of Z values. Returns None if the iterable is empty.

    Quartiles are computed with a simple index method (n//4, n//2, 3n//4).
    The difference from the interpolated version is negligible for sufficiently large samples.
    """
    zs_sorted = sorted(zs)
    n = len(zs_sorted)
    if n == 0:
        return None
    return MeshZStats(
        n=n,
        z_min=zs_sorted[0],
        z_max=zs_sorted[-1],
        z_median=zs_sorted[n // 2],
        z_q25=zs_sorted[n // 4],
        z_q75=zs_sorted[(3 * n) // 4],
    )
