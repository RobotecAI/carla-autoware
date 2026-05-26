"""snap モードで使う既存メッシュ Z 座標の分布統計データ構造と計算。

unreal モジュールに依存しない pure Python。pytest からテスト可能。

Phase 4.2 追加: マップ非依存に Z 範囲判定するため、Full Run 開始時に
既存メッシュの World Z を集めて統計化、Tukey の 1.5 IQR 規則で外れ値を
除外する範囲 (z_low, z_high) を導出する。

これにより:
- 別マップで Z 基準が違っても (海抜/楕円体ズレ等) 自動適応
- subtype 別 pole_height の固定値設定不要 (profile 値はフォールバックとして残す)
"""
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MeshZStats:
    """既存メッシュの World Z 座標分布の統計。

    `tukey_low` / `tukey_high` は外れ値検出の標準手法 (Tukey の 1.5 IQR 規則)
    で算出された範囲。この外側に位置する mesh は「立体駐車場の上の信号機」
    のような外れ値とみなして snap target から除外する。
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
    """Z 値の iterable から MeshZStats を計算する。空なら None を返す。

    四分位数は素朴な index 法 (n//4, n//2, 3n//4)。サンプル数が十分多ければ
    補間版との差は無視できる。
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
