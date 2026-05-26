"""snap_stats.MeshZStats / compute_z_stats のテスト。pure Python なので unreal 不要。"""
import pytest

from lanelet2_traffic_light.frontend_editor.snap_stats import (
    MeshZStats, compute_z_stats,
)


def test_empty_returns_none():
    assert compute_z_stats([]) is None


def test_single_value():
    s = compute_z_stats([100.0])
    assert s is not None
    assert s.n == 1
    assert s.z_min == 100.0
    assert s.z_max == 100.0
    assert s.z_median == 100.0
    assert s.z_q25 == 100.0
    assert s.z_q75 == 100.0


def test_simple_sequence():
    s = compute_z_stats([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
    assert s.n == 12
    assert s.z_min == 1
    assert s.z_max == 12
    # index 法: n//2=6 → zs[6]=7, n//4=3 → zs[3]=4, 3n//4=9 → zs[9]=10
    assert s.z_median == 7
    assert s.z_q25 == 4
    assert s.z_q75 == 10


def test_unsorted_input():
    s = compute_z_stats([5, 1, 3, 2, 4])
    assert s.z_min == 1
    assert s.z_max == 5


def test_iqr_and_tukey():
    s = MeshZStats(n=10, z_min=0, z_max=100, z_median=50, z_q25=20, z_q75=80)
    assert s.iqr == 60
    # tukey_low = 20 - 1.5*60 = -70, tukey_high = 80 + 1.5*60 = 170
    assert s.tukey_low == -70
    assert s.tukey_high == 170


def test_tukey_excludes_outliers():
    """Tukey 範囲外のサンプルを除外できることを確認する用法サンプル。"""
    # 内側: 100-200、外側: 1 件 1000 (外れ値)
    zs = [100, 110, 120, 130, 150, 170, 180, 190, 200, 1000]
    s = compute_z_stats(zs)
    # 1000 は tukey_high より上にあるはず
    assert 1000 > s.tukey_high
