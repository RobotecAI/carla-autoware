"""LiDAR preset: HesaiPandar40P (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "HesaiPandar40P",
    "channels": 40,
    "range": 200.0,
    "min_range": 0.3,
    "upper_fov": 25.0,
    "lower_fov": -15.0,
    "horizontal_fov": 360.0,
    "rotation_frequency": 10.0,
    "points_per_second": 720000,
    "vertical_angles": [
        -15.0, -11.0, -8.0, -5.0, -3.0, -2.0, -1.67, -1.33,
        -1.0, -0.67, -0.33, -0.0, 0.33, 0.67, 1.0, 1.33,
        1.67, 2.0, 2.33, 2.67, 3.0, 3.33, 3.67, 4.0,
        4.33, 4.67, 5.0, 5.33, 5.67, 6.0, 7.0, 8.0,
        9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 19.0, 25.0,
    ],
    "horizontal_angle_offsets": [
        -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, 3.125, -5.208,
        -1.042, 3.125, -5.208, -1.042, 3.125, -5.208, -1.042, 3.125,
        -5.208, -1.042, 3.125, -5.208, -1.042, 3.125, -5.208, -1.042,
        3.125, -5.208, -1.042, 3.125, -5.208, -1.042, -1.042, -1.042,
        -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042,
    ],
    "ring_ids": [
        39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24,
        23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8,
        7, 6, 5, 4, 3, 2, 1, 0,
    ],
}

NOISE = {
    "angular_type": "ray",
    "angular_mean": 0.0,
    "angular_stddev": 0.05730,    # degrees (= 0.001 rad)
    "distance_mean": 0.0,
    "distance_stddev_base": 0.02,
    "distance_stddev_rise": 0.0,
    "angular_axis": "Y",
}

BEAM_DIVERGENCE = {
    "horizontal": 0.13,   # degrees (AWSIM default)
    "vertical": 0.13,     # degrees (AWSIM default)
}
