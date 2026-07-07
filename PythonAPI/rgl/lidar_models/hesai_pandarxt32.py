"""LiDAR preset: HesaiPandarXT32 (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "HesaiPandarXT32",
    "channels": 32,
    "range": 80.0,
    "min_range": 0.05,
    "upper_fov": 16.0,
    "lower_fov": -15.0,
    "horizontal_fov": 360.0,
    "rotation_frequency": 10.0,
    "points_per_second": 640000,
    "vertical_angles": [
        15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0, 8.0,
        7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0,
        -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0,
        -9.0, -10.0, -11.0, -12.0, -13.0, -14.0, -15.0, -16.0,
    ],
    "horizontal_angle_offsets": [],
    "ring_ids": [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
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
