"""LiDAR preset: SickMRS6000 (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "SickMRS6000",
    "channels": 24,
    "range": 40.0,
    "min_range": 0.0,
    "upper_fov": 0.8125,
    "lower_fov": -15.0,
    "horizontal_fov": 240.0,
    "rotation_frequency": 10.0,
    "points_per_second": 221760,
    "vertical_angles": [
        -15.0, -14.3125, -13.625, -12.9375, -12.25, -11.5625, -10.875, -10.1875,
        -9.5, -8.8125, -8.125, -7.4375, -6.75, -6.0625, -5.375, -4.6875,
        -4.0, -3.3125, -2.625, -1.9375, -1.25, -0.5625, 0.125, 0.8125,
    ],
    "horizontal_angle_offsets": [],
    "ring_ids": [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23,
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
