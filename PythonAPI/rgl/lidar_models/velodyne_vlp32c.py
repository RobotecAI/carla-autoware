"""LiDAR preset: VelodyneVLP32C (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "VelodyneVLP32C",
    "channels": 32,
    "range": 200.0,
    "min_range": 0.0,
    "upper_fov": 25.0,
    "lower_fov": -15.0,
    "horizontal_fov": 360.0,
    "rotation_frequency": 10.0,
    "points_per_second": 576000,
    "vertical_angles": [
        25.0, 1.0, 1.667, 15.639, 11.31, 0.0, 0.667, 8.843,
        7.254, -0.333, 0.333, 6.148, 5.333, -1.333, -0.667, 4.0,
        4.667, -1.667, -1.0, 3.667, 3.333, -3.333, -2.333, 2.667,
        3.0, -7.0, -4.667, 2.333, 2.0, -15.0, -10.333, 1.333,
    ],
    "horizontal_angle_offsets": [
        1.4, -4.2, 1.4, -1.4, 1.4, -1.4, 4.2, -1.4,
        1.4, -4.2, 1.4, -1.4, 4.2, -1.4, 4.2, -1.4,
        1.4, -4.2, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4,
        1.4, -1.4, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4,
    ],
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
    "horizontal": 0.171887,   # degrees (from manufacturer manual)
    "vertical": 0.0859435,    # degrees (from manufacturer manual)
}
