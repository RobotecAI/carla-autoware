"""LiDAR preset: RangeMeter (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "RangeMeter",
    "channels": 1,
    "range": 40.0,
    "min_range": 0.0,
    "upper_fov": 0.0,
    "lower_fov": 0.0,
    "horizontal_fov": 0.0,
    "rotation_frequency": 10.0,
    "points_per_second": 10,
    "vertical_angles": [
        0.0,
    ],
    "horizontal_angle_offsets": [],
    "ring_ids": [
        0,
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
