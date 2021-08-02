import numpy as np


deltapose_dataset_params = {
    "type": "delta_pose",
    "size": 5000,
    "world_bounds": np.array([[1, 1], [10, 1], [10, 10], [1, 10]]),
    "obstacles_gen": {
        "obs_type": "rect",
        "num": 10,
        "width_mean": 1,
        "width_std": 1,
        "height_mean": 1,
        "height_std": 1,
        "seed": 1234,
    },
    "sample_gen": {
        "num_consecutive": 2,
        "pos_std": 0.1,
        "theta_std": np.deg2rad(10),
        "seed": 4321,
    },
    "sensorbeams": {"opening_angle": 180, "num_beams": 200}, # 200
}
