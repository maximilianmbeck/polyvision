# -*- coding: latin-1 -*-

import numpy as np

from polyvision.beams.beamdata_processor import BeamDataProcessor
from polyvision.beams.beamdata_visualizer import generatePolygonPatchCollection, BeamDataVisualizer
from polyvision.sensorabstraction import generate_beam_dir_vecs
from polyvision.beams.params import deltapose_dataset_params

def load_deltapose_dataset(path_to_dataset_folder, dataset_name):
    from pathlib import Path
    import json
    import pickle

    print('here')
    dataset_path = Path(path_to_dataset_folder) / dataset_name
    print(dataset_path.is_dir())

    # load params
    params_path = dataset_path / 'params.json'
    with params_path.open() as f:
        params = json.load(f)

    # load dataset 
    datasetfile_path = dataset_path / str(dataset_name+'.pickle')
    with datasetfile_path.open(mode='rb') as f:
        dataset = pickle.load(f)
    X, y, X_col_angles = dataset['X'], dataset['y'], dataset['X_columns']

    return X, y, X_col_angles, params

def generate_deltapose_dataset(datasetsize=None, path_from_home_dir=""):
    params = deltapose_dataset_params
    dataset_gen = BeamDatasetGenerator_deltapose(params)

    X, y, angles = dataset_gen.generate_dataset(datasetsize)

    print(X.shape)
    print(y.shape)
    print(angles.shape)
    dataset = {"X": X, "y": y, "X_columns": angles}

    import os
    import pathlib
    import matplotlib.pyplot as plt

    home_path = os.path.expanduser("~")
    slam_data_path = os.path.join(home_path, path_from_home_dir)

    folder_name = "slam_data_obs_{0}{1}_seed{2}".format(
        params["obstacles_gen"]["obs_type"],
        params["obstacles_gen"]["num"],
        params["obstacles_gen"]["seed"],
    )
    pathlib.Path(slam_data_path + "/" + folder_name).mkdir(parents=True, exist_ok=True)

    slam_dataset_path = pathlib.Path(slam_data_path) / folder_name

    # save plots
    fig, ax = plt.subplots()
    
    # plot1: world visualization
    dataset_gen.beam_data_visualizer.ax_plot_world(ax, 
        dataset_gen._beam_data_processor, params["obstacles_gen"]["seed"], params["obstacles_gen"]["num"]
    )
    fig.savefig(slam_data_path+'/'+folder_name+'/world.png')

    ax.clear()
    # plot2: pose visualization
    pos = np.array([5,3])
    theta = np.deg2rad(90)
    dataset_gen.beam_data_visualizer.ax_plot_world_with_beams_at_pose(ax,
        dataset_gen._beam_data_processor, pos, theta, params["obstacles_gen"]["seed"], params["obstacles_gen"]["num"]
    )
    fig.savefig(slam_data_path+'/'+folder_name+'/world_beam.png')

    save_dataset(dataset, slam_dataset_path, folder_name)
    save_params(params, slam_dataset_path)

    return X, y, angles

def save_dataset(dataset, path, filename):
    import pickle
    with open(path.joinpath(filename + ".pickle"), "wb") as f:
        pickle.dump(dataset, f)

def save_params(params, path): 
    import json
    # save parameters
    params["world_bounds"] = params["world_bounds"].tolist()

    with open(path.joinpath("params.json"), "w") as f:
        json.dump(params, f, ensure_ascii=False, indent=4)

class BeamDatasetGenerator_deltapose(object):
    """This class generates a dataset for learning the deltas between two poses."""

    def __init__(self, parameter_dict):
        self._params = parameter_dict
        self._beam_data_processor = self._create_beam_data_processor()

    @property
    def beam_data_processor(self):
        return self._beam_data_processor

    @property
    def beam_data_visualizer(self, ax=None):
        return BeamDataVisualizer(ax)

    def _create_beam_data_processor(self):
        return BeamDataProcessor.create_from_params(self._params)

    def generate_dataset(self, datasetsize=None):
        # random number generator dataset samples
        from numpy.random import default_rng
        import time

        seed_sample_gen = self._params["sample_gen"]["seed"]
        rng = default_rng(seed_sample_gen)

        if datasetsize is None:
            dataset_size = self._params["size"]
        else:
            dataset_size = datasetsize

        num_beams = self._params["sensorbeams"]["num_beams"]

        # dim dataset X: (size x 2 x num_beams) -> beams
        # dim dataset y: (size x 3 x 3) -> pose delta (x,y,theta)
        X = np.zeros((dataset_size, 2, num_beams))
        y = np.zeros((dataset_size, 3, 3))
        beam_angles = self._beam_data_processor.beam_angles  # column headers for X

        start_t = time.time()
        for i in range(dataset_size):
            X[i, :, :], y[i, :, :] = self._generate_data_sample(rng, num_beams)
            if i % 100 == 0:
                cur_t = time.time()
                elapsed = cur_t - start_t
                print("{0} samples generated in {1}s.".format(i, elapsed))

        return X, y, beam_angles

    def _generate_data_sample(self, rng, num_beams):

        X = np.zeros((2, num_beams))
        y = np.zeros((1, 3, 3))

        pos_std = self._params["sample_gen"]["pos_std"]
        theta_std = self._params["sample_gen"]["theta_std"]
        num_consecutive = self._params["sample_gen"]["num_consecutive"]

        poses = sample_consecutive_poses(
            self._beam_data_processor, pos_std, theta_std, rng, num_consecutive
        )
        pose0 = poses[0]
        pose1 = poses[1]
        pose_d = pose_delta(pose0, pose1)

        # output data: y (delta pose, pose0 + pose1 for debugging)
        y[0, 0, :] = np.hstack(pose_d)
        y[0, 1, :] = np.hstack(pose0)
        y[0, 2, :] = np.hstack(pose1)

        # input data: X
        (
            intersects0,
            readings0,
            angles0,
        ) = self._beam_data_processor.get_sensorbeamreadings_at_pose(pose0[0], pose0[1])
        (
            intersects1,
            readings1,
            angles1,
        ) = self._beam_data_processor.get_sensorbeamreadings_at_pose(pose1[0], pose1[1])

        X[0, :] = readings0
        X[1, :] = readings1

        return X, y

def pose_delta(pose0, pose1):
    """pose: tuple(np.ndarray,theta)"""
    pos_delta = pose1[0] - pose0[0]
    theta_delta = pose1[1] - pose0[1]
    return (pos_delta, theta_delta)


def sample_consecutive_poses(beam_data_processor, pos_std, theta_std, rng, num=2):
    """Returns list of tuples of consecutive poses [(np.ndarray, float), ()]"""
    poses = []
    pos, theta = sample_initial_pose(beam_data_processor, rng)
    poses.append((pos, theta))
    i = 1
    while i < num:
        pos, theta = sample_consecutive_pose(
            beam_data_processor, pos, theta, pos_std, theta_std, rng
        )
        poses.append((pos, theta))
        i += 1
    return poses


def sample_initial_pose(beam_data_processor, rng):
    world_bounds = beam_data_processor.world_bounds
    xlim_lower = np.min(world_bounds[:, 0])
    xlim_upper = np.max(world_bounds[:, 0])
    ylim_lower = np.min(world_bounds[:, 1])
    ylim_upper = np.max(world_bounds[:, 1])
    # rejection sampling -> sample initial pos in world and not in obstacles
    while True:
        x = rng.uniform(xlim_lower, xlim_upper)
        y = rng.uniform(ylim_lower, ylim_upper)
        p = np.array([x, y])
        if beam_data_processor.is_point_in_world(
            p
        ) and not beam_data_processor.is_point_in_obstacles(p):
            break
    pos = p
    theta = rng.uniform(0, 2 * np.pi)
    return pos, theta


def sample_consecutive_pose(beam_data_processor, pos, theta, pos_std, theta_std, rng):
    """Given the current pose (pos,theta), returns a consecutive pose in a random walk fashion (with constraints induced by the world)."""
    # rejection sampling -> sample new pos in world and not in obstacles
    while True:
        x = rng.normal(pos[0], pos_std)
        y = rng.normal(pos[1], pos_std)
        p = np.array([x, y])
        if beam_data_processor.is_point_in_world(
            p
        ) and not beam_data_processor.is_point_in_obstacles(p):
            break

    pos_ = p
    theta_ = rng.normal(theta, theta_std)

    return pos_, theta_


def generate_random_rectangles(
    num, width_mean, width_std, height_mean, height_std, world_bounds, seed=None
):
    from numpy.random import default_rng

    rng = default_rng(seed)
    xlim_lower = np.min(world_bounds[:, 0])
    xlim_upper = np.max(world_bounds[:, 0])
    ylim_lower = np.min(world_bounds[:, 1])
    ylim_upper = np.max(world_bounds[:, 1])

    rects = []

    for i in range(num):
        # sample center pos
        x_pos = rng.uniform(xlim_lower, xlim_upper)
        y_pos = rng.uniform(ylim_lower, ylim_upper)

        # sample width + height
        w = rng.normal(width_mean, width_std)
        h = rng.normal(height_mean, height_std)

        ll = np.array([x_pos - w / 2, y_pos - h / 2])
        lr = np.array([x_pos + w / 2, y_pos - h / 2])
        ur = np.array([x_pos + w / 2, y_pos + h / 2])
        ul = np.array([x_pos - w / 2, y_pos + h / 2])

        rect = np.array([ll, lr, ur, ul])
        rects.append(rect)

    return rects
