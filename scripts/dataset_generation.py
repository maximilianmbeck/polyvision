# -*- coding: latin-1 -*-
import numpy as np

from polyvision.beams.beamdataset_generator import generate_deltapose_dataset, visualize_dataset, load_deltapose_dataset


def main():
    # generate_deltapose_dataset(path_from_home_dir="phd/data/slam")
    path_to_dataset_folder = '/home/max/phd/data/slam'
    dataset_name = 'slam_data_obs_rect10_seed1234'
    # visualize_dataset(path_to_dataset_folder, dataset_name)
    X, y, X_col_angles, params = load_deltapose_dataset(path_to_dataset_folder, dataset_name)

if __name__ == "__main__":
    main()