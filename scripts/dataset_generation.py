# -*- coding: latin-1 -*-
import numpy as np

from polyvision.beams.beamdataset_generator import generate_deltapose_dataset, visualize_dataset


def main():
    # generate_deltapose_dataset(path_from_home_dir="phd/data/slam")
    visualize_dataset('/home/max/phd/data/slam', 'slam_data_obs_rect10_seed1234')


if __name__ == "__main__":
    main()