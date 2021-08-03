# -*- coding: latin-1 -*-
import numpy as np

from polyvision.beams.beamdataset_generator import generate_deltapose_dataset


def main():
    generate_deltapose_dataset(path_from_home_dir="phd/data/slam")


if __name__ == "__main__":
    main()