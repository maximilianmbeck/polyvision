# -*- coding: latin-1 -*-
""" This Module is for visualizing the polyvision functionality. """
import numpy as np
from polyvision_pyapi import BeamDataGenerator
from polyvision.beams.beamdata_visualizer import BeamDataVisualizer
from polyvision.sensorabstraction import generate_beam_dir_vecs
from polyvision.beams.beamdataset_generator import BeamDataProcessor, generate_random_rectangles, load_deltapose_dataset

def testBeamDataProcessor():
    # old params: 70, 5
    beamDirs, beamAngles = generate_beam_dir_vecs(180, 6, direction_angle=0)
    world_bounds = np.array([[1,1],[10,1],[10,10], [1,10]])
    seed = 1234
    num_obs = 10
    rects = generate_random_rectangles(num_obs, 1,1,1,1,world_bounds, seed)
    obstacles = rects
    bdp = BeamDataProcessor(world_bounds, obstacles, beamDirs, beamAngles)
    pos = np.array([6,3])
    theta = np.deg2rad(45)

    poset =(pos, theta)
    print(np.hstack(poset))
    intersects, readings, angles = bdp.get_sensorbeamreadings_at_pose(pos, theta)
    print(intersects)
    print(readings)
    print(np.rad2deg(angles))

    # convert readings to cartesian
    points = bdp.convert_readings_to_cartesian(readings, pos, theta, angles)
    print(points)

    bdv = BeamDataVisualizer()
    bdv.plot_world_with_beams_at_pose(bdp, pos, theta, seed, num_obs)

    bdv.plot_world_with_beams_at_pose_and_contour(bdp, pos, theta, seed, num_obs)

def testBeamDatasetLoader():
    X, y, X_col_angles, params = load_deltapose_dataset('/home/max/phd/data/slam', 'slam_data_obs_rect10_seed1234')
    bdp = BeamDataProcessor.create_from_params(params)
    seed = params['obstacles_gen']['seed']
    num_obs = params['obstacles_gen']['num']
    print(y.shape, X.shape)
    print(y[0])
    bdv = BeamDataVisualizer()
    # bdv.plot_world_with_single_position_samples(bdp, y, range(0,5000), seed, num_obs)
    # bdv.plot_angle_distribution(y, range(0,5000))

    bdv.plot_beam_samples(bdp, X, y, range(10))

def testBeamDataGenerator():
    beamDirs, beamAngles = generate_beam_dir_vecs(70, 5, directionAngle=0)
    world_bounds = np.array([[1,1],[10,1],[10,10], [1,10]])
    obstacles = []
    bdg = BeamDataGenerator(world_bounds, obstacles, beamDirs)
    pos = np.array([5.5,5.5])
    theta = 0
    intersects = bdg.getSensorbeamIntersectPointsAtPose(pos, theta)
    readings = np.linalg.norm(intersects-pos,ord=2, axis=1)
    obs = bdg.getObstacles()
    wbs = bdg.getWorldBounds()
    print(intersects)
    print(intersects-pos)
    print(readings)
    print(obs)
    print(wbs)
    print(bdg.isPointInWorld(pos))
    print(bdg.isPointInObstacles(pos))



if __name__ == "__main__":
    # testBeamDataGenerator()
    testBeamDataProcessor()
    testBeamDatasetLoader()



