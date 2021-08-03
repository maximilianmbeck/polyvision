# -*- coding: latin-1 -*-

import numpy as np
from numpy.lib.function_base import angle
from polyvision_pyapi import BeamDataGenerator


class BeamDataProcessor(object):
    """Python wrapper for C++ BeamDataGenerator."""

    @staticmethod
    def create_from_params(params_dict):
        from polyvision.sensorabstraction import generate_beam_dir_vecs
        from polyvision.beams.beamdataset_generator import generate_random_rectangles
        opening_angle = params_dict['sensorbeams']['opening_angle']
        num_beams = params_dict['sensorbeams']['num_beams']

        beam_dirs, beam_angles = generate_beam_dir_vecs(
            opening_angle, num_beams, direction_angle=0)

        world_bounds = np.array(params_dict['world_bounds'])
        seed_obs = params_dict['obstacles_gen']['seed']
        num_obs = params_dict['obstacles_gen']['num']
        w_mean, w_std, h_mean, h_std = params_dict['obstacles_gen']['width_mean'], params_dict['obstacles_gen'][
            'width_std'], params_dict['obstacles_gen']['height_mean'], params_dict['obstacles_gen']['height_std']
        obs_type = params_dict["obstacles_gen"]["obs_type"]
        if obs_type == "rect":
            rects = generate_random_rectangles(num_obs, w_mean, w_std, h_mean, h_std, world_bounds, seed_obs)
            obstacles = rects

        return BeamDataProcessor(world_bounds, obstacles, beam_dirs, beam_angles)

    def __init__(self, world_bounds, obstacles, beam_dirs, beam_angles):
        self._beam_angles = beam_angles
        self._bdg = BeamDataGenerator(world_bounds, obstacles, beam_dirs)

    @property
    def beam_angles(self):
        return self._beam_angles

    @property
    def world_bounds(self):
        return self._bdg.getWorldBounds()

    @property
    def obstacles(self):
        return self._bdg.getObstacles()

    def is_point_in_world(self, point):
        """point: ndarray (1x2)"""
        return self._bdg.isPointInWorld(point)

    def is_point_in_obstacles(self, point):
        """point: ndarray (1x2)"""
        return self._bdg.isPointInObstacles(point)

    def get_sensorbeamreadings_at_pose(self, pos, theta):
        """pos: ndarray (1x2)
        Returns:
        intersects (intersection points): ndarray (num_beams x 2)
        readings (distances to intersection points): ndarray (num_beams,)
        angles (in rad): ndarray (num_beams,) """
        intersects = self._bdg.getSensorbeamIntersectPointsAtPose(pos, theta)
        # calculate distances from pos to intersects
        readings = np.linalg.norm(intersects - pos, ord=2, axis=1)

        return intersects, readings, self.beam_angles

    def convert_readings_to_cartesian(self, readings, pos, theta, angles=None):
        if angles is None:
            angles = self._beam_angles
        rot_angles = angles+theta
        dir_vecs = np.transpose(np.array([np.cos(rot_angles), np.sin(rot_angles)]))
        points = dir_vecs * readings.reshape(-1,1) + pos
        return points
