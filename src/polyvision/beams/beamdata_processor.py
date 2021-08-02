# -*- coding: latin-1 -*-

import numpy as np
from polyvision_pyapi import BeamDataGenerator


class BeamDataProcessor(object):
    """Python wrapper for C++ BeamDataGenerator."""

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