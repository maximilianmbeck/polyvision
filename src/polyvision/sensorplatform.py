# -*- coding: latin-1 -*-
"""This module describes the SensorPlatform class. """
from __future__ import division
import numpy as np

import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D
from matplotlib.patches import Polygon, PathPatch
from matplotlib.path import Path
from matplotlib.collections import PatchCollection
from beamdatagenerator_pyapi import VisibleArea


def affineTransformationOfPoylgonList(polylist, angle, offset, precision=-1):
    for i in range(0, len(polylist)):
        polylist[i] = affineTransformation(
            polylist[i], angle, offset, precision
        )
    return polylist

# unused:
def affineTransformationOfPolygon(polygon, angle, offset, precision=-1):
    def pointTrafo(p):
        return affineTransformation(p, angle, offset, precision)

    # apply transformation on each point in the polygon (each line in the numpyArray)
    transformedPolygon = np.apply_along_axis(pointTrafo, 1, polygon)
    return transformedPolygon


def affineTransformation(points, angle, offset, precision=-1):
    c = np.cos(np.deg2rad(angle))
    s = np.sin(np.deg2rad(angle))
    rotateMatrix = np.array([[c, -s], [s, c]])
    p = np.dot(rotateMatrix, np.transpose(points))
    p = np.transpose(p)
    p += offset
    if precision != -1:
        p = np.round(p, precision)
    return p


class SensorPlatform(object):
    """
    A SensorPlatform
    """

    def __init__(
        self,
        position: np.ndarray,
        yawAngle: float,
        fieldOfViewContour=[],
        sensor_beam_dirs=None,
        world_bounds=None,
    ):
        """
        Arguments
        ---------
        fieldOfViewContour: list of ndarrays dim(Xx2)
        world_bounds: ndarray dim(Xx2) 
            points describing the outer boundary of the world, used for sensor beams
        """
        self._sensor_beam_directions = sensor_beam_dirs
        self._fieldOfView = fieldOfViewContour
        self._position = np.array([0, 0])
        self._yawAngle = 0
        self.updateSensorPlatformPose(position, yawAngle)

    @property
    def position(self):
        """The position of the SensorPlatform as 1x2 NumpyArray"""
        return self._position

    @property
    def yawAngle(self):
        """The yaw angle of the SensorPlatform"""
        return self._yawAngle

    @property
    def fieldOfView(self):
        """The field of view contour"""
        return self._fieldOfView

    def updateSensorPlatformPose(self, deltaPosition, deltaYawAngle):
        self._position += deltaPosition
        self._yawAngle += deltaYawAngle
        # affine transformation of field of view
        if self._fieldOfView:
            self._fieldOfView = affineTransformationOfPoylgonList(
                self._fieldOfView, deltaYawAngle, deltaPosition
            )
        if self._sensor_beam_directions is not None:
            self._sensor_beam_directions = affineTransformation(self._sensor_beam_directions, deltaYawAngle, deltaPosition)

    def getVisibleArea(self, perceptedPolygons):
        # create visibleArea object for calculations
        visA = VisibleArea(self._position, self._fieldOfView, perceptedPolygons)
        # calculate visible area
        visA.calculateVisibleArea()
        results = [
            visA.getFieldsOfView(),
            visA.getOpaquePolygons(),
            visA.getVisibleAreas(),
            visA.getNonVisibleAreas(),
        ]
        return results

    def getSensorBeamReadings(self, perceptedPolygons):
        # create sensor beam object for calculations

        # calculate sensor beam readings

        # returns readings as array
        pass

    def getVisibilityBorder(self, perceptedPolygons, centerline):
        # create visibleArea object for calculations
        visA = VisibleArea(self._position, self._fieldOfView, perceptedPolygons)
        # calculate visible area
        return visA.getVisibilityBorder(centerline)
