from __future__ import division
import numpy as np


def rotationMatrix(radian):
    c = np.cos(radian)
    s = np.sin(radian)
    return np.array([[c, -s], [s, c]])


def generateFoVWedge(
    openingAngle, visible_range, numberOfPointsOnCircle=15, directionAngle=90, origin=np.array([0, 0])
):
    """
    openingAngle: angle in degree
    range: range of fov
    numberOfPointsOnCircle: resolution of the circle"""

    # initialize fov polygon
    fov = np.array([origin])

    # angles of the circle approximation
    angles = np.linspace(directionAngle - openingAngle / 2, directionAngle + openingAngle / 2, numberOfPointsOnCircle)

    # first approx.
    rad = np.radians(angles[0])
    l = [np.cos(rad) * visible_range, np.sin(rad) * visible_range]

    # add points to polygon
    for a in angles:
        rotateMatrix = rotationMatrix(np.radians(a) - rad)
        tempPoint = np.dot(rotateMatrix, l) + origin
        # print "angle : ", a, " point : ", tempPoint, c, s
        fov = np.append(fov, np.array([tempPoint]), axis=0)

    return fov


def generate_beam_dir_vecs(opening_angle, num_beams, direction_angle=0, visible_range=1.0):
    # angles of the circle approximation
    angles = np.linspace(direction_angle - opening_angle / 2, direction_angle + opening_angle / 2, num_beams)

    # first approx.
    rad = np.deg2rad(angles[0])
    l = [np.cos(rad) * visible_range, np.sin(rad) * visible_range]

    dirs = np.zeros(shape=(num_beams,2))

    i = 0
    # add vectors to list
    for a in angles:
        rotateMatrix = rotationMatrix(np.deg2rad(a) - rad)
        tempPoint = np.dot(rotateMatrix, l)
        # print "angle : ", a, " point : ", tempPoint, c, s
        dirs[i,:] = tempPoint
        i+=1

    return dirs, np.deg2rad(angles)