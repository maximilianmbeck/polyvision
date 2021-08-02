# -*- coding: latin-1 -*-
""" This Module is for visualizing the polyvision functionality. """
from __future__ import division
import numpy as np

from polyvision.visible_area.visible_area_visualizer import generatePolygonPatchCollection, genericTestLinePlot, genericTestAnimation
from polyvision_pyapi import checkInside
from polyvision.sensorabstraction import generateFoVWedge, generate_beam_dir_vecs
from polyvision.visible_area.sensorplatform import SensorPlatform, affineTransformation

precision = 6

def main():
    radian = np.deg2rad(90)
    angles = np.array([np.deg2rad(0), np.deg2rad(120), np.deg2rad(240)])
    points = np.array([np.cos(angles), np.sin(angles)])
    print(points)
    c = np.cos(radian)
    s = np.sin(radian)
    rotmat= np.array([[c, -s], [s, c]])
    points = np.dot(rotmat, points)
    print(points)
    points = points.transpose()
    print(angles)
    print(points)




def testFoVWedge():
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()

    opening_angle = 90
    visible_range = 4
    fov1 = generateFoVWedge(opening_angle, visible_range, directionAngle=90)
    fov2 = generateFoVWedge(135, 2, directionAngle=270)
    fovs = [fov1, fov2]

    fovPatchCol = generatePolygonPatchCollection(fovs)
    ax.add_collection(fovPatchCol)

    print(checkInside(np.array([0, 2]), [fov1]))

    beamDirs, beamAngles = generate_beam_dir_vecs(70, 5)
    print(np.rad2deg(beamAngles))
    ax.scatter(beamDirs[:,0], beamDirs[:,1])
    beamDirs2 = affineTransformation(beamDirs, 45, np.array([-5,0]))
    ax.scatter(beamDirs2[:,0], beamDirs2[:,1])
    # ax.autoscale_view()
    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    ax.set_aspect("equal")
    plt.autoscale(False)
    plt.grid()
    plt.show()

def testAnimation1():
    # initialization car
    fov1 = generateFoVWedge(40, 10, directionAngle=0)
    fov2 = generateFoVWedge(140, 4, directionAngle=180)
    fov3 = generateFoVWedge(25, 7, directionAngle=-30)
    fov4 = generateFoVWedge(25, 7, directionAngle=30)
    fov5 = generateFoVWedge(40, 7, directionAngle=180)
    fov = [fov1, fov2, fov3, fov4, fov5]
    car = SensorPlatform(np.array([0, 0]), 0, fov)

    # initialization obstacles
    p1 = np.array([[-1, 5], [1, 5], [0, 8]])
    p2 = np.array([[-5, 5], [-3, 5], [-3, 3], [-5, 3]])
    p3 = np.array([[-2.2, -3.8], [-0.8, -2.2], [-1.5, -6], [-2.5, -4.5], [-3, -3]])
    p4 = np.array([[8, 4], [8, 2], [5, 0], [10, 1], [8, -2], [7, -4], [11, -1], [11, 2]])
    p5 = np.array([[-4, 0], [-6, 0], [-6, -1.5]])
    obs = [p1, p2, p3, p4, p5]
    genericTestAnimation(obs, car)


def testLine():
    # initialization car
    fov1 = generateFoVWedge(40, 10, directionAngle=0)
    fov2 = generateFoVWedge(140, 4, directionAngle=180)
    fov3 = generateFoVWedge(25, 7, directionAngle=-30)
    fov4 = generateFoVWedge(25, 7, directionAngle=30)
    fov5 = generateFoVWedge(40, 7, directionAngle=180)
    fov = [fov1, fov2, fov3, fov4, fov5]
    car = SensorPlatform(np.array([0, 0]), 0, fov)
    obs = []
    line = np.array([12.0, 0.0, 5.0, 0.0])
    genericTestLinePlot(obs, car, line)




if __name__ == "__main__":
    # main()
    # testFoVWedge()
    # testLine()
    testAnimation1()

