# -*- coding: latin-1 -*-
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

def generatePolygonPatchCollection(listOfNumpyPolygons, colorV="blue", alphaV=0.4):
    polygons = []
    for p in listOfNumpyPolygons:
        polygons.append(Polygon(p, True))

    return PatchCollection(polygons, alpha=alphaV, color=colorV)


def genericTestLinePlot(obstacles, car, line):
    # plotting
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots()

    plt.cla()
    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    # ax.set_xlim(-17, 17)
    # ax.set_ylim(-17, 17)
    ax.set_aspect("equal")
    plt.autoscale(False)
    car.updateSensorPlatformPose(np.array([0, 0]), np.deg2rad(30))

    # get visible area calculation results
    visA = car.getVisibleArea(obstacles)

    # plot obstacles
    obsPatchCol = generatePolygonPatchCollection(visA[1], "red", 0.8)
    ax.add_collection(obsPatchCol)

    # plot visible area
    visAreaPatchCol = generatePolygonPatchCollection(visA[2], "blue", 0.4)
    ax.add_collection(visAreaPatchCol)

    # plot boundary of visible area with a centerline
    print("Line-point coordinates")
    intersection = car.getVisibilityBorder(obstacles, line)[0]
    print(intersection)
    ax.plot([line[0], line[2]], [line[1], line[3]])
    ax.plot(intersection[0], intersection[1], "o", ms=8)

    # plot non-visible area
    visAreaPatchCol = generatePolygonPatchCollection(visA[3], "grey", 0.4)
    ax.add_collection(visAreaPatchCol)

    plt.show()


def genericTestAnimation(obstacles, car):
    # plotting
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots()

    def initAnimation():
        # ax.autoscale_view()
        # ax.set_xlim(-12, 12)
        # ax.set_ylim(-12, 12)
        ax.set_xlim(-17, 17)
        ax.set_ylim(-17, 17)
        ax.set_aspect("equal")
        plt.autoscale(False)

    def animate(frame):
        plt.cla()
        ax.set_xlim(-12, 12)
        ax.set_ylim(-12, 12)
        # ax.set_xlim(-17, 17)
        # ax.set_ylim(-17, 17)
        ax.set_aspect("equal")
        plt.autoscale(False)
        car.updateSensorPlatformPose(np.array([0, 0]), np.deg2rad(30))

        # get visible area calculation results
        visA = car.getVisibleArea(obstacles)

        # plot obstacles
        obsPatchCol = generatePolygonPatchCollection(visA[1], "red", 0.8)
        ax.add_collection(obsPatchCol)

        # plot visible area
        visAreaPatchCol = generatePolygonPatchCollection(visA[2], "blue", 0.4)
        ax.add_collection(visAreaPatchCol)

        # plot non-visible area
        visAreaPatchCol = generatePolygonPatchCollection(visA[3], "grey", 0.4)
        ax.add_collection(visAreaPatchCol)

    ani = FuncAnimation(
        fig,
        animate,
        frames=range(0, 300),
        init_func=initAnimation,
        blit=False,
        repeat=True,
        interval=1,
    )

    return ani


def genericTestPlot(obstacles, car):
    # plotting
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()

    import time

    start_time = time.time()
    # get visible area calculation results
    visAList = car.getVisibleArea(obstacles)
    print("--- %s seconds ---" % (time.time() - start_time))

    # plot obstacles
    obsPatchCol = generatePolygonPatchCollection(visAList[1], "red", 0.8)
    ax.add_collection(obsPatchCol)

    # plot visible area
    visAreaPatchCol = generatePolygonPatchCollection(visAList[2], "blue", 0.4)
    ax.add_collection(visAreaPatchCol)

    # plot non-visible area
    visAreaPatchCol = generatePolygonPatchCollection(visAList[3], "grey", 0.4)
    ax.add_collection(visAreaPatchCol)

    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    # ax.set_xlim(-17, 17)
    # ax.set_ylim(-17, 17)
    ax.set_aspect("equal")
    plt.autoscale(False)
    plt.show()
