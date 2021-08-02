# -*- coding: latin-1 -*-
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


class BeamDataVisualizer(object):
    def __init__(self, ax=None):
        if ax is None:
            self._fig, self._ax = plt.subplots()
        else:
            self._ax = ax

    def plot_world(self, beam_data_processor, seed, num_obs):
        self.ax_plot_world(self._ax, beam_data_processor, seed, num_obs)
        plt.show()

    def ax_plot_world(self, ax, beam_data_processor, seed, num_obs):
        ax.clear()
        world_bounds = beam_data_processor.world_bounds
        # ax limits
        xlim_lower = np.min(world_bounds[:, 0]) - 1
        xlim_upper = np.max(world_bounds[:, 0]) + 1
        ylim_lower = np.min(world_bounds[:, 1]) - 1
        ylim_upper = np.max(world_bounds[:, 1]) + 1

        ax.set_xlim(xlim_lower, xlim_upper)
        ax.set_ylim(ylim_lower, ylim_upper)
        ax.set_aspect("equal")

        # plot world bounds
        wb_patch = Polygon(
            world_bounds, facecolor="None", edgecolor="black", linewidth=3, zorder=-1
        )
        ax.add_patch(wb_patch)

        # plot obstacles
        obstacles = beam_data_processor.obstacles
        obs_patch = generatePolygonPatchCollection(obstacles, "red", 1.0)
        ax.add_collection(obs_patch)

        # set title
        ax.set_title("num_obstacles={0}, seed={1}".format(num_obs, seed))

    def plot_pose(self, pos, theta):
        self.ax_plot_pose(self._ax, pos, theta)
        plt.show()

    def ax_plot_pose(self, ax, pos, theta, scale=0.25):
        # green triangle in direction
        # generate triangle points
        angles = np.array([np.deg2rad(0), np.deg2rad(120), np.deg2rad(240)])
        points = np.array([np.cos(angles), np.sin(angles)])
        points = points * scale
        # transform triangle points to pose
        c = np.cos(theta)
        s = np.sin(theta)
        rotmat= np.array([[c, -s], [s, c]])

        points = np.dot(rotmat, points) + np.reshape(pos, (-1,1))
        points = points.transpose()
        pose_patch = Polygon(points, facecolor="green", zorder=10)
        ax.add_patch(pose_patch)
        ax.plot(pos[0], pos[1], marker='x', color='red', zorder=10.1, ms=5)
        

    def ax_plot_beams_at_pose(self, ax, beam_data_processor, pos, theta):
        # plot beams
        (
            intersects,
            readings,
            angles,
        ) = beam_data_processor.get_sensorbeamreadings_at_pose(pos, theta)

        for i in range(len(readings)):
            ax.plot([pos[0], intersects[i, 0]], [pos[1], intersects[i, 1]], color="b")
            ax.plot(intersects[i, 0], intersects[i, 1], "o", ms=6, color="y")

    def plot_world_with_beams_at_pose(self, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world_with_beams_at_pose(self._ax, beam_data_processor, pos, theta, seed, num_obs)
        plt.show()

    def ax_plot_world_with_beams_at_pose(self, ax, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world(ax, beam_data_processor, seed, num_obs)
        self.ax_plot_beams_at_pose(ax, beam_data_processor, pos, theta)
        self.ax_plot_pose(ax, pos, theta)


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
        frames=range(0, 100),
        init_func=initAnimation,
        blit=False,
        repeat=True,
        interval=10,
    )

    plt.show()


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
