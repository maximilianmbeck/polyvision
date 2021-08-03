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
        

    def ax_plot_beams_at_pose(self, ax_world, ax_contour, beam_data_processor, pos, theta):
        # plot beams
        (
            intersects,
            readings,
            angles,
        ) = beam_data_processor.get_sensorbeamreadings_at_pose(pos, theta)

        if ax_world is not None:
            for i in range(len(readings)):
                ax_world.plot([pos[0], intersects[i, 0]], [pos[1], intersects[i, 1]], color="b")
                ax_world.plot(intersects[i, 0], intersects[i, 1], "o", ms=6, color="y")

        if ax_contour is not None:
            ax_contour.plot(angles, readings)

    def plot_world_with_beams_at_pose(self, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world_with_beams_at_pose(self._ax, beam_data_processor, pos, theta, seed, num_obs)
        plt.show()

    def ax_plot_world_with_beams_at_pose(self, ax, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world(ax, beam_data_processor, seed, num_obs)
        self.ax_plot_beams_at_pose(ax, None, beam_data_processor, pos, theta)
        self.ax_plot_pose(ax, pos, theta)


def generatePolygonPatchCollection(listOfNumpyPolygons, colorV="blue", alphaV=0.4):
    polygons = []
    for p in listOfNumpyPolygons:
        polygons.append(Polygon(p, True))

    return PatchCollection(polygons, alpha=alphaV, color=colorV)