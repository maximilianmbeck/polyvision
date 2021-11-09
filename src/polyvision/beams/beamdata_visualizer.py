# -*- coding: latin-1 -*-
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


class BeamDataVisualizer(object):
    def __init__(self):
        pass

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
        fig, ax = plt.subplots()
        self.ax_plot_pose(ax, pos, theta)
        plt.show()

    def ax_plot_pose(self, ax, pos, theta, scale=0.25):
        # green triangle in direction
        # generate triangle points
        angles = np.array([np.deg2rad(0), np.deg2rad(120), np.deg2rad(240)])
        points = np.array([np.cos(angles), np.sin(angles)])
        points[0] = points[0] * scale
        points[1:3] = points[1:3] * 0.5 * scale
        # transform triangle points to pose
        c = np.cos(theta)
        s = np.sin(theta)
        rotmat= np.array([[c, -s], [s, c]])

        points = np.dot(rotmat, points) + np.reshape(pos, (-1,1))
        points = points.transpose()
        pose_patch = Polygon(points, facecolor="green", zorder=10)
        ax.add_patch(pose_patch)
        ax.plot(pos[0], pos[1], marker='x', color='red', zorder=10.1, ms=20*scale)
        

    def axs_plot_beams_at_pose(self, ax_world, ax_contour, beam_data_processor, pos, theta):
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
            ax_contour.plot(np.rad2deg(angles), readings, 'o-')
            ax_contour.set_xlabel('angle in deg')
            ax_contour.set_ylabel('distance')

    def plot_world_with_beams_at_pose(self, beam_data_processor, pos, theta, seed, num_obs):
        fig, ax = plt.subplots()
        self.ax_plot_world_with_beams_at_pose(ax, beam_data_processor, pos, theta, seed, num_obs)
        plt.show()

    def ax_plot_world_with_beams_at_pose(self, ax, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world(ax, beam_data_processor, seed, num_obs)
        self.axs_plot_beams_at_pose(ax, None, beam_data_processor, pos, theta)
        self.ax_plot_pose(ax, pos, theta)

    def plot_world_with_beams_at_pose_and_contour(self, beam_data_processor, pos, theta, seed, num_obs):
        self.fig, (ax1, ax2) = plt.subplots(2, 1)
        self.axs_plot_world_with_beams_at_pose_and_contour(ax1, ax2, beam_data_processor, pos, theta, seed, num_obs)
        plt.show()

    def axs_plot_world_with_beams_at_pose_and_contour(self, ax_world, ax_contour, beam_data_processor, pos, theta, seed, num_obs):
        self.ax_plot_world(ax_world, beam_data_processor, seed, num_obs)
        self.axs_plot_beams_at_pose(ax_world, ax_contour, beam_data_processor, pos, theta)
        self.ax_plot_pose(ax_world, pos, theta)

    ## visualize samples
    def plot_world_with_single_position_samples(self, beam_data_processor, y, indices, seed, num_obs):
        fig, (ax0, ax1) = plt.subplots(1,2)
        self.ax_plot_world(ax0, beam_data_processor, seed, num_obs)
        self.ax_plot_single_position_samples(ax0, y, indices, first_pose=True)
        self.ax_plot_world(ax1, beam_data_processor, seed, num_obs)
        self.ax_plot_single_position_samples(ax1, y, indices, first_pose=False)
        plt.show()


    def ax_plot_single_position_samples(self, ax, y, sample_indices, first_pose=True):
        if first_pose:
            pose_index = 1 # row in the sample
            c = 'b'
        else:
            pose_index = 2 # row in the sample
            c = 'g'
        for ind in sample_indices:
            ax.plot(y[sample_indices,pose_index,0], y[sample_indices,pose_index,1], '.', ms=2, color=c)
        ax.set_title('num_position_samples={0}'.format(len(sample_indices)))

    def plot_angle_distribution(self, y, sample_indices, n_bins=18):
        fig, (ax0, ax1) = plt.subplots(1,2)
        self.ax_plot_angle_distribution(ax0, y, sample_indices, first_pose=True, n_bins=18)
        self.ax_plot_angle_distribution(ax1, y, sample_indices, first_pose=False, n_bins=18)
        plt.show()

    def ax_plot_angle_distribution(self, ax, y, sample_indices, first_pose=True, n_bins=18):
        if first_pose:
            pose_index = 1 # row in the sample
            c = 'b'
        else:
            pose_index = 2 # row in the sample
            c = 'g'
        for ind in sample_indices:
            ax.hist(np.rad2deg(y[sample_indices,pose_index,2]), bins=n_bins,color=c)
        ax.set_title('num_angles={0}'.format(len(sample_indices)))

    def plot_beam_samples(self, beam_data_processor, X, y, sample_indices, first_pose=True):
        fig, ax = plt.subplots()
        self.ax_plot_beam_samples(ax, beam_data_processor, X, y, sample_indices, first_pose)
        plt.show()

    def ax_plot_beam_samples(self, ax, beam_data_processor, X, y, sample_indices, first_pose=True):
        if first_pose:
            pose_index = 1
            x_readings_ind = 0
        else:
            pose_index = 2
            x_readings_ind = 1

        for i in range(len(sample_indices)):
            readings = X[sample_indices[i],x_readings_ind,:]
            pos = y[sample_indices[i], pose_index, 0:2]
            theta = y[sample_indices[i], pose_index, 2]
            points = beam_data_processor.convert_readings_to_cartesian(readings, pos, theta)
            ax.plot(points[:,0], points[:,1], 'o', ms=1, color='y')
            # ax.plot(pos[0], pos[1], '.', ms=2, color='b')
            self.ax_plot_pose(ax, pos, theta, scale=0.15)

        ax.set_title('num_pose_samples={0}'.format(len(sample_indices)))

        
def generatePolygonPatchCollection(listOfNumpyPolygons, colorV="blue", alphaV=0.4):
    polygons = []
    for p in listOfNumpyPolygons:
        polygons.append(Polygon(p, True))

    return PatchCollection(polygons, alpha=alphaV, color=colorV)