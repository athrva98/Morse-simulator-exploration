# Definition for Robot CustomFootprint
import cv2
import matplotlib.pyplot as plt
import numpy as np
from morse_simulator.algorithms.collision_cpp import check_for_collision
from morse_simulator.algorithms.costmap import Costmap
from morse_simulator.algorithms.util import which_coords_in_bounds, compute_circumscribed_radius, \
    clip_range, get_rotation_matrix_2d


class CustomFootprint:
    """
    Footprint object that is created using a ndarray of points. it will draw a hull around them, and collision
    check using the hull.
    """
    def __init__(self, footprint_points, angular_resolution, inflation_scale=1.0):
        """
        Define a footprint as the filled in hull of the points selected. The first point of the hull_points must
        be where 0 radians is. The points must be inorder either counter clockwise or clockwise.
        :param footprint_points array(N,2)[float]: points to "connect" to make the footprint
        :param angular_resolution float: radian resolution at which to precompute the hulls for
        :param inflation_scale float: scaling factor on the the footprint, inflating its size
        """
        assert angular_resolution > 0.001 * np.pi / 180.\
            and "too small resolution. " \
                "should probably implement with degrees (no multiple of irrational numbers with degrees)"

        self._inflation_scale = inflation_scale
        self._footprint_points = footprint_points
        self._inflated_footprint_points = footprint_points * inflation_scale
        self._angular_resolution = angular_resolution

        self._mask_angles, self._rotated_masks = self._precompute_rotated_footprint_points()
        self._rotated_masks_set = dict()
        self._rotated_outline_coords_set = dict()
        self._mask_radius_set = dict()

    def copy(self):
        """
        Returns a copy of the current footprint object
        :return CustomFootprint: a copy of the current object
        """
        return CustomFootprint(footprint_points=self._footprint_points.copy(),
                               angular_resolution=self._angular_resolution,
                               inflation_scale=self._inflation_scale)

    def no_inflation(self):
        """
        Returns a copy of the current footprint object but with no inflation.
        :return CustomFootprint: a copy of the current object but with no inflation.
        """
        return CustomFootprint(footprint_points=self._footprint_points.copy(),
                               angular_resolution=self._angular_resolution,
                               inflation_scale=1.0)

    def _precompute_rotated_footprint_points(self, debug=False):
        """
        Precompute rotated hulls to all angles specified by the angular_resolution
        :param debug bool: show debug plots?
        :return Union[Tuple[array(N)[float], List(array(N, 2)[float]]]]: angles of which the hulls were rotated, the rotated hulls
        """
        rotation_angles = np.arange(-np.pi, np.pi, self._angular_resolution)
        # TODO WHY NOT get_rotation_matrix_2d(-angle))
        rotated_hulls = [self._inflated_footprint_points.dot(get_rotation_matrix_2d(angle)) for angle in rotation_angles]
        if debug:
            for rotated_point in rotated_hulls:
                plt.plot(rotated_point[:, 0], rotated_point[:, 1])
                plt.show()

        return rotation_angles, rotated_hulls

    def _add_new_masks(self, resolution, debug=False):
        """
        With a specified resolution, we can compute and save a mask for the footprint for that resolution.
        This allows us to check footprints very fast.
        :param resolution float: desired resolution to compute the footprint mask
        :param debug bool: show debug plots?
        """
        # compute needed array size of the footprint mask
        first_hull_px = np.ceil(self._rotated_masks[0] / resolution).astype(np.int)[:, ::-1]

        # add +1 to give a pixel footprint a minimum radius > 0
        mask_radius = np.ceil(compute_circumscribed_radius(first_hull_px)).astype(np.int) + 1
        mask_shape = np.array([2 * mask_radius + 1, 2 * mask_radius + 1])
        ego_coord = np.floor(mask_shape / 2.).astype(np.int)

        # loop through all the rotated hulls, and rasterize them onto the mask
        rotated_footprint_masks = []
        rotated_outline_coords = []
        for i, rotated_hull in enumerate(self._rotated_masks):
            rotated_hull_px = np.ceil(rotated_hull / resolution).astype(np.int)[:, ::-1] + ego_coord
            footprint_mask = -1 * np.ones(mask_shape)
            cv2.drawContours(footprint_mask, [rotated_hull_px[:, ::-1]], contourIdx=0,
                             color=Costmap.OCCUPIED, thickness=-1)
            rotated_footprint_masks.append(footprint_mask)
            rotated_outline_coords.append(rotated_hull_px - ego_coord)
            if debug and i % 10 == 0:
                mask_vis = rotated_footprint_masks[i].copy()
                mask_vis[ego_coord[0], ego_coord[1]] = Costmap.OCCUPIED
                plt.imshow(mask_vis)
                plt.show()

        # save these so we dont have recompute later
        self._mask_radius_set[resolution] = mask_radius
        self._rotated_outline_coords_set[resolution] = np.array(rotated_outline_coords)
        self._rotated_masks_set[resolution] = np.array(rotated_footprint_masks)

    def check_for_collision(self, state, occupancy_map, unexplored_is_occupied=False, use_python=False, debug=False):
        """
        using the state and the map, check for a collision on the map.
        :param state array(3)[float]: state of the robot [x, y, theta]
        :param occupancy_map Costmap: object to check the footprint against
        :param unexplored_is_occupied bool: whether to treat unexplored on the map as occupied
        :param use_python bool: whether to use python collision checking or c++
        :param debug bool: show debug plots?
        :return bool: True for collision
        """
        # check if we have computed a mask for this resolution, if not compute it
        if occupancy_map.resolution not in list(self._rotated_masks_set.keys()):
            self._add_new_masks(occupancy_map.resolution)

        # convert state to pixel coordinates
        state_px = state # the state is already in the grid Frame
        position = np.array(state_px[:2]).astype(np.int)

        # get the mask radius that was saved for this resolution
        mask_radius = self._mask_radius_set[occupancy_map.resolution]

        # compute the closest angle to the current angle in the state, and get that rotated footprint
        closest_angle_ind = np.argmin(np.abs(state_px[2] - self._mask_angles))
        footprint_mask = self._rotated_masks_set[occupancy_map.resolution][closest_angle_ind]
        outline_coords = self._rotated_outline_coords_set[occupancy_map.resolution][closest_angle_ind]

        if not use_python:
            obstacle_values = [Costmap.OCCUPIED, Costmap.UNEXPLORED] if unexplored_is_occupied else [Costmap.OCCUPIED]

            is_colliding = check_for_collision(state_px,
                                               occupancy_map,
                                               footprint_mask=footprint_mask,
                                               outline_coords=outline_coords,
                                               obstacle_values=obstacle_values)
        else:
            # part of robot off the edge of map, it is a collision, because we assume map is bounded
            if not np.all(which_coords_in_bounds(coords=(outline_coords + state_px[:2]).astype(np.int),
                                                 map_shape=occupancy_map.get_shape())):
                return True

            # get a small subsection around the state in the map (as big as our mask),
            # and do a masking check with the footprint to see if there is a collision.
            # if part of our subsection is off the map, we need to clip the size
            # to reflect this, in both the subsection and the mask
            min_range = [position[0] - mask_radius, position[1] - mask_radius]
            max_range = [position[0] + mask_radius + 1, position[1] + mask_radius + 1]
            clipped_min_range, clipped_max_range = clip_range(min_range, max_range, occupancy_map.data.shape)

            min_range_delta = clipped_min_range - np.array(min_range)
            max_range_delta = clipped_max_range - np.array(max_range)

            ego_map = occupancy_map.data[clipped_min_range[0]:clipped_max_range[0],
                                         clipped_min_range[1]:clipped_max_range[1]].copy()

            # todo might be - max_range_delta
            footprint_mask = footprint_mask[min_range_delta[0]:footprint_mask.shape[1] + max_range_delta[0],
                                            min_range_delta[1]:footprint_mask.shape[1] + max_range_delta[1]]

            # treat unexplored space as obstacle
            if unexplored_is_occupied:
                ego_map[ego_map == Costmap.UNEXPLORED] = Costmap.OCCUPIED

            is_colliding = np.any(footprint_mask == ego_map)

            if debug:
                plt.imshow(footprint_mask - 0.1 * ego_map)
                plt.show()

        return is_colliding

    


