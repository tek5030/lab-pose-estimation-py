import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class CameraModel:
    """Struct for camera model data."""

    K: np.ndarray             # camera matrix
    dist_coeffs: np.ndarray   # distortion coefficients

    @property
    def principal_point(self):
        return self.K[0, 2], self.K[1, 2]

    @property
    def focal_lengths(self):
        return self.K[0, 0], self.K[1, 1]


def retain_best(keypoints, num_to_keep):
    num_to_keep = np.minimum(num_to_keep, len(keypoints))
    best = np.argpartition([p.response for p in keypoints], -num_to_keep)[-num_to_keep:]
    return best


def extract_good_ratio_matches(matches, max_ratio):
    """
    Extracts a set of good matches according to the ratio test.

    :param matches: Input set of matches, the best and the second best match for each putative correspondence.
    :param max_ratio: Maximum acceptable ratio between the best and the next best match.
    :return: The set of matches that pass the ratio test.
    """
    if len(matches) == 0:
        return ()

    matches_arr = np.asarray(matches)
    distances = np.array([m.distance for m in matches_arr.ravel()]).reshape(matches_arr.shape)
    good = distances[:, 0] < distances[:, 1] * max_ratio

    # Return a tuple of good DMatch objects.
    return tuple(matches_arr[good, 0])


class PlaneReference:
    def __init__(
            self,
            image_size,
            scene_size,
            origin=np.array([0.0, 0.0, 0.0]),
            x_dir=np.array([1.0, 0.0, 0.0]),
            y_dir=np.array([0.0, 1.0, 0.0])
    ):
        """
        :param image_size: as output from image.shape
        :param scene_size:
        :param origin:
        :param x_dir:
        :param y_dir:
        """
        self._origin = np.asarray(origin)
        self._x_dir = np.asarray(x_dir)
        self._y_dir = np.asarray(y_dir)
        # TODO: [0] og [1] x og y, shape or size
        self._units_per_pixel_x = scene_size[1] / image_size[1]
        self._units_per_pixel_y = scene_size[0] / image_size[0]

    def pixel_to_world(self, pixel):
        return self._origin \
               + (pixel[0] * self._units_per_pixel_x) * self._x_dir \
               + (pixel[1] * self._units_per_pixel_y) * self._y_dir


class PlaneWorldModel:
    """Represents a planar world."""

    def __init__(self, world_image, world_size, grid_size):
        """
        Constructs the world model.

        :param world_image: The world map image.
        :param world_size: The physical size of the world corresponding to the image in meters.
        :param grid_size: Size of the grid cells in the world image in meters.
        """
        self._world_image = world_image
        self._world_size = world_size
        self._grid_size = grid_size
        self._max_num_points = 1000
        self._max_ratio = 0.8

        self._construct_world()

    def _construct_world(self):
        # Convert to gray scale.
        gray_img = cv2.cvtColor(self._world_image, cv2.COLOR_BGR2GRAY)

        # Set up objects for detection, description and matching.
        self._detector = cv2.ORB_create(nfeatures=1000)
        self._desc_extractor = cv2.ORB_create()
        self._matcher = cv2.BFMatcher_create(self._desc_extractor.defaultNorm())

        # Detect keypoints
        keypoints = np.asarray(self._detector.detect(gray_img))
        best = retain_best(keypoints, self._max_num_points)
        keypoints = keypoints[best]

        # Compute descriptors for each keypoint.
        keypoints, new_descriptors = self._desc_extractor.compute(gray_img, keypoints)

        # Do matching step and ration test to remove bad points.
        matches = self._matcher.knnMatch(new_descriptors, new_descriptors, k=2)
        good_matches = extract_good_ratio_matches(matches, max_ratio=self._max_ratio)

        # Store points and descriptors.
        ref = PlaneReference(
            self._world_image.shape[0:2],
            self._world_size,
            np.array([-0.5 * self._world_size[1], 0.5 * self._world_size[0], 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, -1., 0.0]),
        )

        self._world_points = []
        self._descriptors = []
        for match in good_matches:
            self._world_points.append(ref.pixel_to_world(keypoints[match.queryIdx].pt))
            self._descriptors.append(new_descriptors[match.queryIdx, :])

@dataclass
class PoseEstimate:
    """3D-2D pose estimation results"""
    pose_w_c : np.ndarray      # FIXME: sophus se3d
    image_inlier_points: list  # FIXME: 2D inlier points
    world_inlier_points: list  # FIXME: 3D inlier points

    def is_found(self):
        # Default identity orientation means looking away,
        # therefore using default values when no valid estimate was found.
        return False  # FIXME !pose_W_C.rotationMatrix().isIdentity(1e-8);

class PoseEstimator:
    