import cv2
import numpy as np
from pylie import SO3, SE3
from dataclasses import dataclass


class Size:
    def __init__(self, width: float, height: float):
        self._width = width
        self._height = height

    @classmethod
    def from_numpy_shape(cls, shape):
        return cls(*shape[1::-1])

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height


def homogeneous(x):
    """Transforms Cartesian column vectors to homogeneous column vectors"""
    return np.r_[x, [np.ones(x.shape[1])]]


def hnormalized(x):
    """Transforms homogeneous column vector to Cartesian column vectors"""
    return x[:-1] / x[-1]


class PerspectiveCamera:
    """Camera model for the perspective camera"""

    def __init__(self,
                 calibration_matrix: np.ndarray,
                 distortion_coeffs: np.ndarray):
        """Constructs the camera model.

        :param calibration_matrix: The intrinsic calibration matrix.
        :param distortion_coeffs: Distortion coefficients on the form [k1, k2, p1, p2, k3].
        :param pose_world_camera: The pose of the camera in the world coordinate system.
        """
        self._calibration_matrix = calibration_matrix
        self._calibration_matrix_inv = np.linalg.inv(calibration_matrix)
        self._distortion_coeffs = distortion_coeffs

    def undistort_image(self, distorted_image):
        """Undistorts an image corresponding to the camera model.

        :param distorted_image: The original, distorted image.
        :returns: The undistorted image.
        """

        return cv2.undistort(distorted_image, self._calibration_matrix, self._distortion_coeffs)

    def pixel_to_normalised(self, point_pixel):
        """Transform a pixel coordinate to normalised coordinates

        :param point_pixel: The 2D point in the image given in pixels.
        """

        if point_pixel.ndim == 1:
            # Convert to column vector.
            point_pixel = point_pixel[:, np.newaxis]

        return self._calibration_matrix_inv @ homogeneous(point_pixel)

    @property
    def calibration_matrix(self):
        """The intrinsic calibration matrix K."""
        return self._calibration_matrix

    @property
    def calibration_matrix_inv(self):
        """The inverse calibration matrix K^{-1}."""
        return self._calibration_matrix_inv

    @property
    def distortion_coeffs(self):
        """The distortion coefficients on the form [k1, k2, p1, p2, k3]."""
        return self._distortion_coeffs

    @property
    def principal_point(self):
        return self._calibration_matrix[0, 2], self._calibration_matrix[1, 2]

    @property
    def focal_lengths(self):
        return self._calibration_matrix[0, 0], self._calibration_matrix[1, 1]

    @staticmethod
    def looks_at_pose(camera_pos_w: np.ndarray, target_pos_w: np.ndarray, up_vector_w: np.ndarray):
        cam_to_target_w = target_pos_w - camera_pos_w
        cam_z_w = cam_to_target_w.flatten() / np.linalg.norm(cam_to_target_w)

        cam_to_right_w = np.cross(-up_vector_w.flatten(), cam_z_w)
        cam_x_w = cam_to_right_w / np.linalg.norm(cam_to_target_w)

        cam_y_w = np.cross(cam_z_w, cam_x_w)

        return SE3((SO3(np.vstack((cam_x_w, cam_y_w, cam_z_w)).T), camera_pos_w))


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
            image_size: Size,
            scene_size: Size,
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

        self._units_per_pixel_x = scene_size.width / image_size.width
        self._units_per_pixel_y = scene_size.height / image_size.height

    def pixel_to_world(self, pixel):
        pixel = np.atleast_2d(pixel)
        return \
            self._origin \
            + np.einsum('i,j->ij', pixel[:, 0], self._x_dir) \
            + np.einsum('i,j->ij', pixel[:, 1], self._y_dir)


class PlaneWorldModel:
    """Represents a planar world."""

    def __init__(self, world_image: np.array, world_size: Size, grid_length: float):
        """
        Constructs the world model.

        :param world_image: The world map image.
        :param world_size: The physical size of the world corresponding to the image in meters.
        :param grid_length: Length of the grid cells in the world image in meters.
        """
        self._world_image = world_image
        self._world_size = world_size
        self._grid_length = grid_length
        self._max_num_points = 1000
        self._max_ratio = 0.8

        self._construct_world()

    @property
    def world_image(self):
        return self._world_image

    @property
    def world_size(self):
        return self._world_size

    @property
    def grid_length(self):
        return self._grid_length

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

        # Store points and descriptors.
        ref = PlaneReference(
            Size(*self._world_image.shape[1::-1]),
            self._world_size,
            np.array([-0.5 * self._world_size.width, 0.5 * self._world_size.height, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, -1., 0.0]),
        )

        pixels = np.array([k.pt for k in keypoints])
        self._world_points = ref.pixel_to_world(pixels)
        self._descriptors = new_descriptors

    def find_correspondences(self, frame: np.ndarray):
        # Detect keypoints
        frame_keypoints = self._detector.detect(frame)

        # Compute descriptors for each keypoint.
        frame_keypoints, frame_descriptors = self._desc_extractor.compute(frame, frame_keypoints)

        # fixme: use mask to extract points?
        # Do matching step and ratio test to remove bad points.
        matches = self._matcher.knnMatch(frame_descriptors, self._descriptors, k=2)
        good_matches = extract_good_ratio_matches(matches, max_ratio=self._max_ratio)

        frame_idx = [m.queryIdx for m in good_matches]
        world_descriptor_idx = [m.trainIdx for m in good_matches]

        # # Extract good 2d-3d matches.
        image_points = np.array([k.pt for k in np.asarray(frame_keypoints)[frame_idx]])
        world_points = self._world_points[world_descriptor_idx]

        return image_points, world_points


@dataclass
class PoseEstimate:
    """3D-2D pose estimation results"""
    pose_w_c : SE3
    image_inlier_points: list  # FIXME: 2D inlier points
    world_inlier_points: list  # FIXME: 3D inlier points

    def is_found(self):
        # Default identity orientation means looking away,
        # therefore using default values when no valid estimate was found.
        return False  # FIXME !pose_W_C.rotationMatrix().isIdentity(1e-8);

# class PoseEstimator:
#