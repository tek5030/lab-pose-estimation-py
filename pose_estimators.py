import numpy as np
import cv2
from pylie import SO3, SE3
from dataclasses import dataclass


@dataclass
class PoseEstimate:
    """3D-2D pose estimation results"""
    pose_w_c : SE3 = None
    image_inlier_points: np.ndarray = None
    world_inlier_points: np.ndarray = None

    def is_found(self):
        return self.pose_w_c is not None


class PnPPoseEstimator:
    def __init__(self, calibration_matrix: np.ndarray, do_iterative_estimation=False):
        self._calibration_matrix = calibration_matrix
        self._do_iterative_estimation = do_iterative_estimation

    def estimate(self, image_points, world_points):
        """Estimate pose from the 2d-3d correspondences using PnP"""

        # Check that we have a minimum required number of points, here 3 times the theoretic minimum.
        min_number_points = 9
        if len(image_points) < min_number_points:
            return PoseEstimate()

        # Find inliers and compute initial pose with RANSAC.
        retval, rvec, tvec, inliers = cv2.solvePnPRansac(world_points, image_points, self._calibration_matrix, (),
                                                         useExtrinsicGuess=False, iterationsCount=10000,
                                                         reprojectionError=2.0, confidence=0.99,
                                                         flags=cv2.SOLVEPNP_AP3P)

        # Check that we have enough inliers.
        if not retval or len(inliers) < min_number_points:
            return PoseEstimate()

        # Extract inliers.
        inliers = inliers.ravel()
        inlier_image_points = image_points[inliers]
        inlier_world_points = world_points[inliers]

        # Compute the camera pose with an iterative method using the entire inlier set.
        # Use "cv::solvePnP" on inlier points to improve "r_vec" and "t_vec".
        # Use the iterative method with current r_vec and t_vec as initial values.
        if self._do_iterative_estimation:
            rvec, tvec = cv2.solvePnPRefineLM(inlier_world_points, inlier_image_points,
                                              self._calibration_matrix, (), rvec, tvec)

        # We now have the pose of the world in the camera frame!
        pose_c_w = SE3((SO3.Exp(rvec), tvec))

        # Return the pose of the camera in the world frame.
        return PoseEstimate(pose_c_w.inverse(), inlier_image_points, inlier_world_points)
