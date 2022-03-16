import cv2
import numpy as np
import timeit
from visualisation import Scene3D, ARRenderer, print_info_in_image
from pylie import SE3, SO3
from common_lab_utils import (
  PerspectiveCamera, PlaneWorldModel, Size
)
from pose_estimators import (PoseEstimate, PnPPoseEstimator)


def run_pose_estimation_lab():
    # TODO 1: Calibrate camera and set parameters in setupCameraModel().
    # Get the camera model parameters.
    camera_model = setup_camera_model()

    # Construct plane world model.
    world_model = create_world_model()

    # TODO 2-6: Implement HomographyPoseEstimator.
    # TODO 7: Implement MobaPoseEstimator by finishing CameraProjectionMeasurement.
    # Construct pose estimator.
    # pose_estimator = PnPPoseEstimator(camera_model.calibration_matrix, True)
    pose_estimator = HomographyPoseEstimator(camera_model.calibration_matrix)

    # Construct AR visualizer.
    ar_example = ARRenderer(world_model, camera_model)

    # Construct 3D visualiser.
    scene_3d = Scene3D(world_model, camera_model)

    # Setup camera stream.
    video_source = 0
    cap = cv2.VideoCapture(video_source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_model.image_size.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_model.image_size.height)

    if not cap.isOpened():
        print(f"Could not open video source {video_source}")
        return
    else:
        print(f"Successfully opened video source {video_source}")

    while True:
        # Read next frame.
        success, curr_frame = cap.read()

        if not success:
            print(f"The video source {video_source} stopped")
            break

        # Undistort the frame using the camera model.
        undistorted_frame = camera_model.undistort_image(curr_frame)

        # Convert to gray scale for correspondence matching.
        gray_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

        # Find the correspondences between the detected image points and the world points.
        # Measure how long the processing takes.
        start = timeit.default_timer()
        image_points, world_points = world_model.find_correspondences(gray_frame)
        end = timeit.default_timer()
        matching_duration_ms = (end - start) * 1000.0

        # Update the pose estimate.
        # Measure how long the processing takes.
        start = timeit.default_timer()
        estimate = pose_estimator.estimate(image_points, world_points)
        end = timeit.default_timer()
        pose_estimation_duration_ms = (end - start) * 1000.0

        # Update Augmented Reality visualization.
        ar_frame = undistorted_frame.copy()
        ar_rendering, mask = ar_example.update(estimate)
        if ar_rendering is not None:
            ar_frame[mask] = ar_rendering[mask]
        print_info_in_image(ar_frame, estimate, matching_duration_ms, pose_estimation_duration_ms, show_inliers=True)
        cv2.imshow("AR visualisation", ar_frame)

        # Update the windows.
        do_exit = scene_3d.update(undistorted_frame, estimate)
        if do_exit:
            break
        cv2.waitKey(10)


def setup_camera_model():
    """Constructs the camera model according to the results from camera calibration"""

    # TODO 1: Set K according to calibration.
    # Set calibration matrix K
    K = np.array([
        [6.6051081297156020e+02, 0., 3.1810845757653777e+02],
        [0., 6.6051081297156020e+02, 2.3995332228230293e+02],
        [0., 0., 1.]
    ])

    # TODO 1: Set dist_coeffs according to the calibration.
    dist_coeffs = np.array([0., 2.2202255011309072e-01, 0., 0., -5.0348071005413975e-01])

    # TODO 1: Set the image size corresponding to the calibration
    image_size = Size(640, 480)

    return PerspectiveCamera(K, dist_coeffs, image_size)


def create_world_model():
    """Sets up the world according to which version you have printed, either A4 or A3"""

    # World image corresponding to the chosen paper size.
    image_name = {
        "a4": "./world_A4.png",
        "a3": "./world_A3.png"
    }

    # Physical world sizes in meters.
    paper_size = {
        "a4": Size(0.297, 0.210),
        "a3": Size(0.420, 0.297)
    }

    # Grid length in meters.
    # This will be the physical size of axes in the visualization.
    grid_length = {
        "a4": 0.025,
        "a3": 0.040
    }

    # Choose a paper size according to what you are using in the lab.
    paper_format = "a3"

    # Read "world" image corresponding to the chosen paper size.
    world_image = cv2.imread(image_name[paper_format], cv2.IMREAD_UNCHANGED)

    if world_image is None:
        raise RuntimeError("Could not read world image")

    # Create world model.
    return PlaneWorldModel(world_image, paper_size[paper_format], grid_length[paper_format])


class HomographyPoseEstimator:
    def __init__(self, calibration_matrix: np.ndarray):
        self._calibration_matrix_inv = np.linalg.inv(calibration_matrix)

    def estimate(self, image_points, world_points):
        """Estimate pose from the homography computed from 2d-3d (planar) correspondences"""

        # Check that we have a minimum required number of points, here 3 times the theoretic minimum.
        min_number_points = 12
        if len(image_points) < min_number_points:
            return PoseEstimate()

        # Compute the homography.
        H, inlier_mask = cv2.findHomography(world_points, image_points, cv2.RANSAC, 3)
        inliers = inlier_mask.ravel() > 0

        # Check that we have a valid result and enough inliers.
        if H is None or inliers.sum() < min_number_points:
            return PoseEstimate()

        # Extract inliers.
        inlier_image_points = image_points[inliers]
        inlier_world_points = world_points[inliers]

        # Compute the matrix M and extract M_bar (the two first columns of M).
        M = self._calibration_matrix_inv @ H
        M_bar = M[:, :2]

        # Perform SVD on M_bar.
        try:
            U, _, Vh = np.linalg.svd(M_bar, full_matrices=False)
        except np.linalg.LinAlgError:
            print("Warning: SVD computation did not converge")
            return PoseEstimate()

        # Compute R_bar (the two first columns of R) from the result of the SVD.
        R_bar = U @ Vh

        # Construct R by inserting R_bar and computing the third column of R from the two first.
        # Remember to check det(R)!
        R = np.c_[R_bar, np.cross(R_bar[:, 0], R_bar[:, 1])]

        if np.linalg.det(R) < 0:
            R[:, 2] *= -1.

        # Compute the scale factor.
        scale = (R_bar * M_bar).sum() / (M_bar**2).sum()

        # Extract the translation t.
        t = M[:, 2] * scale

        # Check that this is the correct solution by testing the last element of t.
        if t[-1] < 0:
            # Switch to other solution.
            t = -t
            R[:, :2] *= -1.

        # We now have the pose of the world in the camera frame!
        pose_c_w = SE3((SO3(R), t))

        # Return the pose of the camera in the world frame.
        return PoseEstimate(pose_c_w.inverse(), inlier_image_points, inlier_world_points)


if __name__ == "__main__":
    run_pose_estimation_lab()
