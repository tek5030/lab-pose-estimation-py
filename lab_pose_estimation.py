import cv2
import numpy as np
import timeit
from scene_3d import Scene3D
from pylie import SE3, SO3
from common_lab_utils import (
  PerspectiveCamera, PlaneWorldModel, PoseEstimate, Size
)


def run_pose_estimation_lab():
    # TODO 1: Calibrate camera and set parameters in setupCameraModel().
    # Get the camera model parameters.
    camera_model = setup_camera_model()

    # Construct plane world model.
    world = create_world_model()

    # TODO 2-6: Implement HomographyPoseEstimator.
    # TODO 7: Implement MobaPoseEstimator by finishing CameraProjectionMeasurement.
    # Construct pose estimator.
    pose_estimator = None # fixme

    # Construct AR visualizer.
    ar_example = None # fixme

    # Construct 3D visualiser.
    scene_3d = Scene3D(world)

    # Setup camera stream.
    video_source = 0
    cap = cv2.VideoCapture(video_source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"Could not open video source {video_source}")
        return
    else:
        print(f"Successfully opened video source {video_source}")

    # For dummy vis before Ragnar takes it to the next level!
    dummy_theta = 0.
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
        image_points, world_points = world.find_correspondences(gray_frame)
        end = timeit.default_timer()
        correspondence_matching_duration = end - start

        # Update the pose estimate.
        # Measure how long the processing takes.
        start = timeit.default_timer()
        # fixme: pose_estimate = pose_estimator.estimate(matched_image_points, matched_world_points)
        end = timeit.default_timer()
        pose_estimation_duration = end - start

        # Update Augmented Reality visualization.
        # fixme: ar_example.update(undistorted_frame, pose_estimate, camera_model.K, correspondence_matching_duration, pose_estimation_duration)

        # fixme: Temp. dummy estimate, move camera in a circle around the origin.
        dummy_theta += 0.01
        dummy_pos_world_cam = np.array([0.3 * np.cos(dummy_theta), 0.3 * np.sin(dummy_theta), .1])
        dummy_pose = PerspectiveCamera.looks_at_pose(dummy_pos_world_cam, np.zeros([3]), np.array([0., 0., 1.]))
        dummy_estimate = PoseEstimate(dummy_pose, None, None)
        cv2.imshow("window", undistorted_frame)

        # Update the windows.
        do_exit = scene_3d.update(undistorted_frame, dummy_estimate.pose_w_c, camera_model)
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

    return PerspectiveCamera(K, dist_coeffs)


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

if __name__ == "__main__":
    run_pose_estimation_lab()
