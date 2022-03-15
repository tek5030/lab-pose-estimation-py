import cv2
import numpy as np
from scene_3d import Scene3D
from pylie import SE3, SO3
from common_lab_utils import (
  PerspectiveCamera, PlaneWorldModel, PoseEstimate
)


def setup_camera_model():
    # (Using parameters for webcam)
    # Set calibration matrix K
    K = np.array([
        [6.6051081297156020e+02, 0., 3.1810845757653777e+02],
        [0., 6.6051081297156020e+02, 2.3995332228230293e+02],
        [0., 0., 1.]
    ])

    d = np.array([0., 2.2202255011309072e-01, 0., 0., -5.0348071005413975e-01])

    return PerspectiveCamera(K, d)


def create_world_model():
    image_path = {
        "a4": "./world_A4.png",
        "a3": "./world_A3.png"
    }

    # Pysical world in meters.
    paper_size = {
        "a4": (0.297, 0.210),
        "a3": (0.420, 0.297)
    }

    grid_size = {
        "a4": 0.025,
        "a3": 0.040
    }

    # Choose a paper size.
    paper_format = "a4"

    # Read "world" image corresponding to the chosen paper size.
    world_image = cv2.imread(image_path[paper_format], cv2.IMREAD_UNCHANGED)

    if world_image is None:
        return None

    # Create world model.
    return PlaneWorldModel(world_image, paper_size[paper_format], grid_size[paper_format])


def run_pose_estimation_lab():
    # Get the camera model parameters.
    camera_model = setup_camera_model()

    # Construct plane world model.
    world = create_world_model()

    if not world:
        print(f"Failed to load world model")
        return

    # Construct pose estimator.

    # Construct 3D visualiser.
    scene_3d = Scene3D(world)


    # Connect to the camera.
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
        success, curr_image = cap.read()

        if not success:
            print(f"The video source {video_source} stopped")
            break

        # Undistort the frame using the camera model.
        undistorted_img = camera_model.undistort_image(curr_image)

        # Show the results
        cv2.imshow("window", undistorted_img)

        # Dummy estimate, move camera in a circle around the origin.
        dummy_theta += 0.01
        dummy_pos_world_cam = np.array([0.3 * np.cos(dummy_theta), 0.3 * np.sin(dummy_theta), .1])
        dummy_pose = PerspectiveCamera.looks_at_pose(dummy_pos_world_cam, np.zeros([1, 3]), np.array([0., 0., 1.]))
        dummy_estimate = PoseEstimate(dummy_pose, None, None)

        # Update the windows.
        do_exit = scene_3d.update(undistorted_img, dummy_estimate.pose_w_c, camera_model)
        if do_exit:
            break
        cv2.waitKey(10)


if __name__ == "__main__":
    run_pose_estimation_lab()
