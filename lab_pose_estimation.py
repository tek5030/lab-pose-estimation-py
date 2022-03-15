import cv2
import numpy as np

from common_lab_utils import (
  CameraModel, PlaneWorldModel
)


def setup_camera_model():
    # (Using parameters for webcam)
    # Set calibration matrix K
    k = np.array([
        [6.6051081297156020e+02, 0., 3.1810845757653777e+02],
        [0., 6.6051081297156020e+02, 2.3995332228230293e+02],
        [0., 0., 1.]
    ])

    d = np.array([0., 2.2202255011309072e-01, 0., 0., -5.0348071005413975e-01])

    return CameraModel(k, d)


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

    while True:
        # Read next frame.
        success, curr_image = cap.read()

        if not success:
            print(f"The video source {video_source} stopped")
            break

        # Show the results
        cv2.imshow("window", curr_image)

        # Update the GUI and wait a short time for input from the keyboard.
        key = cv2.waitKey(1)

        # React to keyboard commands.
        if key == ord('q'):
            print("Quit")
            break


if __name__ == "__main__":
    run_pose_estimation_lab()
