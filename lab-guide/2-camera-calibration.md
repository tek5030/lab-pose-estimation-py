# Step 2: Camera calibration
To estimate the pose of the camera using map-image correspondences, we will first need to specify the camera calibration parameters.

## 1. Specify the camera calibration
Fill the correct calibration matrix and distortion parameters into `setup_camera_model()` in [lab_pose_estimation.py](../lab_pose_estimation.py).

- If you are using the lab webcams, you can use the calibration results in [cameraParameters.xml](../cameraParameters.xml).
- If you are using your own camera, [you need to calibrate it](https://docs.opencv.org/4.5.5/d7/d21/tutorial_interactive_calibration.html).
  
  You might want to take a look at Thomas' tutorial on canvas.

Then, please continue to the [next step](3-implement-homography-based-pose-estimation.md).
