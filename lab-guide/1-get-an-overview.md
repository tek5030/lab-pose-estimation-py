# Step 1: Get an overview
We will start by presenting an overview of the lab and the contents of this project.

## Lab overview
We are given this 2D world map:

![World map](img/world_map.png)

The main steps in today's lab are:
- Calibrate the camera (to save time, we will use calibration results computed in advance).
- Create a planar 3D world model from the map with point descriptors.
- Estimate the camera pose from 2D-3D correspondences:
  - Homography-based
  - Motion-only Bundle Adjustment (moba)
- Visualize the 3D world frame in the camera views with [Augmented Reality (AR)](https://en.wikipedia.org/wiki/Augmented_reality).
- Visualize the camera and world model in 3D.

## Introduction to the project source files
We have chosen to distribute the code on the following modules:

- [**lab_pose_estimation.py**](../lab_pose_estimation.py)

  Contains the main loop of the program and all exercises. 
  Your main task will be to finish `HomographyPoseEstimator` in this module.
  Please read quickly through the function `run_pose_estimation_lab()` to get a feel of how the lab works.

- [**common_lab_utils.py**](../common_lab_utils.py)

  This module contains utility functions and classes that we will use both in the lab and in the solution.
  Please take a quick look through the code.

- [**visualisation.py**](../visualisation.py)  

  Contains functionality for visualising in 3D and rendering for AR.

- [**bundle_adjustment.py**](../bundle_adjustment.py)

  Contains functionality for solving bundle adjustment problems, 
  including implementations of the Gauss-Newton and Levenberg-Marquardt methods. 

- [**pose_estimators.py**](../pose_estimators.py)

  Contains pose estimators that have already been implemented (`PnPPoseEstimator` and `MobaPoseEstimator`)

- [**solution_pose_estimation.py**](../solution_pose_estimation.py)

  This is our proposed solution to the lab.
  Please try to solve the lab with help from others instead of just jumping straight to the solution ;)

Other files we will use:
- [**calibSettings.xml**](../calibSettings.xml)
  
  Parameter file for calibrating a camera (can be used when calibrating your own camera).

- [**cameraParameters.xml**](../cameraParameters.xml)   

  Camera calibration result that should work ok for the cameras in the lab.

- [**world_A3.png**](../world_A3.png)

  Planar world for printouts on A3 paper (we will use this in the lab).

- [**world_A4.png**](../world_A4.png)

  Planar world for printouts on A4 paper.

Please continue to the [next step](2-camera-calibration.md) to get started!
