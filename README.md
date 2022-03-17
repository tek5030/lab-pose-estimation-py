# Lab: Pose estimation and Augmented Reality
In this lab we will implement pose estimation based on 2D-3D correspondences from scratch!

![Illustration of pose estimation from 3D-2D correspondences](lab-guide/img/pose_estimation.png)

Start by cloning this repository on your machine.
Then open the project.

The lab is carried out by following these steps:
1. [Get an overview](lab-guide/1-get-an-overview.md)
2. [Camera calibration](lab-guide/2-camera-calibration.md)
3. [Implement homography-based pose estimation](lab-guide/3-implement-homography-based-pose-estimation.md)
4. [Experiment with Motion-only Bundle Adjustment](lab-guide/4-experiment-with-motion-only-bundle-adjustment.md)

Please start the lab by going to the [first step](lab-guide/1-get-an-overview.md).

## Setup for Jetson (on the lab)

- Clone the repo into the directory `~/tek5030`
- Run the setup script `setup_jetson.bash` which 
  - creates a "venv"
  - downloads a precompiled VTK-wheel
  - installs requirements from `requirements-jetson.txt`
- Open the editor of your choice

```bash
mkdir -p ~/tek5030
cd ~/tek5030
git clone https://github.com/tek5030/lab-pose-estimation-py.git
cd lab-pose-estimation-py
./setup_jetson.bash

# source venv/bin/activate
# python lab_pose_estimation.py
```
