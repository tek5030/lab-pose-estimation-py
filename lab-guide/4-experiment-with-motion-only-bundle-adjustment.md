# Step 4: Experiment with motion-only bundle adjustment
We have implemented a pose estimator `MobaPoseEstimator` in [pose_estimators.py](../pose_estimators.py).
This estimator will let us refine an estimated pose of the camera by minimizing the reprojection error using non-linear least squares.

First, study the pose estimator and the functionality it uses from [bundle_adjustment.py](../bundle_adjustment.py).
Based on what you learned in the lectures, try to understand what is happening here.

For the optimisation to work, we need a good initial estimate.
We can use our `HomographyPoseEstimator` to compute the initial estimate like this:

```python
    init_pose_estimator = HomographyPoseEstimator(camera_model)
    pose_estimator = MobaPoseEstimator(init_pose_estimator, camera_model)
```

## Experiments
- Run `MobaPoseEstimator` with `HomographyPoseEstimator`.

  Do you see any changes to the estimated poses?
    - Notice that the number of iterations and the cost is printed to the terminal.

- Run `MobaPoseEstimator` with `PnPPoseEstimator`.

  Actually, in this situation, `HomographyPoseEstimator` works really well.
  Let's instead use the provided `PnPPoseEstimator` based only on RANSAC:
    - First, use the PnP method alone like this: `pose_estimator = PnPPoseEstimator(camera_model, do_iterative_estimation=False)`

      How well does this method work? Why?
    - Now, use `PnPPoseEstimator` to give initial estimates to `MobaPoseEstimator`.
  
      Do you see any improvement to the estimates? Why?

- Experiment with optimisation method.

  [bundle_adjustment.py](../bundle_adjustment.py) contains both the Gauss-Newton and Levenberg-Marquardt methods.
    - Which method is `MobaPoseEstimator`? What are the pros and cons with this method?
    - Try changing to the other method. Do you notice any changes?


## Extra
More time? There is more fun to be had! Here are some suggestions:

- Play around with AR by for example adding more (moving) 3D objects in `ArRenderer.__init__()`.
- We have ignored measurement uncertainty in this example.
  If you want to experiment with a more complete example, take a look at lab 5 in TTK21, which is based on the same code: 
 
  https://github.com/ttk21/lab_05

- Calibrate your camera!
