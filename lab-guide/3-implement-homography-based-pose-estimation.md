# Step 3: Implement homography-based pose estimation
We will in this part follow [the slides from this week's lecture](https://www.uio.no/studier/emner/matnat/its/TEK5030/v20/forelesninger/lecture_6_1_pose-estimation.pdf) to finish the class `HomographyPoseEstimator`.

You will find `HomographyPoseEstimator` in [lab_pose_estimation.py](../lab_pose_estimation.py)
Read through the code to get an overview. 
Study `HomographyPoseEstimator.estimate()`, and try to understand what is happening here.

Your task is now to implement homography-based pose estimation by performing the necessary computations given in the lecture.

![Illustration of the homography-based pose estimation method](img/homography_pose_estimator.png)

## 2. Construct the world
The world model is constructed from the aerial photo in `create_world_model()` in [lab_pose_estimation.py](../lab_pose_estimation.py)
It is currently set up to construct a world based on the A3 printout. 
If you are using A4 instead, please fix this here.

Else, continue to the next task.

## 3. Compute the **M** matrix
Compute the matrix

![\mathbf{M} = \mathbf{K} ^{-1} \mathbf{H}_{i\Pi}](img/math_formula-for-M.png)

## 4. Compute the first two columns of the **R** matrix
Compute the matrix

![\mathbf{\bar R}=\mathbf{UV}^T](img/math_formula-for-R_bar.png)

You can read more about the SVD implementation here:   
https://numpy.org/doc/stable/reference/generated/numpy.linalg.svd.html

## 5. Construct the complete **R** matrix
Complete the **R** matrix by computing the last column, taking the cross product between the first two.
Make sure that det(**R**) = 1.

## 6. Compute the scale *&lambda;*
Compute the scale according to

![\lambda = \frac{trace(\mathbf{\bar R}^{\ast T}\mathbf{\bar M})}{trace(\mathbf{\bar M}^{ T}\mathbf{\bar M})}
  = \frac{\sum_{i=1}^{3}\sum_{j=1}^{2} r^{\ast}_{ij}m_{ij}}{\sum_{i=1}^{3}\sum_{j=1}^{2} m^2_{ij}}](img/math_scale.png)
  
## 7. Find the correct solution
Use *&lambda;* and **M** to compute the translation up to the correct sign.

- We can check if we have the correct solution by testing the last element of **t**.
  - How?

- Choose the other solution if this check fails.

Run and test the pose estimator!

Continue to the [next step](4-experiment-with-motion-only-bundle-adjustment.md) when you are finished testing.
