# Exercise 4: Projective geometry

You install a surveillance camera on a flag pole in front of the building.
The camera's data sheet specifies the following parameters:

 * principal point `x_h = [400,300]` pixels
 * camera constant `c=550` pixels
 * sheer `s=0`
 * scale difference `m=0.0025`
 
 
Let the origin of the world coordinate system be at the bottom of the flag
pole. The projection center of the camera is located `Z_0 = 10m` above
the ground and `X_0 = 40cm` in front of the flag pole in direction.

The camera can be rotated vertically in the `(Z,X)` plane around its 
projection center. Let `alpha` be the current rotation angle in radians. The
rotation matrix is then:
```
R = R_2(alpha) =   |cos(alpha)     0   sin(alpha)|
                `  |    0          1       0     |
                `  |-sin(alpha)    0   cos(alpha)|
```

**Exercise steps:**

1. Implement the function `euclideanToHomogeneous` that converts
Euclidean coordinates in 3D to homogeneous coordinates.
2. Implement the function `homogeneousToEuclidean` that converts
the homogeneous coordinates back to Euclidean coordinates. You may assume
that the last component of the homogeneous coordinates is non-zero.
3. Implement the function `setCameraParameters` that returns a data structure
that contains the camera parameters.
4. Implement the function `calibrationMatrix` that returns the
calibration matrix `K` for the camera parameters given as an argument.
5. Implement the function `projectionMatrix` that returns the
projection matrix `P` given the calibration matrix and the rotation angle
`alpha`.
6. Implement the function `projectPoint` that projects a
point in 3D coordinates to image coordinates given the projection matrix `P`.


**Reprojection from image to 3D**

In the lecture, we discussed how to project a given point in 3D space onto a camera image. Many
robotics applications require the reverse: The robot detects a feature in the camera image, for
example an object that the robot should grasp. It then has to compute the corresponding point in
3D space so that it can move its hand there to grasp the object. Unfortunately, projecting from 3D
to the 2D image plane removes one dimension, so reconstructing the 3D point is only possible if an
additional piece of information is available, for example the size of the object or a second image
from a slightly different perspective.

In the following example, the Nao robot observes a drawing on a table and it is supposed to compute
the 3D coordinates of the corners of the house in the drawing. The robot knows that the height of
the table is h, so all points of the drawing will have Z = h in world coordinates. By exploiting this
piece of information, it is possible to reconstruct the full 3D world coordinates of any point on the
drawing.

7. Implement the function reprojectTo3D. Given an input point in image coordinates, the
camera parameters, and the table height in meters, the method should calculate and return
2the corresponding 3D point. See the exercise sheet for hints on how to compute the reprojection.


