# Exercise 12: Footstep planning

Humanoid robots need to plan foot steps for getting from one location to another. In this exercise,
we will extend the A* algorithm from the previous exercise for planning foot steps.

All the methods that need to be implemented are in the file 
`src/12_footstep_planning/src/FootstepPlanning.cpp`, you don't have to change any other files.

1. Implement the cost function for moving from the current footstep to the next footstep in
`getCosts()` according to the cost function defined on slide 12.

2. Implement the Euclidean distance heuristic for the foot steps in `heuristic()`.

3. Calculate the new coordinates `(x', y', theta')` that the foot moves to when the robot executes the
given footstep action `delta_x, delta_y, delta_theta` when it is currently in `(x, y, theta)` in `executeFootstep`.

4. Implement the method `getNeighborNodes()` that should return the neighbor foot steps that
are reachable from the current foot step. The possible foot step actions are given as a parameter
to the method. You can use the method executeFootstep defined above, and the method 
`bool isColliding(footstepNode)` that is already provided by the code.

If you have Gnuplot installed, you can display an animation of the footstep sequence by running
the `plot.gp` script in the `scripts` directory.


