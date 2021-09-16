# Exercise 14: Zero moment point

A robot is standing upright. Now a human exerts a force on the robot's 
chest, pushing the robot backwards (see figure on the exercise sheet). 
How strong can the human push the robot before it starts tiping over 
backwards? 

In this exercise, you will calculate the involved forces and torques and 
use the Zero Moment Point (ZMP) to estimate whether the robot will fall.

Implement the methods in `src/ZMP.cpp` with the help of the explanations 
below and the symbol table on the exercise sheet.

In our scenario, four forces act on the robot: 
 * The gravitational force `F^G` acts on the center of mass `C` and forces 
   the robot to the ground.
 * The user exerts an external force `F^E` acting on point `E` located on 
   the robot's chest.
 * The ground reacts with a pressure force that prevents the robot from 
   sinking into the ground. The pressure force acts on the whole contact 
   area, but it can be substituted by a single force `F^P` acting at the 
   center of pressure `P`.
 * The friction force prevents the robot from sliding backwards and it acts
   on the whole contact area. It can again be substituted by a single force 
   `F^F` acting on the center of pressure `P`. We assume that the friction 
   is high enough so that the robot never slides.
   
We assume that the robot is standing still, so according to Newton's first
law the forces sum up to zero:
```
    F^G + F^E + F^P + F^F = 0
```
1. Calculate `F^gi = F^G + F^E` in the method `giForces()`.
2. Calculate `F^C  = F^P + F^F` in the method `contactForce()` using `F^gi` 
   the equation above.
   
Forces acting on a rigid body induce *angular moments*, also called *torques*
(not to be confused with angular *momentum*). In the figure on the exercise
sheet, for example, a force `F` acting on a wrench induces a torque `M = r × F`
on the screw, where `r` is the lever arm, `×` is the cross product, and F is 
the force acting on the wrench.

The forces acting on the robot induce different angular moments in each
point `X` of the robot. For example, the gravity force `F^G` induces a 
moment `M^G = (C-X) × F^G` as the gravity acts on the center of mass.

According to Newton's first law, the moments in every point `X` also sum up 
to zero if the robot is standing still:
```
    M^G(X) + M^E(X) + M^P(X) + M^F(X) = 0
```

3. Calculate `M^gi` in `giMoments()`.

We would like to determine whether the robot tips over or not, so we are
only interested in the horizontal components of the moment `n × M`.

The zero moment point is defined as the point `Z` where the horizontal 
components of both the gravity-inertial moments `M^gi` and the contact 
moments `M^C` are zero:
```
    n × M^gi(Z) = n × M^C(Z) = 0
```

Given an arbitrary origin `O` of the ground plane coordinate system, the 
point `Z` that fullfills above equation can be computed using
```
    (Z-O) = (n × M^gi(Z)) / (F^gi ⋅ n)
```
This equation results from the general torque formula `M = r × F` solved
for `r` and projected onto the contact plane.

4. Calculate the zero moment point in `zeroMomentPoint()` using above equation.

As long as the zero-moment point is within the support polygon, it conincides
with the center of pressure `P` where the ground reaction force acts on the robot.
When the (virtual) zero-moment point leaves the support polygon, the center
of pressure cannot follow, as pressure can only occur where the robot touches
the ground.

5. Compute the ground reaction moment `M^C(X)` relative to point `X` in the method 
`contactMoment()`. The location of the center of pressure `P` and the point 
`X` are given as arguments to the method.
6. Compute the resulting angular moment at the center of mass `M^gi(C) + M^C(C)`
in `resultingMomentAtCoM()`.

According to Newton's first law, the robot will stay upright if the angular
moment at the center of mass is zero and it will tip over if the
angular moment is non-zero.

The Gnuplot script in `scripts/plot.gp` and the animation in the Wiki show 
the distribution of moments and pressure as the external force increases.
