#include <zmp/ZMP.h>
#include <iostream>

namespace zmp {

/**
 * @brief Computes the gravity-inertial forces assuming that the robot does not move.
 * @return The force vector.
 */
Force ZMP::giForce() const {
	Force F = Eigen::Vector3d::Zero();
	/* TODO: Calculate the sum of the forces acting on the robot (without the ground reaction force) consisting of:
	 * - the gravitational force
	 * - the external force
	 */
	return F;
}

/**
 * @brief Return the contact force that the ground exerts on the robot.
 * @return The ground reaction force vector.
 *
 * We assume that the ground friction is high enough so that the robot does not slide on the ground.
 */
Force ZMP::contactForce() const {
	Force reactionForce = Eigen::Vector3d::Zero();

	// TODO: Calculate the ground reaction force.

	return reactionForce;
}

/**
 * @brief Computes the gravity-inertial angular moment relative to a given point assuming that the robot does not move.
 * @param[in] X An arbitrary point for which to compute the angular moment.
 * @return The angular moment vector.
 */
AngularMoment ZMP::giMoment(const Point3d& X) const {
	AngularMoment M = Eigen::Vector3d::Zero();

	/* TODO: Compute the sum of the angular moments acting on the robot in point X
	 * (without ground reaction moments) consisting of:
	 * - torque induced by the gravity acting on the center of mass
	 * - torque induced by the external force acting on point P
	 */

	return M;
}

/**
 * @brief Calculates the zero moment point.
 * @param[in] O Origin of the contact plane coordinate system.
 * @return Zero moment point
 */
Point3d ZMP::zeroMomentPoint(const Point3d& O) const {
	Point3d zmp = Eigen::Vector3d::Zero();

	/* TODO Compute the zero moment point.
	 * Use the methods giForce() and giMoment() that you implemented above.
	 */


	return zmp;
}

/**
 * @brief Calculate the angular moment induced by the ground reaction force.
 * @param[in] CoP The center of pressure where the ground reaction force acts on the robot.
 * @param[in] X An arbitrary point for which to compute the angular moment.
 * @return The angular moment vector.
 */
AngularMoment ZMP::contactMoment(const Point3d& CoP, const Point3d& X) const {
	AngularMoment reactionMoment = Eigen::Vector3d::Zero();
	/* TODO: Calculate the ground reaction moment that the contact force exerts on
	 * the robot in the center of pressure.
	 *
	 * Use the method contactForce() that you implemented above.
	 */

	return reactionMoment;
}

/**
 * @brief Computes the resulting angular moment acting on the center of mass.
 * @param[in] CoP The center of pressure where the contact force acts on the robot.
 * @return The angular moment = torque that acts on the robot's center of mass.
 *
 * If the result is non-zero, then the robot will accelerate on an arc trajectory.
 */
AngularMoment ZMP::resultingMomentAtCoM(const Point3d& CoP) const {
	AngularMoment M = AngularMoment::Zero();

	/* TODO: Calculate the resulting angular moment at the center of mass (X = C)
	 * that accelerates the robot on a circular trajectory if it is non-zero.
	 *
	 * Ingredients:
	 * - torque induced by the gravity acting on the center of mass
	 * - torque induced by the external force acting on point P
	 * - torque induced by the contact force acting on the center of pressure
	 *
	 * You can use all methods defined above.
	 */


	return M;
}

} // namespace zmp
