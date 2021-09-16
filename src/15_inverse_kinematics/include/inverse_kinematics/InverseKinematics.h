#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <stdexcept>

namespace inverse_kinematics {

typedef Eigen::VectorXd EndeffectorPose;
typedef Eigen::Matrix3d Cartesian2DTransform;
typedef Eigen::VectorXd JointAngles;
typedef Eigen::MatrixXd Jacobian;


Eigen::MatrixXd pseudoInverse(const Jacobian &a);

/**
 * @brief Abstract base class for computing inverse kinematics.
 *
 * Subclasses must implement the pure virtual functions jacobian(const JointAngles&) and forwardKinematic(const JointAngles&).
 */
class InverseKinematics
{
public:
	/**
	 * \brief Constuctor.
	 * @param alpha step size for chooseStep()
	 */
	InverseKinematics(const double& alpha) : alpha(alpha) {};
   	virtual ~InverseKinematics() {};

   	virtual Cartesian2DTransform rotation(const double& angle) const;
   	virtual Cartesian2DTransform translation(const double& distance_x, const double& distance_y) const;
   	virtual EndeffectorPose chooseStep(const EndeffectorPose& e, const EndeffectorPose& g) const;
   	virtual JointAngles computeJointChange(const EndeffectorPose& delta_e, const Jacobian& jacobian) const;
   	virtual JointAngles applyChangeToJoints(const JointAngles& q, const JointAngles& delta_q) const;
   	virtual void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const;

   	/**
   	 * \brief Computes the Jacobian of the endeffector pose with respect to the joint angles.
   	 * \param[in] q The current joint angles as an Eigen::VectorXd of length n where n is the number of links.
   	 * \return The Jacobian as an Eigen::MatrixXd of size n x n.
   	 */
   	virtual Jacobian jacobian(const JointAngles& q) const = 0;

   	/**
   	 * \brief Computes the forward kinematics for a robot arm with n links.
   	 * \param[in] q The current joint angles as an Eigen::VectorXd of size n.
   	 * \return The endeffector pose e as an Eigen::VectorXd of size n.
   	 */
   	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const = 0;

   	const double alpha;  ///< Step size for chooseStep().
};

/**
 * @brief Inverse kinematics for a robot with 2 links.
 *
 * This class overloads InverseKinematics for convenience.
 */
class InverseKinematics_2Links : public InverseKinematics {
public:
	/**
	 * @brief Constructor.
	 * @param a0 length of link 0 in meters.
	 * @param a1 length of link 1 in meters.
	 * @param h  length of the gripper in meters.
	 * @param alpha step size of chooseStep().
	 */
	InverseKinematics_2Links(const double& a0, const double& a1, const double& h, const double& alpha) : InverseKinematics(alpha), a0(a0), a1(a1), h(h) {};
	virtual ~InverseKinematics_2Links() {};

	virtual JointAngles computeIK(const EndeffectorPose& g, const double& maxTranslationalError) const;
	virtual Jacobian jacobian(const JointAngles& q) const;
	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const;

   	const double a0;  ///< Length of link 0 in meters.
   	const double a1;  ///< Length of link 1 in meters.
   	const double h;   ///< Length of the gripper in meters.
};

/**
 * @brief Inverse kinematics for a robot with 3 links.
 *
 * This class overloads InverseKinematics for convenience.
 */
class InverseKinematics_3Links : public InverseKinematics {
public:
	/**
	 * @brief Constructor.
	 * @param a0 length of link 0 in meters.
	 * @param a1 length of link 1 in meters.
	 * @param a2 length of link 2 in meters.
	 * @param h  length of the gripper in meters.
	 * @param alpha step size of chooseStep().
	 */
	InverseKinematics_3Links(const double& a0, const double& a1, const double& a2, const double& h, const double& alpha) : InverseKinematics(alpha), a0(a0), a1(a1), a2(a2), h(h) {};
	virtual ~InverseKinematics_3Links() {};

	virtual JointAngles computeIK(const EndeffectorPose& g, const double& maxTranslationalError, const double& maxAngularError) const;
	virtual Jacobian jacobian(const JointAngles& q) const;
	virtual EndeffectorPose forwardKinematic(const JointAngles& q) const;

   	const double a0;  ///< Length of link 0 in meters.
   	const double a1;  ///< Length of link 1 in meters.
   	const double a2;  ///< Length of link 2 in meters.
   	const double h;   ///< Length of the gripper in meters.
};

}  // namespace inverse_kinematics

#endif  // INVERSE_KINEMATICS_H_
