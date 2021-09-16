#ifndef ZMP_H_
#define ZMP_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace zmp {

typedef Eigen::Vector3d Acceleration3d;
typedef Eigen::Vector3d Force;
typedef Eigen::Vector3d Point3d;
typedef Eigen::Vector3d Normal3d;
typedef Eigen::Vector3d AngularMoment;
typedef double Mass;

/**
 * @brief Zero-Moment Point calculation.
 */
class ZMP {
public:
    Normal3d n;                      ///< normal vector of the ground plane = (0, 0, 1)
	const Acceleration3d g;          ///< acceleration of the gravity in m/s^2 = (0, 0, -9.81)
	Mass m;                          ///< total mass of the robot in kg

    Point3d C;                       ///< coordinates of the center of mass in m

    Point3d O;                       ///< origin of the contact plane coordinate system
    Force F_E;                       ///< external force
    Point3d E;                       ///< attack point of the external force

    Point3d lastCoP;                 ///< last center of pressure

	ZMP() : n(Eigen::Vector3d(0, 0, 1)),
			g(Eigen::Vector3d(0, 0, -9.81)),
			m(0.0),
			C(Point3d::Zero()),
			O(Point3d::Zero()),
			F_E(Force::Zero()),
			E(Force::Zero()),			
			lastCoP(C)
    {
	}
	virtual ~ZMP() {};

	virtual Force giForce() const;
	virtual AngularMoment giMoment(const Point3d& X) const;
	virtual Point3d zeroMomentPoint(const Point3d& O) const;

	virtual Force contactForce() const;
	virtual AngularMoment contactMoment(const Point3d& CoP, const Point3d& X) const;
	virtual AngularMoment resultingMomentAtCoM(const Point3d& CoP) const;

	/**
	 * @brief Computes the center of pressure by clipping the ZMP to the support polygon.
	 * @param[in] zmp The zero moment point.
	 * @return The center of pressure.
	 *
	 * This method assumes that all contact surfaces in the same plane.
	 */
	inline Point3d computeCoP(const Point3d& zmp) const {
		Point3d CoP = zmp;
		if (std::isnan((double) zmp[0]) || std::isnan((double) zmp[1]) || std::isnan((double) zmp[2])) {
			return lastCoP;
		}
		if (CoP[0] < -0.1) CoP[0] = -0.1;
		if (CoP[0] >  0.1) CoP[0] =  0.1;
		if (CoP[1] < -0.1) CoP[1] = -0.1;
		if (CoP[1] >  0.1) CoP[1] =  0.1;
		return CoP;

	}

};

} // namespace zmp

#endif  // ZMP_H_
