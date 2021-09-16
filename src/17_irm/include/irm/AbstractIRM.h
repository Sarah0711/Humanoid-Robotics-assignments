#ifndef INCLUDE_IRM_ABSTRACTIRM_H_
#define INCLUDE_IRM_ABSTRACTIRM_H_

#include <map>
#include <vector>
#include <Eigen/Dense>

namespace irm {

/**
 * @brief Abstract class representing an inverse reachability map.
 */
class AbstractIRM {
public:
	/**
	 * @brief Joint configuration with manipulability score.
	 */
	class JointConfiguration {
	public:
		const Eigen::Vector3d jointAngles; ///< Vector of three joint angles in radians.
		const double manipulability;       ///< Manipulability score (0 = worst, 1 = best)
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		/**
		 * @brief Constructor.
		 * @param jointAngles Vector of three joint angles in radians.
		 * @param manipulability Manipulability score (0 = worst, 1 = best)
		 */
		JointConfiguration(const Eigen::Vector3d& jointAngles, const double manipulability)
		: jointAngles(jointAngles), manipulability(manipulability) {};
	};

	/**
	 * @brief Entry of a reachability map.
	 */
	class RMEntry {
	private:
		Eigen::Vector3d endeffectorPose;  ///< Endeffector pose (x, y, theta)
		const JointConfiguration * bestConfiguration;  ///< The joint configuration with the best manipulability.
		std::vector<const JointConfiguration *> configurations;  ///< List of joint configurations that lead to the endeffectorPose.

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		/**
		 * @brief Construtor.
		 * @param pose The endeffector pose.
		 */
		RMEntry(const Eigen::Vector3d& pose) : endeffectorPose(pose), bestConfiguration(NULL) {};
		~RMEntry() {
			for (std::vector<const JointConfiguration *>::iterator it = configurations.begin(); it != configurations.end(); ++it) {
				delete *it;
			}
			configurations.clear();
		}

		/**
		 * @brief Gets the endeffector pose.
		 * @return Endeffector pose.
		 */
		inline const Eigen::Vector3d& getEndeffectorPose() { return endeffectorPose; }
		/**
		 * @brief Returns a pointer to the configuration with the best manipulability so far (or NULL if no configurations have been added).
		 * @return Pointer to the best configuration if available, otherwise NULL.
		 */
		inline const JointConfiguration * getBestConfiguration() { return bestConfiguration; }
		/**
		 * @brief Get the vector of configurations added with addConfiguration().
		 * @return Vector of configurations.
		 */
		inline const std::vector<const JointConfiguration *>& getConfigurations() { return configurations; }

		/**
		 * @brief Adds a configuration and updates the bestConfiguration if necessary.
		 * @param jointAngles Vector of three joint angles in radians.
		 * @param manipulability Manipulability score (0 = worst, 1 = best)
		 * @param newEndeffectorPose Endeffector pose (x, y, theta)
		 */
		void addConfiguration(const Eigen::Vector3d& jointAngles, const double manipulability, const Eigen::Vector3d& newEndeffectorPose) {
			const JointConfiguration * el = new JointConfiguration(jointAngles, manipulability);
			configurations.push_back(el);

			if (!bestConfiguration || manipulability > bestConfiguration->manipulability) {
				bestConfiguration = el;
				endeffectorPose = newEndeffectorPose;
			}

		}
	};

	typedef std::map<size_t, RMEntry*> RMMapType;

	/**
	 * @brief Entry of an inverse reachability map.
	 */
	class IRMEntry {
	private:
		Eigen::Vector3d basePose;                                ///< Pose of the robot's base (x, y, theta)
		const JointConfiguration * bestConfiguration;            ///< The joint configuration with the best manipulability.
		std::vector<const JointConfiguration*> configurations;   ///< List of joint configrations applicable in the base pose.

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/**
		 * @brief Constructor.
		 * @param pose The pose of the robot's base.
		 */
		IRMEntry(const Eigen::Vector3d& pose) : basePose(pose), bestConfiguration(NULL) {};

		/**
		 * @brief Get the pose of the robot's base (x, y, theta).
		 * @return Base pose.
		 */
		inline const Eigen::Vector3d& getBasePose() { return basePose; }
		/**
		 * @brief Returns a pointer to the configuration with the best manipulability so far (or NULL if no configurations have been added).
		 * @return Pointer to the best configuration if available, otherwise NULL.
		 */
		inline const JointConfiguration * getBestConfiguration() { return bestConfiguration; }
		/**
		 * @brief Get the vector of configurations added with addConfiguration().
		 * @return Vector of configurations.
		 */
		inline const std::vector<const JointConfiguration *>& getConfigurations() { return configurations; }

		/**
		 * @brief Adds a configuration and updates the bestConfiguration if necessary.
		 * @param jointAngles Vector of three joint angles in radians.
		 * @param manipulability Manipulability score (0 = worst, 1 = best)
		 * @param newBasePose Base pose (x, y, theta)
		 */
		void addConfiguration(const Eigen::Vector3d& jointAngles, const double manipulability, const Eigen::Vector3d& newBasePose) {
			const JointConfiguration * const el = new JointConfiguration(jointAngles, manipulability);
			if (!bestConfiguration || manipulability > bestConfiguration->manipulability) {
				bestConfiguration = el;
				basePose = newBasePose;
			}
			configurations.push_back(el);

		}
	};
	typedef std::map<size_t, IRMEntry*> IRMMapType;

private:
	struct MapConfig {
		double min, max, res;
		size_t numCells;
	};
	std::vector<MapConfig> rmMapConfig, irmMapConfig;
	RMMapType rm, rm2d;
	IRMMapType irm2d;
	static const size_t INVALID = -1;

protected:
	/**
	 * @brief Voxel of a reachability map.
	 */
	struct RMVoxel {
		std::vector<const JointConfiguration *> configurations; ///< List of pointers to joint configurations stored in this voxel.
	};

public:
	AbstractIRM();
   	virtual ~AbstractIRM();
   	/**
   	 * @brief Get the reachability map.
   	 * @return Reachability map.
   	 */
	inline const RMMapType& getReachabilityMap() const { return rm; }
	/**
	 * @brief Get the reachability map projected to 2D (for visualization).
	 * @return 2D reachability map.
	 */
	inline const RMMapType& get2DReachabilityMap() const { return rm2d; }
	/**
	 * @brief Get the inverse reachability map projected to 2D (for visualization).
	 * @return 2D inverse reachability map.
	 */
	inline const IRMMapType& get2DInverseReachabilityMap() const { return irm2d; }
	/**
	 * @brief Get the map config of the reachability map.
	 * @return Map config.
	 */
	inline const std::vector<MapConfig>& getRMMapConfig() const { return rmMapConfig; }
	/**
	 * @brief Get the map config of the inverse reachability map.
	 * @return Map config.
	 */
	inline const std::vector<MapConfig>& getIRMMapConfig() const { return irmMapConfig; }
	/**
	 * @brief Converts the voxels of the reachability map rm to RMVoxels and calls computeIRM().
	 */
   	void allocateVoxelsAndComputeIRM();
   	/**
   	 * @brief Computes the forward kinematics.
   	 * @param[in] jointAngles The input joint angles.
   	 * @param[out] endeffectorPose The resulting endeffector pose
   	 * @return True on success
   	 */
   	bool forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose) const;
   	/**
   	 * @brief Computes the inverse kinematics.
   	 * @param[in] endeffectorPose The input endeffector pose
   	 * @param[out] jointAngles The resulting joint angles that lead to the given endeffector Pose
   	 * @return True on success
   	 */
   	bool inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles) const;

protected:
   	/**
   	 * @brief Computes the inverse reachability map.
   	 * @param[in] voxels The voxels of the reachability map.
   	 */
   	virtual void computeIRM(const std::vector<RMVoxel>& voxels) = 0;
   	/**
   	 * @brief Adds a configuration to the reachability map.
   	 * @param jointAngles Vector of three joint angles.
   	 * @param endeffectorPose Endeffector pose that can be reached using these joint angles.
   	 * @param manipulability Manipulability score (0 = worst, 1 = best).
   	 */
   	virtual void addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability);
   	/**
   	 * @brief Adds a configuration to the inverse reachability map.
   	 * @param basePose Base pose of the robot.
   	 * @param jointAngles Vector of three joint angles.
   	 * @param manipulability Manipulability score (0 = worst, 1 = best).
   	 */
   	virtual void addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability);

   	/**
   	 * @brief Create homogenous 4x4 rotation matrix for rotation around X axis
   	 * @param angle Rotation angle in radians
   	 * @return 4x4 homogenous rotation matrix
   	 */
   	Eigen::Matrix4d rotationX(const double& angle) const;

   	/**
   	 * @brief Create homogenous 4x4 rotation matrix for rotation around Y axis
   	 * @param angle Rotation angle in radians
   	 * @return 4x4 homogenous rotation matrix
   	 */
   	Eigen::Matrix4d rotationY(const double& angle) const;

   	/**
   	 * @brief Create homogenous 4x4 rotation matrix for rotation around Z axis
   	 * @param angle Rotation angle in radians
   	 * @return 4x4 homogenous rotation matrix
   	 */
	Eigen::Matrix4d rotationZ(const double& angle) const;

   	/**
   	 * @brief Create homogenous 4x4 translation matrix
   	 * @param t Translation vector
   	 * @return 4x4 homogenous translation matrix
   	 */
	Eigen::Matrix4d translation(const Eigen::Vector3d& t) const;

	std::vector<std::pair<double, double> > jointLimits;
	std::vector<double> linkLengths;

private:
   	size_t getIndex(const std::vector<MapConfig>& mapConfig, const Eigen::Vector3d& pose, const bool& use2d) const;
   	friend class FileIO;
   	friend class IRM_computeManipulability_Test;
};

}




#endif /* INCLUDE_IRM_ABSTRACTIRM_H_ */
