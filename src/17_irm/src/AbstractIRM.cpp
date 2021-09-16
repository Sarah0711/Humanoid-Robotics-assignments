#include <irm/AbstractIRM.h>
#include <angles/angles.h>

namespace irm {

void AbstractIRM::allocateVoxelsAndComputeIRM() {
	std::vector<RMVoxel> voxels;
	voxels.reserve(rm.size());
	for (RMMapType::const_iterator it = rm.begin(); it != rm.end(); ++it) {
		RMVoxel voxel;
		voxel.configurations.reserve(it->second->getConfigurations().size());
		voxel.configurations.insert(voxel.configurations.end(), it->second->getConfigurations().begin(), it->second->getConfigurations().end());
		voxels.push_back(voxel);
	}
	computeIRM(voxels);
}

bool AbstractIRM::forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose) const {
	const double jointAnglesNormalized[] = {
		angles::normalize_angle(jointAngles[0]),
		angles::normalize_angle(jointAngles[1]),
		angles::normalize_angle(jointAngles[2]),
	};

	// check joint limits
	for (size_t i = 0; i < 3; ++i) {
		if (jointAnglesNormalized[i] < jointLimits[i].first || jointAnglesNormalized[i] > jointLimits[i].second) {
			return false;
		}
	}

	Eigen::Matrix4d m =
			rotationZ(jointAngles[0]) * translation(Eigen::Vector3d(linkLengths[0], 0., 0.))
		  * rotationZ(jointAngles[1]) * translation(Eigen::Vector3d(linkLengths[1], 0., 0.))
		  * rotationZ(jointAngles[2]) * translation(Eigen::Vector3d(linkLengths[2], 0., 0.));
	const Eigen::Vector3d rpy = Eigen::Matrix3d(m.topLeftCorner(3, 3)).eulerAngles(0, 1, 2);
	endeffectorPose << m(0, 3), m(1, 3), rpy(2);
	return true;
}

void AbstractIRM::addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability) {
	size_t idx = getIndex(irmMapConfig, basePose, true);
	if (idx == INVALID) {
		return;
	}
	std::pair<IRMMapType::iterator, bool> result = irm2d.insert(std::make_pair(idx, new IRMEntry(basePose)));
	IRMEntry * entry = result.first->second;
	entry->addConfiguration(jointAngles, manipulability, basePose);
}


bool AbstractIRM::inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles) const {
	// Closed-form solution taken from:
	// V. Kumar: "Kinematics of a planar 3R manipulator"

	const double x = endeffectorPose(0);
	const double y = endeffectorPose(1);
	const double phi = endeffectorPose(2);
	const double l1 = 3.0;
	const double l2 = 2.0;
	const double l3 = 0.5;
	const double x_ = x - l3 * cos(phi);
	const double y_ = y - l3 * sin(phi);
	const double s = hypot(x_, y_);
	if (s < 1e-4)
		return false;
	const double gamma = atan2(-y_ / s, -x_ / s);
	for (size_t solution = 0; solution < 2; ++solution) {
		const double sigma = solution == 0 ? 1 : -1;
		double theta1 = gamma + sigma * acos(-(x_*x_ + y_*y_ + l1*l1 - l2*l2) / (2 * l1 * s));
		double theta2 = atan2((y_ - l1*sin(theta1)) / l2, (x_ - l1*cos(theta1)) / l2) - theta1;
		double theta3 = phi - theta1 - theta2;
		if (std::isnan(theta1) || std::isnan(theta2) || std::isnan(theta3) ||
			std::isinf(theta1) || std::isinf(theta2) || std::isinf(theta3))
			continue;
		theta1 = angles::normalize_angle(theta1);
		theta2 = angles::normalize_angle(theta2);
		theta3 = angles::normalize_angle(theta3);
		if (theta1 < jointLimits[0].first || theta1 > jointLimits[0].second ||
			theta2 < jointLimits[1].first || theta2 > jointLimits[1].second ||
			theta3 < jointLimits[2].first || theta3 > jointLimits[2].second)
			continue;
		jointAngles << theta1, theta2, theta3;
		return true;
	}
	return false;
}

void AbstractIRM::addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability) {
	std::pair<size_t, RMMapType *> c[2] = {
			std::make_pair(getIndex(rmMapConfig, endeffectorPose, false), &rm),
			std::make_pair(getIndex(rmMapConfig, endeffectorPose, true), &rm2d),
	};
	for (size_t i = 0; i < 2; ++i) {
		if (c[i].first == INVALID) {
			return;
		}
		std::pair<RMMapType::iterator, bool> result = c[i].second->insert(std::make_pair(c[i].first, new RMEntry(endeffectorPose)));
		RMEntry * entry = result.first->second;
		entry->addConfiguration(jointAngles, manipulability, endeffectorPose);
	}
}

size_t AbstractIRM::getIndex(const std::vector<MapConfig>& mapConfig, const Eigen::Vector3d& pose, const bool& use2d) const {
	size_t idx = 0;
	size_t mult = 1;
	for (size_t i = 0; i < static_cast<size_t>(use2d ? 2 : 3); ++i) {
		const double v = i < 2 ? pose[i] : angles::normalize_angle((double) pose[i]);
		const size_t j = static_cast<size_t>((v - mapConfig[i].min) / mapConfig[i].res);
		if (j > mapConfig[i].numCells) {
			return INVALID;
		}
		idx = idx * mult + j;
		mult += mapConfig[i].numCells;
	}
	return idx;
}

Eigen::Matrix4d AbstractIRM::rotationX(const double& angle) const {
	Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
	result.topLeftCorner(3, 3) = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).matrix();
	return result;
}

Eigen::Matrix4d AbstractIRM::rotationY(const double& angle) const {
	Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
	result.topLeftCorner(3, 3) = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).matrix();
	return result;

}

Eigen::Matrix4d AbstractIRM::rotationZ(const double& angle) const {
	Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
	result.topLeftCorner(3, 3) = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();
	return result;

}

Eigen::Matrix4d AbstractIRM::translation(const Eigen::Vector3d& t) const {
	Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
	result.topRightCorner(3, 1) = t;
	return result;
}

AbstractIRM::AbstractIRM() {
	jointLimits.push_back(std::make_pair(0., M_PI/2.));
	jointLimits.push_back(std::make_pair(-M_PI, M_PI));
	jointLimits.push_back(std::make_pair(-M_PI, M_PI));

	linkLengths.push_back(3.0);
	linkLengths.push_back(2.0);
	linkLengths.push_back(0.5);

	rmMapConfig.resize(3);
	rmMapConfig[0].min = -6.5;
	rmMapConfig[0].max = 6.5;
	rmMapConfig[0].res = 0.1;
	rmMapConfig[1].min = -0.5;
	rmMapConfig[1].max = 6.5;
	rmMapConfig[1].res = 0.1;
	rmMapConfig[2].min = -M_PI;
	rmMapConfig[2].max = M_PI;
	rmMapConfig[2].res = angles::from_degrees(10.0);

	irmMapConfig.resize(3);
	irmMapConfig[0].min = -6.0;
	irmMapConfig[0].max =  6.0;
	irmMapConfig[0].res =  0.1;
	irmMapConfig[1].min = -6.0;
	irmMapConfig[1].max =  6.0;
	irmMapConfig[1].res =  0.1;
	irmMapConfig[2].min = -M_PI;
	irmMapConfig[2].max =  M_PI;
	irmMapConfig[2].res = angles::from_degrees(10.0);
	for (size_t i = 0; i < 3; ++i) {
		rmMapConfig[i].numCells = static_cast<size_t>((rmMapConfig[0].max - rmMapConfig[0].min) / rmMapConfig[0].res) + 1;
		irmMapConfig[i].numCells = static_cast<size_t>((irmMapConfig[0].max - irmMapConfig[0].min) / irmMapConfig[0].res) + 1;
	}
}

AbstractIRM::~AbstractIRM() {
	for (RMMapType::iterator it = rm.begin(); it != rm.end(); ++it) {
		delete it->second;
		it->second = NULL;
	}
	rm.clear();
	for (RMMapType::iterator it = rm2d.begin(); it != rm2d.end(); ++it) {
		delete it->second;
		it->second = NULL;
	}
	rm2d.clear();
	for (IRMMapType::iterator it = irm2d.begin(); it != irm2d.end(); ++it) {
		delete it->second;
		it->second = NULL;
	}
	irm2d.clear();

}

}  // namespace irm
