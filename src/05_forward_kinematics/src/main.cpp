#include <windows-helpers.h>
#include <forward_kinematics/ForwardKinematics.h>
#include <forward_kinematics/FileIO.h>
#include <iostream>

using namespace forward_kinematics;

int main(int /* argc */, char ** /*argv*/) {
	ForwardKinematics fk;
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fio(packagePath + "/data/joints.txt");
	fio.results.reserve(fio.jointAngles.size());
	for (std::vector<std::vector<double> >::const_iterator it = fio.jointAngles.begin(); it != fio.jointAngles.end(); ++it) {
		 fio.results.push_back(fk.computeHandTransform(&(*it)[0]));
	}
	const std::string outputFile = packagePath + "/data/result.txt";
	if (fio.writeToFile(outputFile)) {
		std::cout << "Wrote " << fio.results.size() << " points to " << outputFile << std::endl;
	}

    wait();
	return 0;
}
