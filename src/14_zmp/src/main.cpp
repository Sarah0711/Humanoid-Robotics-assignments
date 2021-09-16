#include <windows-helpers.h>
#include <zmp/ZMP.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace zmp;

typedef double Pressure;

void printVec(std::ostream& s, const Eigen::Vector3d& v) {
	for (size_t i = 0; i < 3; ++i) {
		if (fabs((double) v[i]) < 1e-10) {
			s << 0 << " ";
		} else {
			s << v[i] << " ";
		}
	}
}

int main(int /* argc */, char ** /*argv*/) {
    ZMP zmp;
	zmp.m = 1.0;
	zmp.n << 0, 0, 1;

	zmp.C << 0, 0, 0.5;
	zmp.O = Eigen::Vector3d(0, 0, 0);

	const double cubeHeight = 0.8;
	const double cubeWidth = 0.2;
	const double cubeDepth = 0.2;

	const double timeStep = 0.05;
	const double duration = 3.0;

	Force extForce = Force(0.0, 0.0, 0.0);
	Point3d extPoint = Point3d(0.1, 0.0, 0.6);

	const std::string filename = std::string(PROJECT_SOURCE_DIR) + "/data/log.txt";
	const std::string momentsFilename = std::string(PROJECT_SOURCE_DIR) + "/data/moments.txt";
	std::ofstream file(filename.c_str());

	if (file.good()) {
		file << "#time ZMP.x ZMP.y ZMP.z C.x C.y C.z ExtF.x ExtF.y ExtF.z ExtP.x ExtP.y ExtP.z CF.x CF.y CF.z M_CoM.x M_CoM.y M_CoM.z" << std::endl;
	} else {
		std::cerr << "Cannot open log file " << filename << " for writing.";
	}
	for (double time = 0; time < duration; time += timeStep) {
		zmp.F_E = extForce;
		zmp.E = extPoint;
		const Point3d Z = zmp.zeroMomentPoint(zmp.O);
		const bool zmpExists = !(std::isinf((double) Z[0]) || std::isinf((double) Z[1]) || std::isinf((double) Z[2]));
		const Point3d CoP = zmp.computeCoP(Z);
		const Force CF = zmp.contactForce();
		const AngularMoment CoMTorque = zmp.resultingMomentAtCoM(CoP);
		if (file.good()) {
			file << time << " ";
			printVec(file, Z);
			printVec(file, zmp.C);
			printVec(file, extForce);
			printVec(file, extPoint);
			printVec(file, CF);
			printVec(file, CoMTorque);
			file << std::endl;
		}

		size_t bufferN = strlen(PROJECT_SOURCE_DIR) + 40;
		char *buffer = (char *) calloc(bufferN, sizeof(char));
		snprintf(buffer, bufferN, "%s/data/moments-%4.2f.txt", PROJECT_SOURCE_DIR, time);
		std::ofstream momentsFile(buffer);
		if (momentsFile.good()) {
			momentsFile << "#x y M.x M.y M.z |Mxy| MC.x MC.y MC.z |MCxy| p" << std::endl;
			const int iwidth=11, idepth=11;
			std::vector<double> p;
			p.resize(iwidth*idepth, 0.0);
			const double rx = std::min(1.0, std::max(0.0, (double) (Z[0]+cubeWidth/2.) / cubeWidth));
			const double ry = std::min(1.0, std::max(0.0, (double) (Z[1]+cubeDepth/2.) / cubeDepth));

			const double ax = rx < 2./3. ? 0 : 1 - 3 * (1-rx);
			const double bx = rx > 1./3. ? 1 : 3 * rx;
			const double ay = ry < 2./3. ? 0 : 1 - 3 * (1-ry);
			const double by = ry > 1./3. ? 1 : 3 * ry;

			const int axi = (int) (ax * (iwidth-1));
			const int bxi = (int) (bx * (iwidth-1));
			const int ayi = (int) (ay * (idepth-1));
			const int byi = (int) (by * (idepth-1));

			const double qaa = ((bxi-axi == 0) ? 1 : -2*(2*(ax-bx)+3*rx)/((ax-bx)*(ax-bx))) + ((byi-ayi == 0) ? 1 : -2*(2*(ay-by)+3*ry)/((ay-by)*(ay-by)));
			const double qbb = ((bxi-axi == 0) ? 1 :  2*(  (ax-bx)+3*rx)/((ax-bx)*(ax-bx))) + ((byi-ayi == 0) ? 1 :  2*(  (ay-by)+3*ry)/((ay-by)*(ay-by)));
			const double qab = ((bxi-axi == 0) ? 1 : -2*(2*(ax-bx)+3*rx)/((ax-bx)*(ax-bx))) + ((byi-ayi == 0) ? 1 :  2*(  (ay-by)+3*ry)/((ay-by)*(ay-by)));
			const double qba = ((bxi-axi == 0) ? 1 :  2*(  (ax-bx)+3*rx)/((ax-bx)*(ax-bx))) + ((byi-ayi == 0) ? 1 : -2*(2*(ay-by)+3*ry)/((ay-by)*(ay-by)));

			for (int y = ayi; y <= byi; ++y) {
				const double mry = static_cast<double>(y-ayi)/static_cast<double>(byi-ayi);
				const double l = mry * qaa + (1. - mry) * qab;
				const double r = mry * qba + (1. - mry) * qbb;
				p[y*iwidth + axi] = l;
				p[y*iwidth + bxi] = r;
				for (int x = axi + 1; x < bxi; ++x) {
					const double mrx = static_cast<double>(x-axi)/static_cast<double>(bxi-axi);
					p[y*iwidth + x] =  mrx * r + (1. - mrx) * l;
				}
			}

			double pSum = 0;
			for (size_t i = 0; i < p.size(); ++i) {
				pSum += p[i];
			}

			const double F = zmp.contactForce()[2];
			for (size_t i = 0; i < p.size(); ++i) {
				p[i] = p[i] / pSum * F;
			}

			size_t i = 0;
			for (double y = -cubeDepth/2.; y <= cubeDepth/2. + cubeDepth/20.; y += (cubeDepth / 10.)) {
				for (double x = -cubeWidth/2.; x <= cubeWidth/2. + cubeWidth/20.; x += (cubeWidth / 10.), ++i) {
					const Point3d X(x, y, 0.);
					const AngularMoment giMoment = zmp.giMoment(X);
					const AngularMoment pMoment  = zmp.contactMoment(CoP, X);
					momentsFile << x << " " << y << " " <<
							giMoment[0] << " " << giMoment[1] << " " << giMoment[2] << " " << sqrt(static_cast<double>(giMoment[0]*giMoment[0]+giMoment[1]*giMoment[1])) << " " <<
							pMoment[0]  << " " << pMoment[1]  << " " << pMoment[2]  << " " << sqrt(static_cast<double>(pMoment[0]*pMoment[0]+pMoment[1]*pMoment[1])) << " " <<
							p[i];
					momentsFile << std::endl;
				}
			}
			momentsFile << std::endl;
		} else {
			std::cerr << "Could not open " << buffer << " for writing." << std::endl;
		}
		momentsFile.close();
		free(buffer);

		extForce[0] -= 1.0 * timeStep;
	}
	file.close();
	std::cout << "Wrote log file to " << filename << std::endl;

    wait();
	return 0;
}
