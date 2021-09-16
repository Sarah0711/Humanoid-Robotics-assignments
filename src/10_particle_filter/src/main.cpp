#include <windows-helpers.h>
#include <iostream>
#include <particle_filter/ParticleFilter.h>
#include <particle_filter/FileIO.h>
#include <ctime>
#include <cstdlib>

using namespace particle_filter;

int main(int /* argc */, char ** /*argv*/) 
{
	srand((unsigned) time(0));  //initialize the random number generator

	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath + "/data/data.txt", packagePath + "/data/result.txt");
	if (!fileIO.isOK()) {
        wait();
		return 2;
	}

	const double odom_stdev = 0.5;
	const double measurement_stdev = 0.1;

	std::vector<ParticleFilter::Particle> particles(250);
	ParticleFilter::initParticles(particles);
	fileIO.writeMap(particles);

	for (size_t i = 0; i < fileIO.odom.size(); ++i) {
		ParticleFilter::integrateMotion(particles, fileIO.odom[i], odom_stdev);
		ParticleFilter::integrateObservation(particles, fileIO.measurement[i], measurement_stdev);
		particles = ParticleFilter::resample(particles);
		fileIO.writeMap(particles);
	}	

	std::cout << "Wrote particle positions to 10_particle_filter/data/result.txt." << std::endl;

    wait();
	return 0;
}
