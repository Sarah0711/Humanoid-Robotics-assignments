#include <particle_filter/ParticleFilter.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <boost/random/random_device.hpp>


namespace particle_filter {

/**
 * \brief Calculate the probability phi(d, stdev) of a measurement according to a Gaussian distribution.
 * \param[in] d The difference between the measurement and the mean
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return Probability of the measuredment.
 */
double ParticleFilter::gaussianProbability(const double& d, const double& stdev) {
	double probability = 0.0;
	double param = d/stdev;
	double inv_sqrt_2pi = 0.3989422804014327;
	;
	probability = inv_sqrt_2pi*(1/stdev)*exp(-0.5*param*param);

	/*TODO: Calculate the probability of the measurement for a Gaussian distribution with
	  the given mean and standard deviation */
	return probability;
}

/**
 * \brief Draw a sample from a Gaussian distribution.
 * \param[in] mean The mean of the Gaussian.
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return A random sample drawn from the given Gaussian distribution.
 */
double ParticleFilter::sampleFromGaussian(const double& mean, const double& stdev) {
	double result = 0.0;
	boost::random::random_device generator;
	boost::normal_distribution<double> distribution(mean,stdev);
	result = distribution(generator);

	//TODO: draw a sample from a 1D Gaussian
	return result;
}


/**
 * \brief Initializes the position and weights of the particles.
 * \param[in,out] particles The list of particles.
 *
 * The positions should be distributed uniformly in the interval [0, 10].
 * The weights should be equal and sum up to 1.
 */
void ParticleFilter::initParticles(std::vector<Particle>& particles) {
	//TODO: Distribute the particles randomly between [0, 10] with equal weights
	int size = particles.size();
	boost::random::random_device generator;

	boost::random::uniform_real_distribution<double> distribution(0.0,10.0);

	double weights = 1.0/size;

	for(int i=0 ;i<size; i++){
		particles[i].x = distribution(generator);
		particles[i].weight = weights;
	}
}

/**
 * \brief Normalizes the weights of the particle set so that they sum up to 1.
 * \param[in,out] particles The list of particles.
 */
void ParticleFilter::normalizeWeights(std::vector<Particle>& particles) {
	double normalizationFactor = 0.0;

	int size = particles.size();

	for (int i = 0; i < size;i++)
	{
		normalizationFactor += particles[i].weight;
	}

	for (int i = 0; i < size; i++)
	{
		particles[i].weight = particles[i].weight / normalizationFactor;
	}

	//TODO: normalize the particles' weights so that they sum up to 1.
}

/**
 * \brief Displace the particles according to the robot's movements.
 * \param[in,out] particles The list of particles.
 * \param[in] ux The odometry (displacement) of the robot along the x axis.
 * \param[in] stdev The standard deviation of the motion model.
 */
void ParticleFilter::integrateMotion(std::vector<Particle>& particles, const double& ux, const double& stdev) {
	//TODO: Prediction step: Update each sample by drawing the a pose from the motion model.

	int n = particles.size();
	double mean = 0.0;
	for (int i = 0; i < n;i++)	
	{
		mean += particles[i].x;
	}

	mean /= (double)n;
	//boost::random::random_device generator;

	for (int i = 0; i < n;i++)
	{
		particles[i].x = gaussianProbability(ux, stdev);
	}

}


/**
 * \brief Returns the distance between the given x position and the nearest light source.
 * \param[in] x The position on the x axis.
 * \return The distance to the nearest light source.
 */
double ParticleFilter::getDistanceToNearestLight(const double& x) {
	double dist = 0.0;
	//TODO Return the distance from the robot's position x to the nearest light source.
	double firstSource = 0.0, secondSource = 0.0, thirdSource = 0.0;
	double a = 2.0, b = 6.0, c = 8.0;
	firstSource = fabs(x - a);
	secondSource = fabs(x - b);
	thirdSource = fabs(x - c);

	std::cout << " x  " << x << "firstSource  " << firstSource << "secondSource " << secondSource << " thirdSource " << 
		thirdSource <<  std::endl;

	if (firstSource <= secondSource && firstSource <= thirdSource)
		dist = firstSource;
	else if (secondSource <= firstSource && secondSource <= thirdSource)
		dist = secondSource;
	else
		dist = thirdSource;

	return dist;
}

/**
 * \brief Updates the particle weights according to the measured distance to the nearest light source.
 * \param[in,out] particles The list of particles.
 * \param[in] measurement The measured distance between the robot and the nearest light source.
 * \param[in] stdev The standard deviation of the observation model.
 */
void ParticleFilter::integrateObservation(std::vector<Particle>& particles, const double measurement,
	const double& stdev) {
	//TODO: Correction step: weight the samples according to the observation model.
	int n = particles.size();
	for (int i = 0; i < n;i++)
	{
		double dist = getDistanceToNearestLight(particles[i].x);
		particles[i].weight = gaussianProbability((dist - measurement), stdev);
	}

	// Normalize the weights after updating so that they sum up to 1 again:
	normalizeWeights(particles);
}

/**
 * \brief Resamples the particle set by throwing out unlikely particles and duplicating more likely ones.
 * \param[in] particles The old list of particles.
 * \return The new list of particles after resampling.
 */
std::vector<ParticleFilter::Particle> ParticleFilter::resample(const std::vector<Particle>& particles) {
	std::vector<Particle> newParticles;
	double n = (double) particles.size();
	boost::random::random_device generator;
	boost::random::uniform_real_distribution<double> distribution(particles[0].weight, particles[n].weight);
	/*double r = distribution(generator);
	double c = particles[0].weight;
	int i = 0;
	for (int k = 0; k < particles.size();k++)
	{
		double U = (r + ((double)k + (double)1) - (double)1) / (double) particles.size();
		while (U > c)
		{
			i++;
			c += particles[i].weight;
		}
		//newParticles[k].x = particles[k].x;
		newParticles.push_back(particles[k]);
	}*/

	for (int i = 0; i < particles.size(); i++) {
		newParticles.push_back(particles[distribution(generator)]);
	}
	   
	/*TODO: Use stochastic universal resampling (also called low variance resampling)
	 * to draw a new set of particles according to the old particles' weights */


	return newParticles;
}

}  // namespace particle_filter

