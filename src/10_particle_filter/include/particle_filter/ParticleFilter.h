#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>

#ifndef M_PI
#define M_PI 3.141592654
#endif

namespace particle_filter {

/// \brief Particle filter for 1D environment.
class ParticleFilter
{

public:
	/// \brief Structure representing a particle.
   	struct Particle {
   		double x;          ///< The position of the particle on the x axis
   		double weight;     ///< The weight of the particle

   		Particle() : x(0.0), weight(0.0) {}
   		/**
   		 * \brief Constructs a particle with given position and weight.
   		 * @param x_ Position of the particle on the x axis.
   		 * @param weight_ Weight of the particle.
   		 */
   		Particle(const double& x_, const double& weight_) : x(x_), weight(weight_) {}
   	};

	ParticleFilter() {};
   	virtual ~ParticleFilter() {};

   	static double gaussianProbability(const double& d, const double& stdev);
   	static double sampleFromGaussian(const double& mean, const double& stdev);
   	static double getDistanceToNearestLight(const double& x);
   	static void normalizeWeights(std::vector<Particle>& particles);
   	static void initParticles(std::vector<Particle>& particles);
   	static void integrateMotion(std::vector<Particle>& particles, const double& ux, const double& stdev);
   	static void integrateObservation(std::vector<Particle>& particles, const double measurement, const double& stdev);
   	static std::vector<Particle> resample(const std::vector<Particle>& particles);
};

}  // namespace particle_filter

#endif  // PARTICLE_FILTER_H_
