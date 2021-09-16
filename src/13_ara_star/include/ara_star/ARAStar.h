#ifndef ARA_STAR_H_
#define ARA_STAR_H_

#include <sstream>

#include <ara_star/Heuristic.h>
#include <ara_star/PathPlanning.h>

namespace ara_star {

/**
 * @brief ARA* planning on a grid map.
 */
class ARAStarHeuristic : public GridHeuristic {
public:
	ARAStarHeuristic() : w(1.0) {};
	virtual ~ARAStarHeuristic() {};

	virtual double heuristic(const GridNode * const currentNode,
			const GridNode * const goalNode) const;

public:
	/**
	 * @brief Sets the W value of the ARA* heuristic (1.0 = original heuristic, > 1.0 = inflated heuristic)
	 * @param w The W value.
	 */
	virtual void setW(const double& w) {
		if (w < 0) {
			throw std::invalid_argument("Negative values for w are not allowed.");
		}
		this->w = w;
	}

	/**
	 * @brief Returns the current W value of the ARA* heuristic.
	 * @return W value.
	 */
	virtual const double& getW() const {
		return w;
	}

private:
	double w;
};

/**
 * @brief ARA* planning.
 *
 * This class overloads the methods of GridPathPlanning with grid-based node types for convenience.
 */
class ARAStarPlanning : public GridPathPlanning {
public:
	/**
	 * @brief Constructs an A* planner for a given grid map and heuristic.
	 * @param map The grid map for which a plan should be generated.
	 * @param heuristic The heuristic to use.
	 */
	ARAStarPlanning(const GridMap& map, ARAStarHeuristic& heuristic) : GridPathPlanning(map, heuristic),
	    map_(map), heuristic_(heuristic) {};
	virtual ~ARAStarPlanning() {};

	virtual std::deque<const AbstractNode*> planPath(const ara_star::AbstractNode * const startNode, const ara_star::AbstractNode * const goalNode,const time_t & tstart,const double& timeLimit);

	std::deque<const AbstractNode*> runARA(const double& wInitial, const double& wDelta, const double& timeLimit,const ara_star::AbstractNode * const startNode, const ara_star::AbstractNode * const goalNode);

private:
   	const GridMap map_;
   	ARAStarHeuristic& heuristic_;
};

}  // namespace ara_star


#endif  // ARA_STAR_H_
