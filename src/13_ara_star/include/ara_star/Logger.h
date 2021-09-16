#ifndef INCLUDE_ARA_STAR_LOGGER_H_
#define INCLUDE_ARA_STAR_LOGGER_H_

#include <ara_star/ARAStar.h>
#include <ctime>

namespace ara_star {

/**
 * @brief Helper class for logging ARA* heuristic.
 */
class ARAStarHeuristicLog : public ARAStarHeuristic {
public:
	std::vector<double> wHistory;
	virtual void setW(const double& w);
};

/**
 * @brief Helper class for logging ARA* planning iterations.
 */
class ARAStarPlanningLog : public ARAStarPlanning{
	const GridMap mapHistory;
	ARAStarHeuristicLog& heuristic;
	struct PathLengthData {
		size_t pathLength;
		clock_t start;
		clock_t end;
	};
	std::vector<PathLengthData> pathLengthList;
public:

	std::vector< std::vector<int> > mapLog;
	ARAStarPlanningLog(const GridMap& map, ARAStarHeuristicLog& heuristic);

	virtual std::deque<const AbstractNode*> planPath(const ara_star::AbstractNode * const startNode, const ara_star::AbstractNode * const goalNode,const time_t & tstart,const double& timeLimit);

	virtual void expandNode(const AbstractNode * const currentNode, const ara_star::AbstractNode * const goalNode,
		OpenList& openList, const ara_star::ClosedList& closedList);

	void savePathLengthHistory();
};

}  // namespace ara_star

#endif /* INCLUDE_ARA_STAR_LOGGER_H_ */
