#include <ara_star/ARAStar.h>
#include <ara_star/Logger.h>
#include <windows-helpers.h>
#include <iostream>
#include <fstream>

using namespace ara_star;

int main(int /* argc */, char ** /*argv*/)
{
	const std::string packagePath = PROJECT_SOURCE_DIR;
	ARAStarHeuristicLog heuristic;
	const GridMap *map = FileIO::loadMap(PROJECT_SOURCE_DIR + std::string("/data/map.pbm"));
	OpenList::map = map;
	ARAStarPlanningLog planner(*map, heuristic);
	const double wInitial = 5;
	const double wDelta = .5;
	const double timeLimit = 60.;
	const AbstractNode *startNode = GridNode::get(17, 48);
	const AbstractNode *goalNode = GridNode::get(85, 47);
	std::deque<const AbstractNode *> path = planner.runARA(wInitial, wDelta, timeLimit, startNode, goalNode);
	if (path.empty()) {
		std::cout << "Did not find a path." << std::endl;
	} else {
		std::cout << "Found a path of length " << path.size() << std::endl;
	}

	planner.savePathLengthHistory();
	delete map;
	map = NULL;
    wait();
	return 0;
}
