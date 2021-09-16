#include <windows-helpers.h>
#include <iostream>
#include <path_planning/PathPlanning.h>
#include <path_planning/FileIO.h>

using namespace path_planning;

int main(int /* argc */, char ** /*argv*/) 
{
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO;
	fileIO.openLogfile(packagePath + "/data/log.txt");
	OpenList::fileIO = &fileIO;
	ClosedList::fileIO = &fileIO;

	const GridMap * map = fileIO.loadMap(packagePath + "/data/map.pbm");
	if (!map) {
        wait();
		return 1;
	}
	OpenList::map = map;

	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planning(*map, heuristic);
	const GridNode * const startNode = GridNode::get(4, 9);
	const GridNode * const goalNode = GridNode::get(2, 7);
	std::deque<const AbstractNode *> path = planning.planPath(startNode, goalNode);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << std::endl;
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			if (!*it) {
				throw std::runtime_error("Path contains a NULL pointer.");
			}
			std::cout << (*it)->toString() << std::endl;
		}
	}

	delete map;
	map = NULL;

    wait();
	return 0;
}
