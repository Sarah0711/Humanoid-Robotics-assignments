#include <windows-helpers.h>
#include <iostream>
#include <rrt/RRT.h>
#include <rrt/FileIO.h>
#include <rrt/Logger.h>
#include <ctime>
#include <cstdlib>

using namespace rrt;

int main(int /* argc */, char ** /*argv*/) {
	srand((unsigned) time(NULL));

	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO;
	fileIO.openLogfile(packagePath + "/data/log.txt");

	const GridMap * const map = fileIO.loadMap(packagePath + "/data/map.pbm");
	if (!map) {
        wait();
   		return 1;
	}

	GridNode * const startNode = GridNode::get(7, 7);
	GridNode * const goalNode = GridNode::get(11, 33);

	RRTWithLoggers rrtTree(&fileIO, *map, startNode, goalNode);

	const std::deque<AbstractNode *> path = rrtTree.planPath(startNode, goalNode, 10000);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Extended " << rrtTree.getNumExtendedNodes() << " nodes. Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << "length " << path.size() << std::endl;
	}

	delete map;
    wait();
	return 0;

}
