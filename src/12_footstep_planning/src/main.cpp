#include <windows-helpers.h>
#include <iostream>
#include <footstep_planning/FileIO.h>
#include <footstep_planning/FootstepPlanning.h>

using namespace footstep_planning;

int main(int /* argc */, char ** /*argv*/)
{
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath);
	FootstepPlanning planning(fileIO.map);
	const FootstepNode * const start = FootstepNode::get(1.0, 0.2, 0.0, LEFT);
	const FootstepNode * const goal = FootstepNode::get(2.0, 2.0, -0.5 * M_PI, RIGHT);
	std::cout << "Planning path (this will take a while) ..." << std::endl;
	const std::deque<const AbstractNode *> path = planning.planPath(start, goal);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << std::endl;
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			if (!*it) {
				throw std::runtime_error("The path contains a NULL pointer.");
			}
			std::cout << (*it)->toString() << std::endl;
		}
	}

	std::cout << "Done. " << std::endl;
    wait();
	return 0;
}
