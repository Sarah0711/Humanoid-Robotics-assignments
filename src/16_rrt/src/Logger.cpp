#include <rrt/Logger.h>
#include <algorithm>

namespace rrt {

RRTWithLoggers::RRTWithLoggers(FileIO * const fileIO, const GridMap& map, const GridNode * const startNode,
		const GridNode * const goalNode) :
		RRTGrid(map), fileIO(fileIO), listNameForward("forward"),		
    	listNameBackward("backward"), numExtendedNodes(0),
    	startNode(startNode), goalNode(goalNode) {
}

RRTWithLoggers::~RRTWithLoggers() {
}

RRT::ExtendStepReturnValue RRTWithLoggers::extendClosestNode(AbstractNode * const randomNode,
		std::vector<AbstractNode *> & list, const std::vector<AbstractNode *> & otherList) {
	++numExtendedNodes;
	const std::string * listName;
	if (list[0] == startNode) {
		listName = &listNameForward;
	} else if (list[0] == goalNode) {
		listName = &listNameBackward;
	} else {
		throw std::runtime_error("Invalid list name");
	}
	std::vector<AbstractNode *> listBefore(list.begin(), list.end());
	const AbstractNode * const closestNode = getClosestNodeInList(randomNode, list);

	ExtendStepReturnValue retval = RRT::extendClosestNode(randomNode, list, otherList);
	// Find the node that was not there before (we don't know where it was inserted in the list):
	for (std::vector<AbstractNode *>::const_reverse_iterator it = list.rbegin(); it != list.rend(); ++it) {
		if (std::find(listBefore.begin(), listBefore.end(), *it) == listBefore.end()) {
			fileIO->logExtend((GridNode *) closestNode, (GridNode *) (*it), (GridNode *) randomNode, listName);
			return retval;
		}
	}
	fileIO->logFailedExtend((GridNode *) closestNode, (GridNode *) randomNode, listName);
	return retval;
}

}  // namespace rrt
