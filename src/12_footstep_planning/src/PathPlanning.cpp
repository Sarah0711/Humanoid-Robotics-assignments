#include <footstep_planning/PathPlanning.h>
#include <cmath>

namespace footstep_planning {

void PathPlanning::expandNode(const AbstractNode * const currentNode, const AbstractNode * const goalNode,
		OpenList& openList, const ClosedList& closedList) {

	// Get the list of all neighbors of the cell
	typedef std::vector<AbstractNode *> Neighbors;
	Neighbors neighbors = getNeighborNodes(currentNode);
	for (Neighbors::const_iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		// If the successor is already on the closed list: do nothing
		if (closedList.contains(*neighborIt)) {
			continue;
		}

		// Calculate the costs of the successor node: costs of the current node
		// plus transition costs to the successor
		const double tentative_g = currentNode->costs + getCosts(currentNode, *neighborIt);

		// If the successor is already in the open list and the new path is not better
		// than the old path: do nothing
		if (openList.contains(*neighborIt) && tentative_g > (*neighborIt)->costs) {
			continue;
		}

		// Set predecessor and save new g value
		(*neighborIt)->setPredecessor(currentNode);
		(*neighborIt)->costs = tentative_g;

		// Calculate the new f value (costs from start + heuristic to goal)
		const double f = tentative_g + heuristic(*neighborIt, goalNode);

		// Update f value of the node in the open list or insert the new node
		if (openList.contains(*neighborIt)) {
			openList.updateCosts(*neighborIt, f);
		} else {
			openList.enqueue(*neighborIt, f);
		}
	}
}

std::deque<const AbstractNode*> PathPlanning::planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode) {
	// Create empty lists for open nodes and closed nodes
   	OpenList openList;
   	ClosedList closedList;

   	std::deque<const AbstractNode*> resultPath;
	openList.enqueue(startNode, 0.0);
	while (!openList.isEmpty()) {
		const AbstractNode * currentNode = openList.removeMin();
		if (isCloseToGoal(currentNode, goalNode)) {
			resultPath = followPath(currentNode);
			break;
		}
		closedList.add(currentNode);
		expandNode(currentNode, goalNode, openList, closedList);
	}

	return resultPath;
}


std::deque<const AbstractNode*> PathPlanning::followPath(const AbstractNode * const node) {
	std::deque<const AbstractNode *> path;
	for (const AbstractNode * currentNode = node; currentNode != NULL;
			currentNode = currentNode->getPredecessor()) {
		path.push_front(currentNode);
	}
	return path;
}

}  // namespace footstep_planning
