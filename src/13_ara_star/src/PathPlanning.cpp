#include <ara_star/PathPlanning.h>
#include <cmath>

namespace ara_star {

double GridPathPlanning::getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const {
	double result = 0.0;

	const double dx = currentNode->x - successorNode->x;
	const double dy = currentNode->y - successorNode->y;
	result = sqrt(dx * dx + dy * dy);

	return result;
}


std::vector<AbstractNode *> GridPathPlanning::getNeighborNodes(const GridNode * const currentNode, const GridMap& map) {
	std::vector<AbstractNode *> result;

	for (int dx = -1; dx <= 1; ++dx) {
		for (int dy = -1; dy <= 1; ++dy) {
			if (dx != 0 || dy != 0) {
				const int x = currentNode->x + dx;
				const int y = currentNode->y + dy;
				if (x >= 0 && x < static_cast<int>(map.width) &&
					    y >= 0 && y < static_cast<int>(map.height) &&
					    !map.isOccupied(x, y)) {
					result.push_back(GridNode::get(x, y));
				}
			}
		}
	}

	return result;
}


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


bool GridPathPlanning::isCloseToGoal(const GridNode * const currentNode, const GridNode * const goalNode) {
	// We already implemented this method for you, nothing to do here.
	return ((currentNode->x == goalNode->x) && (currentNode->y == goalNode->y));
}


std::deque<const AbstractNode*> PathPlanning::followPath(const AbstractNode * const node) {
	std::deque<const AbstractNode *> path;
	for (const AbstractNode * currentNode = node; currentNode != NULL;
			currentNode = currentNode->getPredecessor()) {
		path.push_front(currentNode);
	}
	return path;
}

}  // namespace ara_star
