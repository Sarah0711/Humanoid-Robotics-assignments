#include <rrt/RRT.h>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <algorithm>

namespace rrt {

/**
 * \brief Draws a random grid node that is not occupied and not in the list.
 * \param[in] list The list of nodes that have already been explored.
 * \param[in] listGoal The goal of the tree.
 * \return The random node.
 */
AbstractNode * RRTGrid::getRandomNode(const std::vector<AbstractNode *>& list, AbstractNode * const listGoal) const {
	AbstractNode* randomNode = NULL;

	// TODO: Draw a random grid cell (90% probability) or return the listGoal (10% probability)

	/* Available methods and fields:
	 * - map.width: width of the map in cells
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */

	return randomNode;
}

/**
 * \brief Calculates the Euclidean distance between two nodes.
 * \param[in] node1 The first node.
 * \param[in] node2 The second node.
 * \return The Euclidean distance between the two nodes.
 */
double RRTGrid::distance(GridNode * const node1, GridNode * const node2) const {
	double dist = 0.0;

	// TODO: Return the Euclidean distance between the two nodes.

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */

	return dist;
}

/**
 * \brief Given a node, this method returns the index of the nearest node in a list.
 * \param[in] node The reference node.
 * \param[in] list The list of nodes that should be searched for the closest node.
 * \return The closest node, or NULL if the list is empty.
 */
AbstractNode * RRT::getClosestNodeInList(AbstractNode * const node, const std::vector<AbstractNode *>& list) const {
	AbstractNode * nearestNode = NULL;

	// TODO: Return the index of the closest node from the list.

	/* Available methods:
	 * - distance(node1, node2): Defined above, returns the Euclidean distance
	 */


	return nearestNode;
}

/**
 * \brief Returns the neighbors of a grid cell that are not occupied and not already expanded.
 * \param[in] currentNode The current grid node.
 * \param[in] list The list of already expanded nodes in the current tree.
 */
std::vector<AbstractNode*> RRTGrid::getNeighbors(GridNode * const currentNode, const std::vector<AbstractNode*>& list) const {
	std::vector<AbstractNode*> neighbors;

	/* TODO: Fill the neighbors vector with neighbor cells of currentNode that are
	 * within the map bounds, not occupied, and not in the list of already expanded nodes.
	 */

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 * - map.width: width of the map in cells
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */
	for (int i = -1; i <= 1; ++i)
	{
		for (int j = -1; j <= 1; ++j)
		{
			if (i != 0 || j != 0)
			{
				if ((currentNode->x + i) >= 0 && (currentNode->x + i < map.width))
				{
					if ((currentNode->y + j) >= 0 && (currentNode->y + j < map.height))
					{
						if (!map.isOccupied(currentNode->x + i, currentNode->y + j))
						{
							if (!(std::find(list.begin(), list.end(), GridNode::get(currentNode->x + i, currentNode->y + j)) != list.end()))
							{
								neighbors.push_back(GridNode::get(currentNode->x + i, currentNode->y + j));
							}
						}
					}
				}
			}
		}
	}


	return neighbors;
}

/**
 * \brief Tries to connect the two trees and returns the connection node.
 * \param[in] currentNode The current node.
 * \param[in] neighbors The list of neighbors of the current node.
 * \param[in] otherList The list of already expanded nodes in the other tree.
 * \return The neighbor node that connects both trees, or NULL if the trees cannot be connected.
 */
AbstractNode * RRT::tryToConnect(AbstractNode* const currentNode,
		const std::vector<AbstractNode*>& neighbors,
		const std::vector<AbstractNode*>& otherList) const
{
	AbstractNode* connectionNode = NULL;

	/* TODO: Check if one of the neighbors is already contained in the "otherList"
	 * (list of already expanded nodes in the other tree). If so, return that neighbor
	 * as the connection node and establish the connection with neighbor->setConnection(closestNode). */

	/* Available methods and fields:
	 * - node->setConnection(AbstractNode * connection): sets the other predecessor node of the
	 *      current node (must be from the other list) (i.e. set connection between the two lists).
	 */
	for (int i = 0; i < neighbors.size(); ++i)
	{
		if ((std::find(otherList.begin(), otherList.end(), neighbors[i]) != otherList.end()))
		{
			neighbors[i]->setConnection(currentNode);
			connectionNode = neighbors[i];
		}
		
	}
	return connectionNode;
}

/**
 * \brief Determines the neighbor that is closest to the random node, sets its predecessor
 * to the current node, and adds it to the list of explored nodes.
 * \param[in] currentNode The current node.
 * \param[in] neighbors The list of neighbors of the current node.
 * \param[in] randomNode The randomly drawn node.
 * \param[in,out] list The list of already expanded nodes.
 */
void RRT::addNearestNeighbor(AbstractNode* const currentNode, std::vector<AbstractNode*>& neighbors,
		AbstractNode* const randomNode, std::vector<AbstractNode*>& list) const {

	/* TODO: Determine the neighbor that is closest to the random node, set its predecessor
	 * to the current node, and add it to the list of explored nodes.
	 */

	/* Available methods and fields:
	 * - node->setPredecessor(AbstractNode* node): store the predecessor node for a node (required
	 *     later for extracting the path)
	 * - getClosestNodeInList(node, list): Defined above
	 */
	currentNode->setPredecessor(getClosestNodeInList(randomNode, neighbors));
	list.push_back(getClosestNodeInList(randomNode, neighbors));

}

/**
 * @brief Given a node, this method expands the nearest node in a list, where the new expanded neighbor is the closest neighbor with respect to the given node.
 * @param[in] randomNode The reference node towards which the list should be expanded.
 * @param[in,out] list The list of nodes that should be expanded for the closest node.
 * @param[in] otherList The other list that should NOT be modified.
 * @return REACHED, TRAPPED, or EXTENDED.
 */
RRT::ExtendStepReturnValue RRT::extendClosestNode(AbstractNode * randomNode,
		std::vector<AbstractNode *> & list, const std::vector<AbstractNode *> & otherList) {

	// Nothing to do in this method - we've already implemented it for you :-)

	if (connectionNode != NULL) {
		// We already found a path in a previous step - nothing to do anymore
		return REACHED;
	}

	// Find the node in the list that is closest to the given random node
	AbstractNode * const closestNode = getClosestNodeInList(randomNode, list);

	// Get the neighbors of the closest node that can be reached in one step
	std::vector<AbstractNode *> neighbors = getNeighbors(closestNode, list);

	// Does one of the neighbor nodes connect the two lists?
	connectionNode = tryToConnect(closestNode, neighbors, otherList);

	if (connectionNode != NULL) {
		// Yes: We found a connection between the two lists and connectionNode is the
		// neighbor that connects the two lists.
		return REACHED;
	} else if (neighbors.empty()) {
		// There are no valid neighbor nodes: We are trapped in a dead end.
		return TRAPPED;
	} else {
		// Extend the list by adding the neighbor that is closest to the random node.
		addNearestNeighbor(closestNode, neighbors, randomNode, list);
		return EXTENDED;
	}
}

/**
 * \brief Reconstructs the path from the start to the goal once the connection is found.
 * \param[in] connectionNode The connection node where both trees meet.
 * \param[in] startNode The start node.
 * \param[in] goalNode The goal node.
 * \return The path from the start node to the goal node.
 */
std::deque<AbstractNode *> RRT::constructPath(
        AbstractNode * const connectionNode, 
        AbstractNode * const startNode,
		AbstractNode * const goalNode) const 
{
	std::deque<AbstractNode *> path;
	if (connectionNode == NULL) {
		return path;
	}

	/* TODO: Reconstruct the path from the start node to the goal node in the correct order.
	 *
	 * Hints:
	 * - Start with the connection node and follow the chain of predecessors in both trees.
	 * - Depending on which tree the connection node is part of, you may have to reverse
	 *   the order of the nodes in the end.
	 * */

	/* Available methods:
	 * - node->getPredecessor(): returns the predecessor saved with setPredecessor()
	 * - node->getConnection() : returns the predecessor from the other list saved with setConnection()
	 * - path.push_front(AbstractNode* node): Inserts the node at the beginning of the path
	 * - path.push_back(AbstractNode* node): Inserts the node at the end of the path
	 */
	
	AbstractNode* node;
	path.push_back(connectionNode);
	int flag = 0;
	node = connectionNode;
	while ( node != startNode)
	{
		node = node->getPredecessor();
		if (node == goalNode)
		{
			flag = 1;
			break;
		}
		path.push_front(node);	
	}
	node = connectionNode->getConnection();
	path.push_back(node);
	while (node != goalNode)
	{
		node = node->getPredecessor();
		if (node == startNode)
		{
			break;
		}
		path.push_back(node);	
	}
	if (flag == 1)
	{
		reverse(path.begin(), path.end());
		path.push_front(startNode);
		path.push_back(goalNode);
	}
	return path;
}

/**
 * \brief Plans a path on a grid map using RRT.
 * \param[in] startNode The start node of the path.
 * \param[in] goalNode The goal node where the path should end up.
 * \param[in] maxIterations The maximum number of iterations.
 * \return The planned path, or an empty path in case the algorithm exceeds the maximum number of iterations.
 */
std::deque<AbstractNode *> RRT::planPath(AbstractNode * const startNode, AbstractNode * const goalNode, const size_t& maxIterations) {
	std::deque<AbstractNode *> result;

	// Reset the connection node if it was set in a previous run:
	connectionNode = NULL;

	// Create new node lists for the forward and backward trees:
	std::vector<AbstractNode *> startList;
	std::vector<AbstractNode *> goalList;

	// Add the start and goal nodes to the corresponding lists:
	startList.push_back(startNode);
	goalList.push_back(goalNode);

	/* TODO:  Expand trees from both the start node and the goal node at the same time
	 * until they meet. When extendClosestNode() returns a connection node, then call
	 * constructPath() to find the complete path and return it. */
	return result;
}
}  // namespace rrt


