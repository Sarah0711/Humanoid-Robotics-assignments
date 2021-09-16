#include <gtest/gtest.h>
#include <path_planning/PathPlanning.h>
#include <math.h>

using namespace path_planning;

TEST(PathPlanning, getCosts) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);
	EXPECT_DOUBLE_EQ(planner.getCosts(GridNode::get(2, 3), GridNode::get(5, 7)), 5.0);
	EXPECT_DOUBLE_EQ(planner.getCosts(GridNode::get(2, 3), GridNode::get(2, 3)), 0.0);
	EXPECT_DOUBLE_EQ(planner.getCosts(GridNode::get(5, 3), GridNode::get(4, 7)), planner.getCosts(GridNode::get(4, 7), GridNode::get(5, 3)));

}

TEST(PathPlanning, straightLineDistanceHeuristic) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7)), 5.0);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(2, 3), GridNode::get(2, 3)), 0.0);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(5, 3), GridNode::get(4, 7)), planner.heuristic(GridNode::get(4, 7), GridNode::get(5, 3)));
}

TEST(PathPlanning, manhattanDistanceHeuristic) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	ManhattanDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7)), 7.0);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(2, 3), GridNode::get(2, 3)), 0.0);
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(5, 3), GridNode::get(4, 7)), planner.heuristic(GridNode::get(4, 7), GridNode::get(5, 3)));
}

TEST(PathPlanning, getNeighborNodes) {
	std::vector<bool> data(100, false);
	data[5 * 10 + 5] = true;
	GridMap map(10, 10, data);
	OpenList::map = &map;
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);
	std::vector<AbstractNode *> v = planner.getNeighborNodes(GridNode::get(3, 7));
	if (v.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x < 2 || node->x > 4 || node->y < 6 || node->y > 8) {
			FAIL() << "The method returns (" << node->x << ", " << node->y << ") as a neighbor of (3, 7), although it is not a neighboring cell.";
		}
		if (node->x == 3 && node->y == 7) {
			FAIL() << "The method includes the current node as a neighbor node. The current node must not be returned as a neighbor in order to prevent infinite loops.";
		}
	}
	for (int x = 2; x <= 4; ++x) {
		for (int y = 6; y <= 8; ++y) {
			if (x == 3 && y == 7) {
				continue;
			}
			bool found = false;
			for (size_t i = 0; i < v.size() && !found; ++i) {
				const GridNode * node = static_cast<const GridNode *>(v[i]);
				if (node->x == x && node->y == y) {
					found = true;
				}
			}
			if (!found) {
				FAIL() << "The node (" << x << ", " << y << ") should be returned as a neighbor of (3, 7), but it is not in the result vector.";
			}
		}
	}

	v = planner.getNeighborNodes(GridNode::get(4, 4));
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x == 5 && node->y == 5) {
			FAIL() << "The method returns nodes that are occupied in the map. These nodes must not be returned as neighbors.";
		}
	}

	v = planner.getNeighborNodes(GridNode::get(9, 9));
	std::pair<int, int> expected[3] = {
			std::make_pair<int, int>(8, 8),
			std::make_pair<int, int>(8, 9),
			std::make_pair<int, int>(9, 8)
	};
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x > 9 || node->y > 9) {
			FAIL() << "The method returns cells that are outside the map. The method should make sure only cells within the map bounds get returned.";
		}
	}
	for (size_t i = 0; i < 3; ++i) {
		bool found = false;
		const int x = expected[i].first;
		const int y = expected[i].second;
		for (size_t j = 0; j < v.size() && !found; ++j) {
			const GridNode * node = static_cast<const GridNode *>(v[j]);
			if (node->x == x && node->y == y) {
				found = true;
			}
		}
		if (!found) {
			FAIL() << "The node (" << x << ", " << y << ") should be returned as a neighbor of (9, 9), but it is not in the result vector.";
		}
	}

	v = planner.getNeighborNodes(GridNode::get(0, 0));
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x < 0 || node->y < 0) {
			FAIL() << "The method returns cells that are outside the map. The method should make sure only cells within the map bounds get returned.";
		}
	}
	std::pair<int, int> expected2[3] = {
			std::make_pair<int, int>(0, 1),
			std::make_pair<int, int>(1, 0),
			std::make_pair<int, int>(1, 1)
	};
	for (size_t i = 0; i < 3; ++i) {
		bool found = false;
		const int x = expected2[i].first;
		const int y = expected2[i].second;
		for (size_t j = 0; j < v.size() && !found; ++j) {
			const GridNode * node = static_cast<const GridNode *>(v[j]);
			if (node->x == x && node->y == y) {
				found = true;
			}
		}
		if (!found) {
			FAIL() << "The node (" << x << ", " << y << ") should be returned as a neighbor of (0, 0), but it is not in the result vector.";
		}
	}
}

void makeMap(const char *mapchar, std::vector<bool>& data) {
	data.resize(strlen(mapchar));
	for(size_t i = 0; i < data.size(); ++i) {
		if (mapchar[i] == '#') {
			data[i] = true;
		} else {
			data[i] = false;
		}
	}
}

class OpenListTest : public OpenList {
public:
	void test(const GridMap& map, const ClosedList& closedList) const {
		for (size_t x = 0; x < map.width; ++x) {
		    for (size_t y = 0; y < map.height; ++y) {
		        const GridNode * const gridNode = GridNode::get(x, y);
		        if (this->contains(gridNode)) {
			        if (map.isOccupied(gridNode->x, gridNode->y)) {
				        FAIL() << "The method adds an occupied cell to the open list. Only free cells should be added.";
			        }
			        if (closedList.contains(gridNode)) {
				        FAIL() << "The method adds a cell to the open list that is already in the closed list. Closed cells should not be reopened." << gridNode->toString();
			        }
		        }
		    }
		}
	}
};

TEST(PathPlanning, expandNode) 
{
	const double epsilon = 1e-5;

	const char mapchar[101] =
			"# ##G     "
			"  ##      "
			"  ##      "
			"          "
			" ###  ##  "
			"   #  ### "
			"   ##     "
			"  ## #   S"
			"#      # #"
			" #      ##";
	std::vector<bool> data;
	makeMap(mapchar, data);


	GridMap map(10, 10, data);
	OpenList::map = &map;
	ManhattanDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);

	OpenListTest openList;
   	ClosedList closedList;

	GridNode *goal = GridNode::get(4, 0);
	//col,row
	openList.enqueue(GridNode::get(5, 9), 12.);
	(GridNode::get(5, 9))->costs = 2.;
	openList.enqueue(GridNode::get(5, 8), 10. + sqrt(2.));
	(GridNode::get(5, 8))->costs = 1. + sqrt(2.);
	openList.enqueue(GridNode::get(4, 7), 8. + 3. * sqrt(2.));
	(GridNode::get(4, 7))->costs = 1. + 3. * sqrt(2.);
	openList.enqueue(GridNode::get(7, 7), 10. + 2. * sqrt(2.));
	(GridNode::get(7, 7))->costs = 2. * sqrt(2.);
	openList.enqueue(GridNode::get(6, 6), 11.14);
	const double dummyVal = 10.;
	(GridNode::get(6, 6))->costs = dummyVal; //2. + sqrt(2.);
	const double oldOpenListValue66 = openList.getCosts(GridNode::get(6, 6));
	openList.enqueue(GridNode::get(4, 5), 6. + 3. * sqrt(2.));
	(GridNode::get(4, 5))->costs = 1. + 3. * sqrt(2.);
	openList.enqueue(GridNode::get(5, 5), 8. + 2. * sqrt(2.));
	(GridNode::get(5, 5))->costs = 2. + 2. * sqrt(2.);

	closedList.add(GridNode::get(6, 9));
	(GridNode::get(6, 9))->costs = 1.;
	closedList.add(GridNode::get(6, 8));
	(GridNode::get(6, 8))->costs = sqrt(2.);
	closedList.add(GridNode::get(6, 7));
	(GridNode::get(6, 7))->costs = 1. + sqrt(2.);
	closedList.add(GridNode::get(5, 6));
	(GridNode::get(5, 6))->costs = 1. + 2. * sqrt(2.);

	(GridNode::get(7, 6))->costs = 1. + 2. * sqrt(2.);
	planner.expandNode(GridNode::get(7, 6), goal, openList, closedList);

	if (openList.contains(GridNode::get(7, 6))) {
		FAIL() << "The method adds the node that is currently being expanded to the open list.";
	}
	if (openList.contains(GridNode::get(8,7)) == false || openList.contains(GridNode::get(8,6)) == false)
		FAIL() << "An expected expanded node is missing";
	openList.test(map, closedList);
	if (fabs(GridNode::get(8, 6)->costs) < epsilon)
		FAIL() << "The method does not set node->costs to the current costs from the start node";
	if (fabs(GridNode::get(8, 6)->costs - (2.0 + 2.0 * sqrt(2.0))) > epsilon) {
		FAIL() << "node->costs does not contain the correct costs of the path from the start node to the current node";
	}
	if (GridNode::get(8, 6)->getPredecessor() == NULL) {
		FAIL() << "The method does not set the predecessor of the newly added nodes";
	}
	if (GridNode::get(8, 6)->getPredecessor() != GridNode::get(7, 6)) {
		FAIL() << "The method does not set the predecessor of the newly added nodes to the correct predecessor node.";
	}
	if (GridNode::get(6, 7)->getPredecessor() != NULL) {
		FAIL() << "The method overwrites the predecessor of a node that has already been closed";
	}
	if (GridNode::get(7, 7)->getPredecessor() != NULL) {
		FAIL() << "The method overwrites the predecessor of a node on the open list that had a shorter path";
	}
	if (GridNode::get(6, 6)->getPredecessor() == NULL) {
		FAIL() << "The method does not overwrite the predecessor of a node on the open list that had a longer path";
	}


	if (fabs((GridNode::get(7, 7))->costs - (2. * sqrt(2.))) > epsilon)
		FAIL() << "Updated the cost of a node that is not supposed to be updated (i.e. the cost of that node should remain unchanged, because it is the minimum cost that can be achieved  at that moment)";

	
	if (fabs((GridNode::get(6, 6))->costs - dummyVal) < epsilon)
		FAIL() << "Did not update the cost of a node on the open list although a shorter path has been found";
	if (fabs((GridNode::get(6, 6))->costs - (2. + 2. * sqrt(2.))) > epsilon)
		FAIL() << "Updated the cost of a node on the open list with an incorrect cost value";
	if (fabs(openList.getCosts(GridNode::get(6, 6)) - oldOpenListValue66) < epsilon)
		FAIL() << "Did not update the cost value of the priority queue entry for a node on the open list";
	if (fabs(openList.getCosts(GridNode::get(6, 6)) - (10. + 2. * sqrt(2.))) > epsilon)
		FAIL() << "Updated the cost value of the priority queue entry for a node on the open list with an incorrect value";

	if (fabs((GridNode::get(8, 6))->costs - (2. + 2. * sqrt(2.))) > epsilon)
		FAIL() << "Costs are not updated correctly";

	if (fabs((GridNode::get(8, 7))->costs - (1. + 3. * sqrt(2.))) > epsilon)
		FAIL() << "Costs are not updated correctly";

	if (fabs(openList.getCosts(GridNode::get(8, 6))  - (2. + 2. * sqrt(2.))) < epsilon) {
		FAIL() << "The method inserts new nodes to the openList with costs g (= costs from start), but it should be f = g + h (costs so far plus heuristic to goal).";
	}
	if (fabs(openList.getCosts(GridNode::get(8, 6))  - (12. + 2. * sqrt(2.))) > epsilon){
		FAIL() << "The method inserts new nodes to the openList with invalid costs, they should be f = g + h (costs so far plus heuristic to goal).";
	}

	return;
 
}

TEST(PathPlanning, planPath) 
{
	const char mapchar[101] =
			"# ##      "
			"  ##      "
			"  ##      "
			"          "
			" ###  ##  "
			"   #  ### "
			"   ##    #"
			"  ## #    "
			"#      # #"
			" #      ##";
	std::vector<bool> data;
	makeMap(mapchar, data);
	
	GridMap map(10, 10, data);
	OpenList::map = &map;
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);

	OpenList openList;
   	ClosedList closedList;

	GridNode *goal = GridNode::get(0, 4);
	GridNode *start = GridNode::get(7, 9);

	std::deque<const AbstractNode*> path = planner.planPath(start,goal);
	std::deque<const AbstractNode*> shortestPath;

	if (path.empty()) {
		FAIL() << "The method returned an empty path.";
	}

	bool foundStart = false, foundGoal = false;
	for (size_t i = 0; i < path.size(); ++i) {
		if (path[i] == start) {
			if (i == 0) {
				foundStart = true;
			} else  {
				FAIL() << "The path contains the start node, but it is not the first element of the path.";
			}
		}
		if (path[i] == goal) {
			if (i == path.size() - 1) {
				foundGoal = true;
			} else {
				FAIL() << "The path contains the goal node, but it is not the last element of the path.";
			}
		}
	}
	if (!foundStart)
		FAIL() << "The start node is not included in the path";
	if (!foundGoal)
		FAIL() << "The goal node is not included in the path";

	if (path.size()>10)
		FAIL()<<"The path is longer than expected";
	else if (path.size()<10)
		FAIL()<<"The path is shorter than expected";

	int lx = start->x, ly = start->y;
	double length = 0.0;
	for (size_t i = 0; i < path.size(); ++i) {
		const int x = static_cast<const GridNode*>(path[i])->x;
		const int y = static_cast<const GridNode*>(path[i])->y;
		if (map.isOccupied(x, y)) {
			FAIL() << "The planned path leads through an occupied cell.";
		}
		const double dx = static_cast<double>(x - lx);
		const double dy = static_cast<double>(y - ly);
		length += sqrt(dx * dx + dy * dy);
		lx = x;
		ly = y;
	}
	const double optLength = (6. + 3. * sqrt(2.));
	if (length - optLength > 0.1) {
		FAIL() << "The planned path is suboptimal. It has length " << length << ", but the shortest possible path has length " << optLength;
	}
	
	return;
 
}
TEST(PathPlanning, followPath) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, heuristic);

	GridNode *a = GridNode::get(5, 1);
	GridNode *b = GridNode::get(2, 2);
	GridNode *c = GridNode::get(2, 5);
	GridNode *d = GridNode::get(3, 2);

	ASSERT_NO_THROW(a->setPredecessor(b));
	ASSERT_NO_THROW(b->setPredecessor(c));
	ASSERT_NO_THROW(c->setPredecessor(d));

	std::deque<const AbstractNode *> path = planner.followPath(a);

	if (path.empty()) {
		FAIL() << "The method returns an empty path.";
	}

	for (size_t i = 0; i < path.size(); ++i) {
		if (!path[i]) {
			FAIL() << "The method returns a path that contains a NULL pointer.";
		}
	}

	if (path.size() == 1) {
		if (path[0] == a) {
			FAIL() << "The method returns only the current node instead of the complete path.";
		} else if (path[0] == d) {
			FAIL() << "The method returns only the start node instead of the complete path.";
		} else {
			FAIL() << "The method returns only a single node instead of the complete path.";
		}
	} else if (path.size() < 4) {
		FAIL() << "The method returns a path that is too short.";
	} else if (path.size() > 4) {
		FAIL() << "The method returns a path that is too long.";
	}
	if (path.size() == 4 && path[0] == d && path[1] == c && path[2] == b && path[3] == a) {
		return;
	} else if (path.size() == 4 && path[0] == a && path[1] == b && path[2] == c && path[3] == d) {
		FAIL() << "The method returns the path in reverse order (from the goal to the start).";
	} else {
		FAIL() << "The method returns the nodes in incorrect order.";
	}
}


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

