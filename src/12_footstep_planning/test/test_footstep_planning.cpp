#include <gtest/gtest.h>
#include <footstep_planning/FootstepPlanning.h>

using namespace footstep_planning;

class DummyFootstepPlanning : public FootstepPlanning {
public:
	DummyFootstepPlanning() : FootstepPlanning(NULL), distanceToNearestObstacle(0.0) {};
	virtual ~DummyFootstepPlanning() {};

	double getDistanceToNearestObstacle(const FootstepNode * const /*step*/) const {
		return distanceToNearestObstacle;
	}
	bool isColliding(const FootstepNode * const /*step*/) {
		return distanceToNearestObstacle < 0.2;
	}
	void setDistanceToNearestObstacle(const double& distance) {
		distanceToNearestObstacle = distance;
	}

private:
	double distanceToNearestObstacle;

};

TEST(FootstepPlanning, getCosts) {
	const double tolerance = 1e-5;
	DummyFootstepPlanning ofp;
	const FootstepNode * const node1 = FootstepNode::get(2.0, 3.0, M_PI/2, LEFT);
	const FootstepNode * const node3 = FootstepNode::get(3.0, 4.0, M_PI/2, RIGHT);
	double result = ofp.getCosts(node1, node1);
	if (!std::isinf(result)) {
		FAIL() << "The costs for stepping with the same foot twice in a row should be infinite, otherwise the robot may go beyond its physical capabilities. getCosts() returns " << result << " instead.";
	}
	ofp.setDistanceToNearestObstacle(std::numeric_limits<double>::max());
	result = ofp.getCosts(node1, node3);
	if (fabs(result - sqrt(2.0)) > tolerance) {
		FAIL() << "The method uses incorrect Euclidean costs" << std::endl;
	}
}

TEST(FootstepPlanning, heuristic) {
	const FootstepHeuristic h;
	const FootstepNode * const currentNode = FootstepNode::get(2.0, 3.0, M_PI/2, LEFT);
	const FootstepNode * const goalNode = FootstepNode::get(3.0, 4.0, M_PI/2, RIGHT);
	if (h.heuristic(currentNode, goalNode) == 0.0) {
		FAIL() << "The method returns zero.";
	}
	ASSERT_DOUBLE_EQ(sqrt(2.0), h.heuristic(currentNode, goalNode));
}

TEST(FootstepPlanning, executeFootstep) {
	const double tolerance = 1e-5;
	DummyFootstepPlanning ofp;
	const FootstepNode * const currentFootstep1 = FootstepNode::get(2.0, 3.0, 0, LEFT);
    const FootstepNode * const currentFootstep2 = FootstepNode::get(2.0, 3.0, M_PI/2, LEFT);
    const FootstepPlanning::FootstepAction action1(4.0, 5.0, 0, RIGHT);
    const FootstepPlanning::FootstepAction action2(4.0, 5.0, M_PI/2, RIGHT);
    const FootstepNode * const result11 = ofp.executeFootstep(currentFootstep1, action1);
    //const FootstepNode * const result12 = ofp.executeFootstep(currentFootstep1, action2);
    const FootstepNode * const result21 = ofp.executeFootstep(currentFootstep2, action1);
    const FootstepNode * const result22 = ofp.executeFootstep(currentFootstep2, action2);

    if (result11->x == 0.0 && result11->y == 0.0 && result11->theta == 0.0 && result22->x == 0.0 && result22->y == 0.0 && result22->theta == 0.0) {
    	FAIL() << "The method does not do anything.";
    }
    if (fabs(result11->x - 6.0) > tolerance || fabs(result11->y - 8.0) > tolerance) {
    	FAIL() << "The method does not calculate the translation correctly even if all angles are zero.";
    }
    if (fabs(result21->x - 6.0) < tolerance && fabs(result21->y - 8.0) < tolerance) {
    	FAIL() << "The method does not apply the translation correctly if the robot's current orientation is non-zero.";
    }

    if (fabs(result22->x - 6.0) < tolerance && fabs(result22->y - 8.0) < tolerance) {
    	FAIL() << "The method adds delta_x to x and delta_y to y without respecting the current orientation of the robot. The footstep actions are given from the robot's point of view, not in the global frame.";
    }
    if (fabs(result22->x - (-2.0)) < tolerance && fabs(result22->y - (-2.0)) < tolerance) {
    	FAIL() << "The method applies the translation with respect to the robot's new orientation instead of the old orientation. First apply the translation, then calculate the new rotation.";
    }
    std::cout<<"result22 x="<<result22->x <<" y="<<result22->y<<std::endl;

    if (fabs(result22->x - 7.0) > tolerance) {
    	FAIL() << "x is incorrect.";
    }
    if (fabs(result22->y - (-1.0)) > tolerance) {
    	FAIL() << "y is incorrect.";
    }
    if (fabs(result22->theta - M_PI) > tolerance) {
    	FAIL() << "theta is incorrect.";
    }
}

TEST(FootstepPlanning, getNeighborNodes) {
	DummyFootstepPlanning dfp;
	dfp.setDistanceToNearestObstacle(2.0);
	const FootstepNode * const currentFootstep = FootstepNode::get(2.0, 3.0, M_PI, LEFT);
	std::vector<AbstractNode* > result = dfp.getNeighborNodes(currentFootstep);
	if (result.empty()) {
		FAIL() << "The method returns an empty vector.";
	}

	struct FS {
		const double x, y, theta;
		bool found;
	} expectedFootsteps[12] = {
		{2.00,  2.84, M_PI,       false},
		{1.92,  2.91, M_PI,       false},
		{2.04,  2.91, M_PI,       false},
		{2.00,  2.88, M_PI,       false},
		{1.95,  2.86, M_PI,       false},
		{1.99,  2.87, M_PI - 0.5, false},
		{1.985, 2.90, M_PI + 0.5, false},
		{1.96,  2.88, M_PI + 0.3, false},
		{2.03,  2.88, M_PI + 0.5, false},
		{1.94,  2.88, M_PI,       false},
		{1.96,  2.90, M_PI,       false},
		{2.02,  2.88, M_PI,       false}
	};

	for (size_t i = 0; i < result.size(); ++i) {
		const FootstepNode *footstep = static_cast<FootstepNode *>(result[i]);
		if (footstep->foot != RIGHT) {
			FAIL() << "The method returns a foot step for the wrong foot.";
		}
		bool found = false;
		for (size_t j = 0; j < 12; ++j) {
			if (fabs(footstep->x - expectedFootsteps[j].x) < 0.01 && fabs(footstep->y - expectedFootsteps[j].y) < 0.01 &&
					fabs(angles::shortest_angular_distance(footstep->theta, expectedFootsteps[j].theta)) < 0.01) {
				if (expectedFootsteps[j].found) {
					FAIL() << "The method returns the same footstep twice.";
				} else {
					expectedFootsteps[j].found = true;
				}
				found = true;
				break;
			}
		}
		if (!found) {
			std::cout << footstep->x << " " << footstep->y << " " << footstep->theta << std::endl;
			FAIL() << "The method returns a footstep that is not reachable from the current footstep.";
		}
	}
	for (size_t j = 0; j < 12; ++j) {
		if (!expectedFootsteps[j].found) {
			FAIL() << "A feasible footstep is missing from the result vector.";
		}
	}

	dfp.setDistanceToNearestObstacle(0.0);
	result = dfp.getNeighborNodes(currentFootstep);
	if (!result.empty()) {
		FAIL() << "The method returns footsteps that collide with obstacles. Use the method isColliding(FootstepNode *node) to check for collisions.";
	}
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
