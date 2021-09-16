#define NOMINMAX
#include <gtest/gtest.h>
#include <ara_star/ARAStar.h>
#include <math.h>

using namespace ara_star;

#ifdef _WIN32
    #include <windows.h>

    void msleep(unsigned milliseconds)
    {
        Sleep(milliseconds);
    }
#else
    #include <unistd.h>

    void msleep(unsigned milliseconds)
    {
        usleep(milliseconds * 1000); // takes microseconds
    }
#endif

TEST(ARAStar, heuristic) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	ARAStarHeuristic h;
	ARAStarPlanning planner(map, h);

	h.setW(0.0);
	{
		const double res = planner.heuristic(GridNode::get(0, 0), GridNode::get(0, 1));
		if (fabs(res) > 1e-5) {
			FAIL() << "For w = 0 the heuristic should always be 0, but heuristic((0,0), (0,1)) returned " << res << ".";
		}
	}

	h.setW(1.0);
	{
		const double res = planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7));
		if (fabs(res - 5.0) > 1e-5) {
			FAIL() << "For w = 1 the heuristic should return the Euclidean costs, but heuristic((2,3), (5,7)) returned " << res << " instead of 5.";
		}
	}

	h.setW(3.0);
	{
		const double res1 = planner.heuristic(GridNode::get(0, 0), GridNode::get(0, 1));
		const double res2 = planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7));
		if (fabs(res1 - 1.0) < 1e-5 && fabs(res2 - 5.0) < 1e-5) {
			FAIL() << "The heuristic returns the Euclidean distance without considering w.";
		}
	}
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(0, 0), GridNode::get(0, 1)), 3.0) << " for w = 3.0";
	EXPECT_DOUBLE_EQ(planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7)), 15.0) << " for w = 3.0";
}

class ARAStarHeuristicDummy : public ARAStarHeuristic {
public:
	std::vector<double> wHistory;

	virtual void setW(const double& w) {
		if (getW() != w) {
			wHistory.push_back(w);
		}
		ARAStarHeuristic::setW(w);
	}
};

class ARAStarDummy : public ARAStarPlanning {
public:
	ARAStarDummy(const GridMap& map, ARAStarHeuristic& heuristic) : ARAStarPlanning(map, heuristic), lastW(-1), sleepTime(1), heuristic_(heuristic) {}
	virtual ~ARAStarDummy() {}

	virtual std::deque<const AbstractNode*> planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode,
			const time_t & /*tstart*/,const double& /*timeLimit*/) {
		if (lastW < 0 && fabs(heuristic_.getW()) < 1e-5) {
			throw std::runtime_error("The method did not set w to the initial value.");
		}
		if (lastW >= 0 && lastW < heuristic_.getW()) {
			std::stringstream ss;
			ss << "w increased from " << lastW << " to " << heuristic_.getW() << ", but it should only decrease.";
			throw std::runtime_error(ss.str());
		}
		if (lastW >= 0 && (lastW - heuristic_.getW()) < 1e-3) {
			throw std::runtime_error("planPath() was called twice with the same w value.");
		}
		if (lastW >= 0 && heuristic_.getW() < 1.0) {
			std::stringstream ss;
			ss << "The method calls planPath with 0 < w < 1.0, which does not make sense. Stop when w reaches 1.0.";
		}
		lastW = heuristic_.getW();

		msleep(sleepTime);

		// dummy result
		std::deque<const AbstractNode*> result;
		result.push_back(startNode);
		result.push_back(goalNode);
		return result;
	}

	double lastW;
	unsigned sleepTime;

private:
	ARAStarHeuristic& heuristic_;
};

TEST(ARAStar, runARA) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	OpenList::map = &map;
	ARAStarHeuristicDummy h;
	ARAStarDummy planner(map, h);
	const double wInitial = 3.0;
	const double wDelta = 0.5;
	double expectedW[] = {3.0, 2.5, 2.0, 1.5, 1.0};
	const size_t N = 5;
	const double timeLimitInfinite = std::numeric_limits<double>::max();
	const double timeLimit = 2;
	std::deque<const AbstractNode *> path;
	try {
		path = planner.runARA(wInitial, wDelta, timeLimitInfinite, GridNode::get(2, 3), GridNode::get(5, 7));
	} catch (const std::runtime_error& e) {
		FAIL() << e.what();
	}
	if (h.wHistory.empty()) {
		FAIL() << "The method never calls heuristic_.setW()";
	} else {
		if (fabs(h.wHistory[0] - wInitial) > 1e-5) {
			FAIL() << "The method initializes w to " << h.wHistory[0] << " instead of wInitial.";
		}
		for (size_t i = 0; i < N && i < h.wHistory.size(); ++i) {
			if (fabs(h.wHistory[i] - expectedW[i]) > 1e-5) {
				std::stringstream err;
				err << "The method sets w to the sequence [";
				for (size_t j = 0; j <= i; ++j) {
					err << h.wHistory[j];
					if (j < i)
						err << ", ";
				}
				err <<", ...], but the expected sequence is [";
				for (size_t j = 0; j <= i; ++j) {
					err << expectedW[j];
					if (j < i)
						err << ", ";
				}
				err << ", ...].";
				FAIL() << err.str();
				break;
			}
		}
	}
	if (path.empty()) {
		FAIL() << "The method returns an empty path although the planner found a valid path.";
	}

	h.wHistory.clear();
	planner.sleepTime = 1000;
	planner.lastW = -1;
	try {
		path = planner.runARA(wInitial, wDelta, timeLimit, GridNode::get(2, 3), GridNode::get(5, 7));
	} catch (const std::runtime_error& e) {
		FAIL() << e.what();
	}

	if (h.wHistory.size() > 3) {
		FAIL() << "The method does not stop after the timeout is reached.";
	}
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

