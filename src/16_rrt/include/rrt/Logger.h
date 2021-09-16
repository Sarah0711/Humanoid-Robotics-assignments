#ifndef INCLUDE_RRT_LOGGER_H_
#define INCLUDE_RRT_LOGGER_H_

#include <rrt/RRT.h>
#include <rrt/FileIO.h>

namespace rrt {
/**
 * @brief Helper subclass of RRTGrid that logs node expansions to a file.
 */
class RRTWithLoggers : public RRTGrid {
public:
	/**
	 * @brief Constructor.
	 * @param fileIO The FileIO instance for logging.
	 * @param map The grid map.
	 * @param startNode The start node.
	 * @param goalNode The goal node.
	 */
	RRTWithLoggers(FileIO * const fileIO, const GridMap& map, const GridNode * const startNode, const GridNode * const goalNode);
	virtual ~RRTWithLoggers();
	/**
	 * Returns the number of extended nodes.
	 * @return Number of extended nodes.
	 */
	inline const size_t& getNumExtendedNodes() const { return numExtendedNodes; }

private:
	FileIO * const fileIO;
	const std::string listNameForward;
	const std::string listNameBackward;
	mutable size_t numExtendedNodes;
	const GridNode * const startNode;
	const GridNode * const goalNode;

	/**
	 * @copydoc RRTGrid::extendClosestNode
	 */
	virtual ExtendStepReturnValue extendClosestNode(AbstractNode * const randomNode,
			std::vector<AbstractNode *> & list, const std::vector<AbstractNode *> & otherList);
};

}  // namespace rrt

#endif /* INCLUDE_RRT_LOGGER_H_ */
