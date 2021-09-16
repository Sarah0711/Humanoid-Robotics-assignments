#include <path_planning/OpenList.h>
#include <iostream>

namespace path_planning {

FileIO *OpenList::fileIO = NULL;
bool OpenList::logToStdout = false;

const GridMap *OpenList::map = NULL;
OpenList::OpenList() :
		duplicateWarning(false),
		reinsertWarning(false),
		data(map->width * map->height)
{
}

OpenList::~OpenList() {
}

OpenList::OpenListData& OpenList::getData(AbstractNode * node) {
	GridNode * g = dynamic_cast<GridNode *>(node);
	if (!g) {
		throw std::invalid_argument("Used OpenList with a node that is not an instance of GridNode");
	}
	const int c = g->y * map->width + g->x;
	if (c < 0 || c > data.size()) {
		throw std::invalid_argument("Used OpenList with a grid node that is outside the map");
	}
	return data[c];
}

OpenList::OpenListData& OpenList::getData(const AbstractNode * const node) const {
	const GridNode * const g = dynamic_cast<const GridNode * const>(node);
	if (!g) {
		throw std::invalid_argument("Used OpenList with a node that is not an instance of GridNode");
	}
	const int c = g->y * map->width + g->x;
	if (c < 0 || c > data.size()) {
		throw std::invalid_argument("Used OpenList with a grid node that is outside the map");
	}
	return data[c];
}

void OpenList::enqueue(const AbstractNode * const node, const double costs) {
	if (!node) {
		throw std::invalid_argument("OpenList::enqueue(): node is NULL");
	}
	if (fileIO) {
		fileIO->logOpenNode(node);
	} else if (logToStdout) {
		std::cout << "enqueue " << node->toString() << std::endl;
	}
	OpenList::OpenListData& d = getData(node);
	if (!duplicateWarning && d.state == OpenListData::OPEN) {
		std::cerr << "Warning: adding node " << node->toString()
				<< " multiple times to the open list. Use updateCosts() instead for changing the costs of a node."
				<< std::endl;
		duplicateWarning = true;
	}
	if (!reinsertWarning && d.state == OpenListData::CLOSED) {
		std::cerr << "Warning: re-adding node " << node->toString()
				<< " that has already been removed from the open list."
				<< std::endl;
			reinsertWarning = true;
	}
	d.state = OpenListData::OPEN;
	d.cost = costs;
	NodeWrapper *wrapper = new NodeWrapper(node, costs);
	openList.push(wrapper);
}
const AbstractNode * OpenList::removeMin() {
	while (!openList.empty()) {
		NodeWrapper *wrapper = openList.top();
		const AbstractNode *node = wrapper->node;
		openList.pop();
		delete wrapper;
		OpenList::OpenListData& d = getData(node);
		if (d.state == OpenListData::OPEN) {
			d.state = OpenListData::CLOSED;
			return node;
		}
		// here updated nodes are deleted (lazy deletion)
	}
	throw std::runtime_error("OpenList::removeMin() is called but the queue is empty");
}
void OpenList::updateCosts(const AbstractNode * const node, const double costs) {
	if (!node) {
		throw std::invalid_argument("OpenList::updateCosts(): node is NULL");
	}
	OpenList::OpenListData& d = getData(node);
	if (d.state != OpenListData::OPEN) {
		throw std::runtime_error(
				"Tried to update the costs for a node that has not been enqueued to the open list.");
	}
	d.cost = costs;
	// lazy deletion: insert copy now, delete later
	NodeWrapper *wrapper = new NodeWrapper(node, costs) ;
	openList.push(wrapper);
}

double OpenList::getCosts(const AbstractNode * const node) const {
	if (!node) {
		throw std::invalid_argument("OpenList::getCosts(): node is NULL");
	}
	const OpenList::OpenListData& d = getData(node);
	if (d.state != OpenListData::OPEN) {
		throw std::runtime_error(
				"Tried to get the costs for a node that has not been enqueued to the open list.");
	}
	return d.cost;
}

bool OpenList::isEmpty() const {
	// do lazy deletion
	while(!openList.empty() && getData(openList.top()->node).state == OpenListData::CLOSED) {
		openList.pop();
	}
	return openList.empty();
}
bool OpenList::contains(const AbstractNode * const node) const {
	if (!node) {
		throw std::invalid_argument("OpenList::contains(): node is NULL");
	}
	return getData(node).state == OpenListData::OPEN;
}

}  // namespace planning
