#include <footstep_planning/OpenList.h>
#include <iostream>

namespace footstep_planning {

bool OpenList::logToStdout = false;

OpenList::OpenList() :
		duplicateWarning(false),
		reinsertWarning(false) {
}

OpenList::~OpenList() {
}


void OpenList::enqueue(const AbstractNode * const node, const double costs) {
	if (!node) {
		throw std::invalid_argument("OpenList::enqueue(): node is NULL");
	}
	if (logToStdout) {
		std::cout << "enqueue " << node->toString() << std::endl;
	}
	if (!duplicateWarning && node->openListData->state == OpenListData::OPEN) {
		std::cerr << "Warning: adding node " << node->toString()
				<< " multiple times to the open list. Use updateCosts() instead for changing the costs of a node."
				<< std::endl;
		duplicateWarning = true;
	}
	if (!reinsertWarning && node->openListData->state == OpenListData::CLOSED) {
		std::cerr << "Warning: re-adding node " << node->toString()
				<< " that has already been removed from the open list."
				<< std::endl;
			reinsertWarning = true;
	}
	node->openListData->state = OpenListData::OPEN;
	node->openListData->cost = costs;
	NodeWrapper *wrapper = new NodeWrapper(node, costs);
	openList.push(wrapper);
}
const AbstractNode * OpenList::removeMin() {
	while (!openList.empty()) {
		NodeWrapper *wrapper = openList.top();
		const AbstractNode *node = wrapper->node;
		openList.pop();
		delete wrapper;
		if (node->openListData->state == OpenListData::OPEN) {
			node->openListData->state = OpenListData::CLOSED;
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
	if (node->openListData->state != OpenListData::OPEN) {
		throw std::runtime_error(
				"Tried to update the costs for a node that has not been enqueued to the open list.");
	}
	// lazy deletion: insert copy now, delete later
	NodeWrapper *wrapper = new NodeWrapper(node, costs) ;
	openList.push(wrapper);
}

double OpenList::getCosts(const AbstractNode * const node) const {
	if (!node) {
		throw std::invalid_argument("OpenList::getCosts(): node is NULL");
	}
	if (node->openListData->state != OpenListData::OPEN) {
		throw std::runtime_error(
				"Tried to get the costs for a node that has not been enqueued to the open list.");
	}
	return node->openListData->cost;
}

bool OpenList::isEmpty() const {
	// do lazy deletion
	while(!openList.empty() && openList.top()->node->openListData->state == OpenListData::CLOSED) {
		openList.pop();
	}
	return openList.empty();
}
bool OpenList::contains(const AbstractNode * const node) const {
	if (!node) {
		throw std::invalid_argument("OpenList::contains(): node is NULL");
	}
	return node->openListData->state == OpenListData::OPEN;
}

}  // namespace planning
