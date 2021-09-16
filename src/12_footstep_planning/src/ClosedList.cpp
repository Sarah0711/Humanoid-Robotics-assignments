#include <footstep_planning/ClosedList.h>
#include <footstep_planning/FileIO.h>
#include <footstep_planning/AbstractNode.h>
#include <footstep_planning/FileIO.h>
#include <iostream>

namespace footstep_planning {

bool ClosedList::logToStdout = false;

ClosedList::ClosedList() : duplicateWarning(false) {
}

ClosedList::~ClosedList() {
}


void ClosedList::add(const AbstractNode * const node) {
	if (!node) {
		throw std::invalid_argument("ClosedList::add() called with NULL argument");
	}
	if (logToStdout) {
		std::cout << "close " << node->toString() << std::endl;
	}
	if (!duplicateWarning && list.find(node) != list.end()) {
		std::cerr << "Warning: adding node " << node->toString()
				<< " multiple times to the closed list." << std::endl;
		duplicateWarning = true;
	}
	list.insert(node);
}
bool ClosedList::contains(const AbstractNode * const node) const {
	if (!node) {
		throw std::invalid_argument("ClosedList::contains() called with NULL argument");
	}
	return list.find(node) != list.end();
}

}  // namespace footstep_planning
