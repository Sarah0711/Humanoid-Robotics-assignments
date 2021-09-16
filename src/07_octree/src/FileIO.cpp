#include <octree/FileIO.h>
#include <fstream>
#include <iostream>

namespace octree {


bool FileIO::writeToFile(const Octree& oc, const std::string& filename) {
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open file " << filename << " for writing results" << std::endl;
		return false;
	}

	numWritten = 0;
	writeNode(ofs, oc.root);

	ofs.close();
	return true;
}

void FileIO::writeNode(std::ofstream& ofs, const Node *node) {
	if (!node || numWritten > 1000) {
		return;
	}
	ofs << node->corner1(0) << " " << node->corner1(1) << " " << node->corner1(2) << " " <<
		   node->corner2(0) << " " << node->corner2(1) << " " << node->corner2(2) << " " <<
		   node->depth << " ";
	switch (node->content) {
	case FREE: ofs << "FREE"; break;
	case OCCUPIED: ofs << "OCCUPIED"; break;
	case MIXED: ofs << "MIXED"; break;
	default: ofs << "UNKNOWN"; break;
	}
	ofs << std::endl;
	++numWritten;
	for (size_t i = 0; i < 8; ++i) {
		writeNode(ofs, node->children[i]);
	}
}

}  // namespace octree
