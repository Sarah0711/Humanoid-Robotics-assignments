#include <ara_star/FileIO.h>
#include <iostream>

namespace ara_star {

FileIO::FileIO() {}

FileIO::~FileIO()  {
};

const GridMap * FileIO::loadMap(const std::string& filename) {
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open map file " << filename << std::endl;
		return NULL;
	}
	std::string header;
	ifs >> header;
	if (header != "P1") {
		std::cerr << "Error: map file is not in PBM format" << std::endl;
		return NULL;
	}

	size_t width = 0, height = 0;
	ifs >> width >> height;
	if (width > 100 || height > 100) {
		throw std::runtime_error("Map is too big");
	}

	std::vector<bool> data;
	data.reserve(width * height);
	char c;
	while (ifs.get(c)) {
		switch (c) {
		case '0': data.push_back(false); break;
		case '1': data.push_back(true); break;
		default: //ignore
			break;
		}
	}

	ifs.close();

	return new GridMap(width, height, data);
}

} /* namespace ara_star */

