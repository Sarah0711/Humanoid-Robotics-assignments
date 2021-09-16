#include <footstep_planning/FileIO.h>
#include <iostream>
#include <cstring>

namespace footstep_planning {

FileIO::FileIO(const std::string& package_path) {
	const size_t angleStep = 15;
	const std::string filename = package_path + "/data/map.pbm";
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open map file " << filename << std::endl;
		return;
	}
	std::string header;
	ifs >> header;
	if (header != "P1") {
		std::cerr << "Error: map file is not in PBM format" << std::endl;
		return;
	}

	int width = 0, height = 0;
	ifs >> width >> height;
	if (width > 1000 || height > 1000) {
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

	std::vector<std::vector<unsigned char> > distanceMaps(180 / angleStep);

	const size_t bufferLength = package_path.length() + 25;
	char *buffer = (char *) calloc(bufferLength, sizeof(char));
	memset(buffer, 0, bufferLength);
	for (size_t angle = 0; angle < 180; angle += angleStep) {
		size_t i = angle / angleStep;
		snprintf(buffer, bufferLength, "%s/data/distance_%zu.pgm", package_path.c_str(), angle);
		ifs.open(buffer, std::ios::binary);
		if (!ifs.good()) {
			std::cerr << "Could not load " << buffer << " for reading distance map." << std::endl;
			return;
		}
		ifs >> header;
		if (header != "P5") {
			std::cerr << "Error: distance map file is not in PGM format" << std::endl;
			return;
		}

		int nwidth, nheight, depth;
		ifs >> nwidth >> nheight >> depth;
		if (width != nwidth || height != nheight) {
			std::cerr << "Error: distance map does not have same dimensions as map" << std::endl;
			return;
		}
		if (depth != 255) {
			std::cerr << "Error: distance map has invalid depth " << depth << std::endl;
			return;
		}

		distanceMaps[i].reserve(width * height);
		ifs.get(c); // line break
		while (ifs.get(c)) {
			distanceMaps[i].push_back(c);
		}
		ifs.close();
		if (distanceMaps[i].size() != static_cast<size_t>(width * height)) {
			std::cerr << "Error: distance map has " << distanceMaps[i].size() << " elements, but expected " << width << " x " << height << " = " << width * height << " elements." << std::endl;
			return;
		}
	}
	free(buffer);
	buffer = NULL;

	map = new FootstepMap(width, height, data, distanceMaps, 0.01, 5.0);
}

FileIO::~FileIO()  {
};

void FileIO::logPath(const std::string& filename, std::deque<const AbstractNode*> path) {
	std::ofstream ofs(filename.c_str());
	if (ofs.good()) {
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			if (!*it) {
				throw std::runtime_error("Path contains a NULL pointer");
			}
			ofs << (*it)->toLogString() << std::endl;
		}
	} else {
		std::cerr << "Error: Could not open " << filename << " for writing the path." << std::endl;
	}
	ofs.close();
}

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

} /* namespace footstep_planning */

