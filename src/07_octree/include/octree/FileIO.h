#ifndef OCTREE_FILEIO_H_
#define OCTREE_FILEIO_H_

#include <string>
#include <fstream>
#include <octree/Octree.h>

namespace octree {

/**
 * \brief Write octree data to a file.
 */
class FileIO {
public:
	FileIO() : numWritten(0) {};
	virtual ~FileIO() {};
	/**
	 * Write an octree to a file.
	 * @param oc Octree.
	 * @param filename Name of the output file.
	 * @return True if the file was written successfully.
	 */
	bool writeToFile(const Octree& oc, const std::string &filename);
	/**
	 * Returns the number of written nodes after saving an octree to a file using writeToFile().
	 * @return Number of nodes.
	 */
	inline const int& getNumWritten() { return numWritten; }

private:
	void writeNode(std::ofstream& ofs, const Node * const node);
	int numWritten;
};


}  // namespace octree


#endif  // OCTREE_FILEIO_H_
