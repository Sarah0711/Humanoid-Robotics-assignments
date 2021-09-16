#ifndef IRM_FILEIO_H_
#define IRM_FILEIO_H_

#include <string>
#include <irm/IRM.h>

namespace irm {

/**
 * @brief Helper class for writing maps to log files.
 */
class FileIO {
public:
	/**
	 * @brief Constructor.
	 * @param packagePath The path to the package's root directory.
	 */
	FileIO(const std::string& packagePath);
	virtual ~FileIO();

	/**
	 * @brief Write the reachability map to a file.
	 * @param irm The IRM instance.
	 */
	void writeRM(const IRM& irm);
	/**
	 * @brief Write the inverse reachability map to a file.
	 * @param irm The IRM instance.
	 */
	void writeIRM(const IRM& irm);

private:
	const std::string& packagePath;
	template<typename MapType>
	void writeHelper(const std::vector<IRM::MapConfig>& mapConfig, const IRM& irm, const MapType& rm, std::ofstream& ofs);
};

} /* namespace irm */

#endif /* IRM_FILEIO_H_ */
