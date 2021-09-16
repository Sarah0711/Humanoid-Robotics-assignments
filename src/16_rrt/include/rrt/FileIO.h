#ifndef RRT_FILEIO_H_
#define RRT_FILEIO_H_

#include <fstream>
#include <deque>
#include <rrt/AbstractNode.h>
#include <rrt/GridNode.h>
#include <rrt/GridMap.h>

namespace rrt {

/**
 * @brief Helper class for loading maps and saving log files.
 */
class FileIO {
public:
        FileIO();
        virtual ~FileIO();
        /**
         * @brief Opens a log file.
         * @param filename Filename of the log file.
         * @return True iff the file was opened successfully.
         */
        bool openLogfile(const std::string& filename);
        /**
         * @brief Logs a path to a file.
         * @param filename Filename of the log file.
         * @param path The path from a start node to a goal node.
         */
        void logPath(const std::string& filename, const std::deque<AbstractNode *>& path);
        /**
         * @brief Logs an extend step to the log file previously opened with openLogfile().
         * @param from From node.
         * @param to To node.
         * @param randomNode Random node.
         * @param listName Name of the tree.
         */
        void logExtend(const GridNode * const from, const GridNode * const to, const GridNode * const randomNode, const std::string * const listName);
        /**
         * @brief Logs a failed extend step (= dead end) to the log file previously opened with openLogfile().
         * @param from From node.
         * @param randomNode Random node.
         * @param listName Name of the tree.
         */
        void logFailedExtend(const GridNode * const from, const GridNode * const randomNode, const std::string * const listName);
        /**
         * @brief Loads a map from a file.
         * @param filename The filename of the map.
         * @return Pointer to the grid map loaded from the given file, or NULL in case of failure.
         */
        static const GridMap * loadMap(const std::string& filename);

private:
        std::ofstream logfile;
};

} /* namespace rrt */

#endif /* RRT_FILEIO_H_ */
