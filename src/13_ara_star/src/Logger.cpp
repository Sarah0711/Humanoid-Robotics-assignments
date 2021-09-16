#include <ara_star/Logger.h>

namespace ara_star {

void ARAStarHeuristicLog::setW(const double& w) {
		ARAStarHeuristic::setW(w);
		wHistory.push_back(w);
};

ARAStarPlanningLog::ARAStarPlanningLog(const GridMap& map, ARAStarHeuristicLog& heuristic)
: ARAStarPlanning(map,heuristic),mapHistory(map), heuristic(heuristic)
{
		std::vector<int> row(mapHistory.width,0);
		for(size_t r=0;r<mapHistory.height;r++)
			mapLog.push_back(row);

		for(size_t r=0;r<mapHistory.height;r++)
			for(size_t c=0;c<mapHistory.width;c++)
				mapLog.at(r).at(c) = mapHistory.isOccupied(static_cast<int>(c),static_cast<int>(r));


	}

std::deque<const AbstractNode*> ARAStarPlanningLog::planPath(const ara_star::AbstractNode * const startNode,
		const AbstractNode * const goalNode, const time_t & tstart, const double& timeLimit){
		PathLengthData data;
		data.start = clock();
		std::deque<const AbstractNode*> resultPath = ARAStarPlanning::planPath(startNode, goalNode,tstart,timeLimit);
		data.end = clock();
		data.pathLength = resultPath.size();
		pathLengthList.push_back(data);

		std::string mapFileName,pathFileName;
		char wString[10];
		snprintf(wString, sizeof(wString), "%.1f", heuristic.wHistory[heuristic.wHistory.size()-1]);
		mapFileName = std::string(PROJECT_SOURCE_DIR) + "/data/GridMap_"+wString+".txt";
		pathFileName = std::string(PROJECT_SOURCE_DIR) +"/data/Path_"+wString+".txt";


		std::ofstream mapFile (mapFileName.c_str());
		if (mapFile.is_open()){
			for(size_t r=0;r<mapHistory.height;r++){
				for(size_t c=0;c<mapHistory.width;c++)
				    	mapFile << mapLog.at(r).at(c)<<" ";
				mapFile << "\n";
			}
			mapFile.close();
		}

		std::ofstream pathFile (pathFileName.c_str());
		if (pathFile.is_open()) {
			for(size_t i=0;i<resultPath.size();++i) {
				pathFile<<((GridNode*)resultPath[i])->x<<" "<<((ara_star::GridNode*)resultPath[i])->y<<"\n";
			}
			pathFile.close();
		}

		for(size_t r=0;r<mapHistory.height;r++){
			for(size_t c=0;c<mapHistory.width;c++) {
				if (mapLog.at(r).at(c)==2) {
					mapLog.at(r).at(c) = 0;
				}
			}
		}
		return resultPath;
	}

	void ARAStarPlanningLog::expandNode(const AbstractNode * const currentNode, const ara_star::AbstractNode * const goalNode,
		OpenList& openList, const ara_star::ClosedList& closedList)
	{
		ARAStarPlanning::expandNode(currentNode,goalNode,openList,closedList);

		for(size_t r=0;r<mapHistory.height;r++) {
			for(size_t c=0;c<mapHistory.width;c++) {
				if (openList.contains(GridNode::get(static_cast<int>(c), static_cast<int>(r)))) {
					mapLog.at(r).at(c) = 2;
				}
			}
		}

	}
	void ARAStarPlanningLog::savePathLengthHistory() {
		const std::string filename = std::string(PROJECT_SOURCE_DIR) + "/data/pathLengthHistory.txt";
#if (defined(_MSC_VER) && (_MSC_VER >= 1400))
		FILE *file;
		if (fopen_s(&file, filename.c_str(), "w") != 0) {
			file = NULL;
		}

#else
		FILE *file = fopen(filename.c_str(), "w");
#endif
		if (file) {
			for (size_t i=0;i<std::min(heuristic.wHistory.size(), pathLengthList.size());++i) {
				if (pathLengthList[i].pathLength > 0) {
					fprintf(file, "%.1f %zu %.2f\n",
							heuristic.wHistory[i],
							pathLengthList[i].pathLength,
							static_cast<double>(pathLengthList[i].end - pathLengthList[i].start) / CLOCKS_PER_SEC);
				}
			}
			fclose(file);
		}
	}


}  // namespace ara_star
