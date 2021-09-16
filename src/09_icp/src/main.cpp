#include <windows-helpers.h>
#include <iostream>
#include <icp/ICP.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <icp/FileIO.h>

using namespace icp;

int main(int /* argc */, char ** /*argv*/)
{
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath + "/data/data.txt");

	double threshold = 0.5;
	std::cout<<"Apply one iteration of ICP using closet point corresponding points: \n";
	StdVectorOfVector2d P_1 = fileIO.P;
	bool convergenceFlag = false;
	for(int i=0;i<10000;i++)
	{
		P_1 = ICP::iterateOnce(fileIO.Q,P_1,convergenceFlag,0,threshold);
		if (convergenceFlag==1)
		{
			std::cout<<" \n **********###########################********* \n";
			break;
		}
	}
	std::cout << "List of points obtained: "<< std::endl;
	for(size_t i=0;i<P_1.size();i++)
		std::cout << P_1[i].transpose() << std::endl;

	fileIO.writeMap(P_1, packagePath + "/data/closestPointResult.txt");

	std::cout<<"***************************************** \n";

	std::cout<<"Apply one iteration of ICP using point-to-line corresponding points: \n";
	StdVectorOfVector2d P_2 = fileIO.P;
	convergenceFlag = 0;
	for(size_t i=0;i<10000;i++)
	{
		P_2 = ICP::iterateOnce(fileIO.Q,P_2,convergenceFlag,1,threshold);
		if (convergenceFlag==1)
		{
			std::cout<<" \n **********###########################********* \n";
			break;
		}
	}
	std::cout << "List of points obtained: "<< std::endl;
	for(size_t i=0;i<P_2.size();i++)
		std::cout << P_2[i].transpose() << std::endl;

	fileIO.writeMap(P_2, packagePath + "/data/pointToLineResult.txt");

    wait();
	return 0;
}
