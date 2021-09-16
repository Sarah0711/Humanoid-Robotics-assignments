#include <windows-helpers.h>
#include <iostream>
#include <irm/IRM.h>
#include <irm/FileIO.h>
#include <ctime>

using namespace irm;

int main(int /* argc */, char ** /*argv*/)
{
	srand((unsigned int) time(NULL));

	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath);

	IRM irm;
	irm.computeRM(50000);
	fileIO.writeRM(irm);

	irm.allocateVoxelsAndComputeIRM();
	fileIO.writeIRM(irm);

    wait();
	return 0;
}
