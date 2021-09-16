#ifndef WINDOWS_HELPERS_H_
#define WINDOWS_HELPERS_H_

#if _WIN32 || _WIN64
#include <iostream>

void wait() {
	std::cout << std::endl << "Finished. Press Enter to leave the program." << std::endl;
	getchar();
}

#else
void wait() {}
#endif

#endif // WINDOWS_HELPERS_H_
