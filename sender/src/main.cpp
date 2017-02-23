#define WIN32_LEAN_AND_MEAN     // Exclude rarely-used stuff from Windows headers
#include <windows.h>
#include <iostream>
#include "CKinect.h"

int main(int agrc, char **agrv) {
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);
	//std::cout << sizeof(ControlParam)*25 << std::endl;
	if (SUCCEEDED(CoInitialize(NULL))) {
		CKinect kinect;
		if (SUCCEEDED(kinect.initKinect())) {
			kinect.runMessageLoop();
		}
		CoUninitialize();
	}
	return 0;
}