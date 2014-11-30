// STD includes
#include "stdafx.h"
#include <string>
#include <stdio.h>
#include <iostream>

// ROS
#include "ros.h"

// Project includes
#include "HR2I_Kinect.h"
#include "HR2ISceneViewer.h"

using namespace std;

int _tmain(int argc, _TCHAR * argv[]) {
	const string GR_PARAMS_PATH = "Parameters\\GestureRecognitionParameters.txt"; 
	const string GESTURE_MODELS_PATH = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\";
	const int RGB_Depth = 0; // 0 - None, 1 - RGB, 2 - Depth

	// Initialization
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	hr = k2u.openBodyFrameReader();
	if (!SUCCEEDED(hr)) return -1;

	BodyRGBViewer body_view(&k2u);
	//HR2ISceneViewer viewer("Human MultiRobot Interaction Viewer", true);
	thread iface = body_view.RunThreaded(RGB_Depth, true, false);

	// HR2I
	HR2I_Kinect2 hr2i;
	hr2i_thesis::GestureRecognitionResult gr_res = hr2i.recognizeGestures(GR_PARAMS_PATH, hr2i.readDynamicModels(GESTURE_MODELS_PATH), k2u, body_view);
	cout << "Hello World!" << endl;
	int x; cin >> x;
	iface.join();
}
