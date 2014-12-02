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

/// Aux methods
void getDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	const HWND hDesktop = GetDesktopWindow();
	GetWindowRect(hDesktop, &desktop);
	horizontal = desktop.right;
	vertical = desktop.bottom;
}

void setConsolePosition(int posX, int posY) {
	HWND console = GetConsoleWindow();
	RECT r;
	GetWindowRect(console, &r); //stores the console's current dimensions

	SetWindowPos(console, HWND_NOTOPMOST, posX, posY, r.left - r.right, r.bottom - r.top, SWP_NOSIZE | SWP_NOZORDER);
}

void computePCLWindowSizeAndPos(std::array<int, 2>& sizeW, std::array<int, 2>& posW, BodyRGBViewer& body_view) {
	if (!body_view.isRunning()) throw std::exception("Cannot compute size if BodyViewer is not running!");
	std::array<int, 2> desktopR; getDesktopResolution(desktopR[0], desktopR[1]);
	std::array<int, 2> viewR, viewPortR; 
	bool wpos = body_view.getWindowSize(viewR[0], viewR[1]);
	if (!wpos) viewR = { { 976, 647 } };
	wpos = body_view.getViewPortSize(viewPortR[0], viewPortR[1]);
	if (!wpos) viewPortR = { { 968, 616 } };
	//int horOffset = 

	sizeW[0] = desktopR[0] - viewR[0]- (viewR[0]-viewPortR[0]);
	sizeW[1] = viewPortR[1];
	
	posW[0] = viewR[0];
	posW[1] = 0; // Top

	setConsolePosition(0, viewR[1]);
}

void showMessageRecomputeGC() {
	MessageBox(NULL, L"Ground plane coefficients need to be updated.\nPlease set them in the PCL viewer window to continue.",
		       L"User intervention needed", MB_OK | MB_ICONASTERISK);
}

// MAIN
int _tmain(int argc, _TCHAR * argv[]) {
	const string GR_PARAMS_PATH = "Parameters\\GestureRecognitionParameters.txt"; 
	const string GROUND_PARAMS_PATH = "Parameters\\GroundPlaneCoeffs.txt";
	const string GESTURE_MODELS_PATH = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\";
	const int RGB_Depth = 1; // 0 - None, 1 - RGB, 2 - Depth

	// Initialization
	cout << "Initializing Kinect 2 interface... ";
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	// Multiframe not used because RGB fucks it...
	//hr = k2u.openMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Body);
	hr = k2u.openBodyFrameReader();
	hr = k2u.openDepthFrameReader();
	if (!SUCCEEDED(hr)) return -1;
	cout << "DONE" << endl;	

	// Viewers initialization
	cout << "Initializing visualizers... ";
	BodyRGBViewer body_view(&k2u);
	thread iface = body_view.RunThreaded(RGB_Depth, true, false);
	
	std::array<int, 2> size, pos;
	computePCLWindowSizeAndPos(size, pos, body_view);
	HR2ISceneViewer pcl_viewer("Human MultiRobot Interaction 3D Viewer", true, size, pos);
	cout << "DONE" << endl;

	// HR2I init
	HR2I_Kinect2 hr2i(&body_view, &pcl_viewer);

	// Check ground coefficients
	cout << "Checking ground coefficients... ";
	bool recomputeGroundCoeffs = false;
	vector<float> ground_coeffs;
	try { ground_coeffs = hr2i.readGroundPlaneCoefficients(GROUND_PARAMS_PATH); }
	catch (exception& e) { recomputeGroundCoeffs = true; }
	if (recomputeGroundCoeffs || !hr2i.checkGroundCoefficients(&k2u, ground_coeffs)) {
		showMessageRecomputeGC();
		cout << "Ground coefficients must be recomputed. Please select the points..." << endl;
		ground_coeffs = hr2i.computeGroundCoefficientsFromUser(&k2u);
		hr2i.writeGroundPlaneCoefficients(ground_coeffs, GROUND_PARAMS_PATH);
		cout << "DONE: ground coefficients were stored in \"" << GROUND_PARAMS_PATH << "\"" << endl;
	}
	else cout << "DONE";
	//hr2i.setGroundCoefficients(ground_coeffs); // Redundant...

	// Main code
	hr2i_thesis::GestureRecognitionResult gr_res = hr2i.recognizeGestures(GR_PARAMS_PATH, hr2i.readDynamicModels(GESTURE_MODELS_PATH), k2u);
	cout << "Hello World!" << endl;
	int x; cin >> x;
	iface.join();
}
