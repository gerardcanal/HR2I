// STD includes
#define _CRT_SECURE_NO_WARNINGS // Also in project configuration as putting the define here is not enough
#include "stdafx.h"
#include <string>
#include <stdio.h>
#include <iostream>

// ROS
#include "ros.h"

// Project includes
#include "HR2I_Kinect2.h"
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

void checkGroundParams(HR2I_Kinect2& hr2i, Kinect2Utils& k2u, HR2ISceneViewer& pcl_viewer) {
	const string GROUND_PARAMS_PATH = "Parameters\\GroundPlaneCoeffs.txt";

	cout << "Checking ground coefficients... ";
	bool recomputeGroundCoeffs = false;
	vector<float> ground_coeffs;
	try { ground_coeffs = hr2i.readGroundPlaneCoefficients(GROUND_PARAMS_PATH); }
	catch (exception& e) { recomputeGroundCoeffs = true; }
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundplane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	if (recomputeGroundCoeffs || !hr2i.checkGroundCoefficients(&k2u, ground_coeffs, groundplane)) {
		do{
			showMessageRecomputeGC();
			cout << "Ground coefficients must be recomputed. Please select the points..." << endl;
			ground_coeffs = hr2i.computeGroundCoefficientsFromUser(&k2u);
		} while (!hr2i.checkGroundCoefficients(&k2u, ground_coeffs, groundplane));
		hr2i.writeGroundPlaneCoefficients(ground_coeffs, GROUND_PARAMS_PATH);
		cout << "DONE: ground coefficients were stored in \"" << GROUND_PARAMS_PATH << "\"" << endl;
	}
	else cout << "DONE" << endl;
	//hr2i.setGroundCoefficients(ground_coeffs); // Redundant...
	// Get plane point
	pcl::PointXYZ p = groundplane->at(groundplane->size() / 2); // Get random point
	vector<float> gpoint = { p.x, p.y, p.z };
	hr2i.setGroundInfo(ground_coeffs, gpoint);
	pcl_viewer.setGroundCoeffs(ground_coeffs);
}

// MAIN
int _tmain(int argc, _TCHAR * argv[]) {
	const string GR_PARAMS_PATH = "Parameters\\GestureRecognitionParameters.txt"; 
	const string GESTURE_MODELS_PATH = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\";
	const int RGB_Depth = 1; // 0 - None, 1 - RGB, 2 - Depth

	// Initialization
	cout << "Initializing Kinect 2 interface... ";
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	// Multiframe not used because RGB interferes with it...
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
	checkGroundParams(hr2i, k2u, pcl_viewer);

	// Main code
	for (int i = 0; i < 10; ++i) {
		hr2i_thesis::GestureRecognitionResult gr_res = hr2i.recognizeGestures(GR_PARAMS_PATH, hr2i.readDynamicModels(GESTURE_MODELS_PATH), k2u);
		if (gr_res.gestureId == gr_res.idPointAt) {
			pcl::PointXYZ ppoint(gr_res.ground_point.x, gr_res.ground_point.y, gr_res.ground_point.z);
			//pcl_viewer.setPointingPoint(ppoint);
			hr2i.getAndDrawScene(&k2u, ppoint, true, true, OBJECT_RADIUS);
			Sleep(15000);
			pcl_viewer.setPointingPoint(pcl::PointXYZ(0,0,0));
		}
	}
	cout << "Hello World!" << endl;
	int x; cin >> x;
	iface.join();
}


//FAKEMAIN 
/*
int _tmain(int argc, _TCHAR * argv[]) {
	cout << "Initializing Kinect 2 interface... ";
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	hr = k2u.openBodyFrameReader();
	hr = k2u.openDepthFrameReader();
	if (!SUCCEEDED(hr)) return -1;
	cout << "DONE" << endl;

	ICoordinateMapper* cmapper = NULL;
	k2u.getCoordinateMapper(cmapper);

	std::array<int, 2> size, pos;
	HR2ISceneViewer pcl_viewer("Human MultiRobot Interaction 3D Viewer", true);
	while (true) {
		IDepthFrame* df = k2u.getLastDepthFrameFromDefault();
		if (df != NULL) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = K2PCL::segmentObjectsFromScene(pcPtr);
			pcl_viewer.setScene(pcPtr);
			pcl_viewer.setSegmentedClusters(clusters);
			cout << endl << "-------------------------------------------------" << endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr jointclusters(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (int i = 0; i < clusters.size(); ++i) 
			{
				pcl::PointXYZ cent = K2PCL::compute3DCentroid(clusters[i]);
				vector<float> stdcent = K2PCL::pclPointToVector(cent);
				vector<float> origin = { -0.02f, 0.0f, 1.3f };
				if (Utils::euclideanDistance(stdcent, origin) < 0.4) {

					pair<double, double> areavol = K2PCL::computeAreaVolume(clusters[i]);
					cout << "Cluster " << i << ": Size: " << clusters[i]->size() << " Area: " << areavol.first << " Volume: " << areavol.second << " Centroid: ";
					cout << "Centroid: " << cent.x << " " << cent.y << " " << cent.z << " Distance: " << Utils::euclideanDistance(stdcent, origin) << endl << endl;
					pcl_viewer.setPointingPoint(cent);
				}

				/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRGBptr(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcRGBptr->resize(clusters[i]->size());
				uint8_t r = rand() % 255, g = rand() % 255, b = rand() % 255;
				for (int j = 0; j < clusters[i]->size(); ++j) {
					pcl::PointXYZRGB auxp(r, g, b);
					auxp.x = clusters[i]->at(j).x;
					auxp.y = clusters[i]->at(j).y;
					auxp.z = clusters[i]->at(j).z;
					pcRGBptr->at(j) = auxp;
				}
				*jointclusters += *pcRGBptr;*/ /*
			}
			//pclviewer.showCloud(jointclusters);
		}
		SafeRelease(df);
	}
}*/