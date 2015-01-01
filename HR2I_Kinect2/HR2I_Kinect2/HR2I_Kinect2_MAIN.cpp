// STD includes
#define _CRT_SECURE_NO_WARNINGS // Also in project configuration as putting the define here is not enough
#include "stdafx.h"
#include <string>
#include <stdio.h>
#include <iostream>

// ROS
#include "ros.h"
#include "hr2i_thesis/GestureRecognitionResult.h"
#include "hr2i_thesis/PointCloudClusterCentroids.h"
#include "hr2i_thesis/Kinect2Command.h"

#define GR_TOPIC_NAME "recognized_gesture"
#define CLUSTERS_TOPIC_NAME "kinect2_clusters"

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

void checkGroundParams(HR2I_Kinect2& hr2i, Kinect2Utils& k2u, HR2ISceneViewer& pcl_viewer, const string& GROUND_PARAMS_PATH, ros::NodeHandle& nh) {
	cout << "Checking ground coefficients... ";
	bool recomputeGroundCoeffs = false;
	vector<float> ground_coeffs;
	try { ground_coeffs = hr2i.readGroundPlaneCoefficients(GROUND_PARAMS_PATH); }
	catch (exception& e) { recomputeGroundCoeffs = true; }
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundplane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	if (recomputeGroundCoeffs || !hr2i.checkGroundCoefficients(ground_coeffs, groundplane)) {
		do{
			showMessageRecomputeGC();
			cout << "Ground coefficients must be recomputed. Please select the points..." << endl;
			ground_coeffs = hr2i.computeGroundCoefficientsFromUser();
			nh.spinOnce();
		} while (!hr2i.checkGroundCoefficients(ground_coeffs, groundplane));
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

bool USE_ROS = true;
HR2I_Kinect2* hr2iPtr; // Global because if not the subscriber fucks it.

void cmd_subs_cb(const hr2i_thesis::Kinect2Command& cmd) { 
	string cmd_s = (cmd.command == cmd.recGestCmd) ? "Recognize gestures" : "Segment objects";
	cout << "Command received \"" << cmd_s << "\" from " << cmd.header.frame_id;

	if (hr2iPtr->getCurrentState() == HR2I_Kinect2::WAITING) {
		hr2iPtr->k2CommandReceivedCb(cmd);
	}
	else cout << " - ignored as not in Waiting state";
	cout << endl;
}


// MAIN 
int _tmain(int argc, _TCHAR * argv[]) {
	const string GROUND_PARAMS_PATH = "Parameters\\GroundPlaneCoeffs.txt";
	const string GR_PARAMS_PATH = "Parameters\\GestureRecognitionParameters.txt";
	const string GESTURE_MODELS_PATH = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\";
	const int RGB_Depth = 1; // 0 - None, 1 - RGB, 2 - Depth

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////// Initialization //////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////
	// Kinect ////
	//////////////
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

	/////////////////////
	//// Viewers ////////
	/////////////////////
	cout << "Initializing visualizers... ";
	BodyRGBViewer body_view(&k2u);
	thread iface = body_view.RunThreaded(RGB_Depth, true, false);
	
	std::array<int, 2> size, pos;
	computePCLWindowSizeAndPos(size, pos, body_view);
	HR2ISceneViewer pcl_viewer("Human MultiRobot Interaction 3D Viewer", true, size, pos);
	cout << "DONE" << endl;

	/////////////////////
	////// ROS //////////
	/////////////////////
	cout << "Initializing ROS... ";
	ifstream myfile; // Get ROS MASTER URI from file
	myfile.open("Parameters/ROS_MASTER_HOST.txt");
	if (!myfile.is_open()) {
		cerr << "ERROR: ROS_MASTER_HOST.txt is not in Parameters/ROS_MASTER_HOST.txt" << endl;
		exit(-1);
	}
	string ROS_MASTER_HOST;
	getline(myfile, ROS_MASTER_HOST);
	while (ROS_MASTER_HOST[0] == '#') std::getline(myfile, ROS_MASTER_HOST);
	myfile.close();

	ros::NodeHandle nh; // ROS node handle
	if (USE_ROS) {
		char *ros_master = const_cast<char*>(ROS_MASTER_HOST.c_str());
		printf("Connecting to server at %s\n", ros_master);
		nh.initNode(ros_master);
	}
	hr2i_thesis::GestureRecognitionResult gr_msg;
	ros::Publisher gest_pub(GR_TOPIC_NAME, &gr_msg);
	nh.advertise(gest_pub);

	hr2i_thesis::PointCloudClusterCentroids pcc_msg;
	ros::Publisher cluster_pub(CLUSTERS_TOPIC_NAME, &pcc_msg);
	nh.advertise(cluster_pub);
	cout << "DONE" << endl;

	// HR2I init
	HR2I_Kinect2 hr2i(&body_view, &pcl_viewer, &k2u, &nh);
	hr2iPtr = &hr2i;
	ros::Subscriber < hr2i_thesis::Kinect2Command > k2cmdSub("kinect2_command", &cmd_subs_cb);
	nh.subscribe(k2cmdSub);

	// Check ground coefficients
	checkGroundParams(hr2i, k2u, pcl_viewer, GROUND_PARAMS_PATH, nh);
	nh.spinOnce(); // Just in case checkGround takes too much...

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////// Main code ///////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// First state
	//currentState = GESTURE_RECOGNITION;
	//hr2i.recognizeGestureState(GR_PARAMS_PATH, GESTURE_MODELS_PATH, &gest_pub);
	//nh.spinOnce();
	while (body_view.isRunning()) {
		hr2i_thesis::Kinect2Command cmd = hr2i.waitForCommandState();
		if (cmd.command == cmd.recGestCmd) {
			hr2i.recognizeGestureState(GR_PARAMS_PATH, GESTURE_MODELS_PATH, &gest_pub);
		}
		else if (cmd.command == cmd.segmentBlobs) {
			hr2i.clusterObjectsState(&cluster_pub);
		}
		else cerr << "ERROR: Unknown command code \"" << cmd.command << "\" received. Something strange happened.";
		nh.spinOnce();
	}
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
			//vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = K2PCL::segmentObjectsFromScene(pcPtr);
			vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = K2PCL::segmentObjectsNearPointFromScene(pcPtr, OBJECT_RADIUS, pcl::PointXYZ(-0.1103f, -0.5095f, 1.4396f));

			pcl_viewer.setScene(pcPtr);
			pcl_viewer.setSegmentedClusters(clusters);
			cout << endl << "-------------------------------------------------" << endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr jointclusters(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (int i = 0; i < clusters.size(); ++i) 
			{
				pcl::PointXYZ cent = K2PCL::compute3DCentroid(clusters[i]);
				vector<float> stdcent = K2PCL::pclPointToVector(cent);
				vector<float> origin = { -0.1103f, -0.5095f, 1.4396f };
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