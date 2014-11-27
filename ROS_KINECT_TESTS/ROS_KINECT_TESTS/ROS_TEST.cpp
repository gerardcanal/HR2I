// rosserial_hello_world.cpp : Example of sending command velocities from Windows using rosserial
// ROS includes
#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <windows.h>

#include <deque>
#include <fstream>
#include <string>

using namespace std;

// Gesture Recognition includes and methods
#include <thread>
#include <mutex>
#include "GestureRecognition.h"
#include "Kinect2Utils.h"
#include "BodyRGBViewer.h"

// Globals
const int RGB_Depth = 2; // 0 - None, 1 - RGB, 2 - Depth
bool get_data = true;
mutex mtx;
deque<Skeleton> inputFrames;

void dataGetter(Kinect2Utils* k2u, GestureRecognition* gr, BodyRGBViewer* view, GRParameters params) {
	bool rightBody = true;
	mtx.lock();
	bool _work = get_data;
	bool first = true;
	UINT64 id = 0;
	mtx.unlock();
	while (_work) {
		IBodyFrame* bodyFrame = k2u->getLastBodyFrameFromDefault();
		if (bodyFrame) {		
			view->setBodyFrameToDraw(bodyFrame);
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, id, first);
			/// temporal cheat
			if (sk.getTrackingID() > 0) {
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			} // end of cheat
			else if (!first && sk.getTrackingID() == id) {
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
			else if (first && id != sk.getTrackingID()) {
				id = sk.getTrackingID(); // Even though the skeleton is empty -i.e. id == -1- this doesn't change nything
				first = false;
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
		}
		SafeRelease(bodyFrame); // If not the bodyFrame is not get again
		if (inputFrames.size() >= params.pointAtTh[2]) inputFrames.pop_front();
		mtx.lock();
		_work = get_data;
		mtx.unlock();
	}
}

void recognizeGestures(const vector<vector<vector<float>>>& models, Kinect2Utils& k2u, BodyRGBViewer& view) {
	GestureRecognition gr;
	GRParameters params = GestureRecognition::readParameters("Results\\GestureRecognitionParameters.txt");
	get_data = true;
	thread datagetter(dataGetter, &k2u, &gr, &view, params);
	Gesture gest = gr.RecognizeGesture(models, params);
	mtx.lock();
	get_data = false;
	mtx.unlock();
	datagetter.join();
	cout << "Recognized gesture: " << ((gest == SALUTE)? "HELLO!" : "POINT_AT!") << endl;
	if (gest == POINT_AT) {
		// Take the mean joint points
		vector<float> Hand(3, 0.0);
		vector<float> Elbow(3, 0.0);
		for (int i = 0; i < inputFrames.size(); ++i) {
			CameraSpacePoint h = inputFrames[i].getJointPosition(JointType_HandRight);
			Hand[0] += h.X; Hand[1] += h.Y; Hand[2] += h.Z;
			CameraSpacePoint e = inputFrames[i].getJointPosition(JointType_ElbowRight);
			Elbow[0] += e.X; Elbow[1] += e.Y; Elbow[2] += e.Z;
		}
		Hand[0] /= inputFrames.size(); Hand[1] /= inputFrames.size(); Hand[2] /= inputFrames.size();
		Elbow[0] /= inputFrames.size(); Elbow[1] /= inputFrames.size(); Elbow[2] /= inputFrames.size();

		// Get intersection point
		vector<float> lineVector = Utils::subtract(Hand, Elbow); // Vector Elbow->Hand EH = H-E
		//cout << "\tDirection vector is: (" << lineVector[0] << ", " << lineVector[1] << ", " << lineVector[2] << ")" << endl;
		if (lineVector[1] < -0.05) { // Direction of pointing is descendent. 0.05 To remove some errors...
			vector<float> n = { 0, 1, 0 }; // Normal vector -> Vertical
			vector<float> planePoint = { 0, -0.5, 0 }; // Point which corresponds to the floor plane
			vector<float> groundPoint = Utils::linePlaneIntersection(Hand, lineVector, planePoint, n); // Hand is a point of the line
			cout << "\tPointed point is: (" << groundPoint[0] << ", " << groundPoint[1] << ", " << groundPoint[2] << ")" << endl;
		}
		else {
			cout << "\tPointing was not directed to the ground!!!" << endl;
		}
	}
}

vector<vector<vector<float>>> readModels() {
	string gestPath = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\";
	std::vector<std::vector<std::vector<float>>> models(N_DYNAMIC_GESTURES);
	models[SALUTE] = Skeleton::gestureFeaturesFromCSV(gestPath + "HelloModel/HelloModel_features.csv");
	//models[POINT_AT] = Skeleton::gestureFeaturesFromCSV(gestPath + "PointAtModel/PointAtModel_features.csv");
	//models[POINT_AT] = GestureRecognition::addThirdFeature(models[POINT_AT]);
	return models;
}
extern void testEqualDTWMethods();
extern void trainDTWParameters();
extern void showValuesGestTh();


#include "K2PCL.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "HR2ISceneViewer.h"

void showLivePCtest() {
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	k2u.openMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth | 
								   FrameSourceTypes::FrameSourceTypes_Body);

	ICoordinateMapper* cmapper = NULL;
	k2u.getCoordinateMapper(cmapper);
	
	//pcl::visualization::CloudViewer pclviewer = pcl::visualization::CloudViewer("PCL Viewer");
	HR2ISceneViewer viewer("Human MultiRobot Interaction Viewer", true);

	{
		IDepthFrame* df = NULL;
		while (df == NULL) {
			IMultiSourceFrame* msf = k2u.getLastMultiSourceFrameFromDefault();
			if (msf == NULL) continue;

			SafeRelease(df);
			df = k2u.getDepthFrame(msf);
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
		viewer.setScene(pcPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
		
		struct HR2ISceneViewer::pp_callback_args* cb_args;
		viewer.registerPointPickingCb(cb_args);
		std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
		while (viewer.getNumPickedPoints() < 5); // Wait until we have enough points
		pcl::PointCloud<pcl::PointXYZ>::Ptr pickedPoints = viewer.getPickedPointsCloud();
		viewer.unregisterPointPickingCb();
	}

	viewer.setPointingPoint(pcl::PointXYZ(2, -2, 5));
	
	while (true) {
		IMultiSourceFrame* msf = k2u.getLastMultiSourceFrameFromDefault();
		if (msf == NULL) continue;

		IDepthFrame* df = k2u.getDepthFrame(msf);
		IBodyFrame* bodyFrame = k2u.getBodyFrame(msf);
		if (df != NULL) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);

			// Paint the biggest plane red
			//pcPtr = K2PCL::downSample(pcPtr, 0.01f);
			/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRGBptr(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcRGBptr->resize(pcPtr->size());*/

			/*std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> planeinfo = K2PCL::segmentPlane(pcPtr);
			pcl::PointIndices::Ptr indices = planeinfo.first;*/
			pcl::PointIndices::Ptr indices = K2PCL::segmentPlaneByDirection(pcPtr, std::vector<float>({ /*ground*/0.87f, 0.15f, -0.5f /*floor0.004f, -1, 0.07f*/ }));
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr plane = K2PCL::extractIndices(indices, pcPtr); // pcPtr has the remaining points
			/*int i;
			for (i = 0; i < pcPtr->size(); ++i) {
				pcl::PointXYZRGB auxp(255, 255, 255);
				auxp.x = pcPtr->at(i).x;
				auxp.y = pcPtr->at(i).y;
				auxp.z = pcPtr->at(i).z;
				pcRGBptr->at(i) = auxp;
			}
			for (int j = 0; j < plane->size(); ++j, ++i) {
				pcl::PointXYZRGB auxp(255, 0, 0);
				auxp.x = plane->at(j).x;
				auxp.y = plane->at(j).y;
				auxp.z = plane->at(j).z;
				pcRGBptr->at(i) = auxp;
			}
			//pcRGBptr->width = pcPtr->width; pcRGBptr->height = pcPtr->height;
			// End paint
			pclviewer.showCloud(pcRGBptr, "pointCloud");*/
			viewer.setScene(pcPtr, plane);
		}

		if (bodyFrame != NULL) {
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, 0, true);
			viewer.setPerson(sk);
		}

		SafeRelease(bodyFrame);
		SafeRelease(df);
		SafeRelease(msf);
	}
}

// MAIN
int _tmain(int argc, _TCHAR * argv[])
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("C:/Users/Gerard/Dropbox/MAI/3dSemester/TFM/src/HR2I/PCL_Test1/PCL_Test1/pcd/model.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);
	viewer.showCloud(cloud);
	//while (!viewer.wasStopped()) {} return 512;*/
	std::cerr << "samedirection: " << Utils::sameDirection(vector<float>({ 0, 1, 0 }), vector<float>({ 0, 5, 0 }), 0.1) << endl;
	showLivePCtest();
	return 3;
	/*std::vector<float> l0 = {0,4,0};
	std::vector<float> l = {1,2,1};
	std::vector<float> p0 = {1,2,0};
	std::vector<float> n = {1, -2, 3};
	vector<float> intersect = Utils::linePlaneIntersection(l0,l,p0,n);
	cout << intersect[0] << " " << intersect[1] << " " << intersect[2] << endl;
	int x;
	cin >> x;*/
	//trainDTWParameters();
	Kinect2Utils k2u;
	HRESULT hr = k2u.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	hr = k2u.openBodyFrameReader();
	if (!SUCCEEDED(hr)) return -1; 

	BodyRGBViewer view(&k2u);
	thread iface = view.RunThreaded(RGB_Depth, true, false);
	std::vector<std::vector<std::vector<float>>> models = readModels();
	int loopCount = 0; int changeMode = 5; int mode = 1;
	while (true) { 
		recognizeGestures(models, k2u, view); 
		if (++loopCount%changeMode == 0) {
			view.changeMode(mode, true);
			mode = mode++ % 2;
		}
	}
	iface.join();
	return 0;
	// End of test


	std::ifstream myfile;
	myfile.open("ROS_MASTER_HOST.txt");
	std::string ROS_MASTER_HOST;
	std::getline(myfile, ROS_MASTER_HOST);
	while (ROS_MASTER_HOST[0] == '#') std::getline(myfile, ROS_MASTER_HOST);
	myfile.close();


	ros::NodeHandle nh;
	char *ros_master = const_cast<char*>(ROS_MASTER_HOST.c_str());

	printf("Connecting to server at %s\n", ros_master);
	nh.initNode(ros_master);

	printf("Advertising cmd_vel message\n");
	geometry_msgs::Twist twist_msg;
	ros::Publisher cmd_vel_pub("cmd_vel", &twist_msg);
	nh.advertise(cmd_vel_pub);

	printf("Go robot go!\n");
	while (1)
	{
		twist_msg.linear.x = 5.1;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = -1.8;
		cmd_vel_pub.publish(&twist_msg);

		nh.spinOnce();
		Sleep(100);
	}

	printf("All done!\n");
	return 0;
}