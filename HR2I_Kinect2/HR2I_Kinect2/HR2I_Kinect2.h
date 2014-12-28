#include "stdafx.h"
// Std
#include <thread>
#include <mutex>
#include <vector>

// PCL
#include <pcl/sample_consensus/sac_model_plane.h>

// Project inclusions
#include "Kinect2Utils.h"
#include "GestureRecognition.h"
#include "BodyRGBViewer.h"
#include "HR2ISceneViewer.h"
#include "K2PCL.h"

// ROS
#undef ERROR
#include "ros.h"
#include "hr2i_thesis/GestureRecognitionResult.h"
#include "hr2i_thesis/Kinect2Command.h"
#include "hr2i_thesis/PointCloudClusterCentroids.h"

#define OBJECT_RADIUS 0.5 // Distance between the point and the objects which are to be segmented
#define SHOWING_GESTURE_TIME 15000
#define SHOWING_CLUSTERS_TIME 20000

using namespace std;

class HR2I_Kinect2 {
public:
	HR2I_Kinect2(BodyRGBViewer* view, HR2ISceneViewer* pcl_viewer, Kinect2Utils* k2u, ros::NodeHandle* nh);
	~HR2I_Kinect2();

	// Gesture methods
	hr2i_thesis::GestureRecognitionResult recognizeGestures(const string& GRParams_path, const vector<vector<vector<float>>>& models);
	vector<vector<vector<float>>> readDynamicModels(string gestPath = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\");
	
	// Ground methods
	std::vector<float> computeGroundCoefficientsFromUser();
	bool HR2I_Kinect2::checkGroundCoefficients(std::vector<float> vec_g_coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_out);
	std::vector<float> HR2I_Kinect2::readGroundPlaneCoefficients(string path = "Parameters\\GroundPlaneCoeffs.txt");
	void HR2I_Kinect2::writeGroundPlaneCoefficients(std::vector<float> vec_g_coeffs, string path = "Parameters\\GroundPlaneCoeffs.txt");
	void setGroundInfo(const vector<float>& ground_coeffs, const vector<float>& groundPoint);
	deque<Skeleton>* getInputFrames();
	void getAndDrawScene(pcl::PointXYZ pointingPoint, bool drawBody, bool drawObjects, int obj_radius = OBJECT_RADIUS, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* objects = NULL);

	// Callbacks
	void k2CommandReceivedCb(const hr2i_thesis::Kinect2Command& cmd);

	// States
	void recognizeGestureState(const string& gr_params_path, const string& gr_models_path, ros::Publisher* gesture_pub);
	hr2i_thesis::Kinect2Command waitForCommandState();
	void clusterObjectsState(ros::Publisher* clusters_pub);
private:
	// Vars
	mutex gr_mtx;
	bool get_data;
	deque<Skeleton> inputFrames;
	vector<float> ground_coeffs;
	vector<float> groundplane_point;
	Kinect2Utils* k2u;

	// ROS
	ros::NodeHandle* nh;
	recursive_mutex roscmd_mtx; // Recursive because nh->spinOnce() calls a lock which is made by the same thread, resulting in an error...
	hr2i_thesis::Kinect2Command k2cmd;
	pcl::PointXYZ pointingPoint;

	// Viewers
	BodyRGBViewer* body_view;
	HR2ISceneViewer* pcl_viewer;


	// Methods
	void dataGetter(GestureRecognition* gr, GRParameters params);
	//void setPCLScene(IDepthFrame* df, ICoordinateMapper* cmapper, std::vector<float> vec_g_coeffs);
	pcl::PointCloud<pcl::PointXYZ>::Ptr HR2I_Kinect2::getOnePointCloudFromKinect();
};