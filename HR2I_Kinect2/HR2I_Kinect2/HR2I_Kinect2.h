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
#include "hr2i_thesis/GestureRecognitionResult.h"

using namespace std;

class HR2I_Kinect2 {
public:
	HR2I_Kinect2(BodyRGBViewer* view, HR2ISceneViewer* pcl_viewer);
	~HR2I_Kinect2();

	// Gesture methods
	hr2i_thesis::GestureRecognitionResult recognizeGestures(const string& GRParams_path, const vector<vector<vector<float>>>& models, Kinect2Utils& k2u);
	vector<vector<vector<float>>> readDynamicModels(string gestPath = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\");
	
	// Ground methods
	std::vector<float> computeGroundCoefficientsFromUser(Kinect2Utils* k2u);
	bool HR2I_Kinect2::checkGroundCoefficients(Kinect2Utils* k2u, std::vector<float> vec_g_coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_out);
	std::vector<float> HR2I_Kinect2::readGroundPlaneCoefficients(string path = "Parameters\\GroundPlaneCoeffs.txt");
	void HR2I_Kinect2::writeGroundPlaneCoefficients(std::vector<float> vec_g_coeffs, string path = "Parameters\\GroundPlaneCoeffs.txt");
	void setGroundInfo(const vector<float>& ground_coeffs, const vector<float>& groundPoint);
	deque<Skeleton>* getInputFrames();

private:
	// Vars
	mutex gr_mtx;
	bool get_data;
	deque<Skeleton> inputFrames;
	vector<float> ground_coeffs;
	vector<float> groundplane_point;

	// Viewers
	BodyRGBViewer* body_view;
	HR2ISceneViewer* pcl_viewer;


	// Methods
	void dataGetter(Kinect2Utils* k2u, GestureRecognition* gr, GRParameters params);
	void setPCLScene(IDepthFrame* df, ICoordinateMapper* cmapper, std::vector<float> vec_g_coeffs);
	pcl::PointCloud<pcl::PointXYZ>::Ptr HR2I_Kinect2::getOnePointCloudFromKinect(Kinect2Utils* k2u);
};