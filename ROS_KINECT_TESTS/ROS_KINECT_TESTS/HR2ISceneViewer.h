#pragma once
#undef max // To avoid windows compilation problems in the pcl
#undef min // To avoid windows compilation problems in the pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include "Skeleton.h"
class HR2ISceneViewer
{
public:
	HR2ISceneViewer(std::string name, bool pickpoints);
	~HR2ISceneViewer();

	void setScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr floor);
	void setPerson(const Skeleton& skel);
	void setPointingPoint(const pcl::PointXYZ& point);
	void unregisterPointPickingCb();
	void registerPointPickingCb();
	int getNumPickedPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getPickedPointsCloud();
private:
	pcl::visualization::CloudViewer _viewer;

	// Point picking
	boost::signals2::connection pointpicker;
	static pcl::visualization::PCLVisualizer::Ptr pclvisualizerPtr; // For pointpicking easyness...
	static pcl::PointCloud<pcl::PointXYZ>::Ptr pickedPoints;
	static bool finishedPicking;
	static std::mutex ppmtx;
	static void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);

	//Show elements
	static pcl::PointCloud<pcl::PointXYZ>::Ptr _scene;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr _floor;
	static Skeleton _person;
	static pcl::PointXYZ _pointingPoint;
	static std::mutex mtx;
	static bool created;
	static bool scene_updated;
	static bool body_updated;

	// Viewer loop callbacks
	static void initScene(pcl::visualization::PCLVisualizer& viewer);
	static void updateScene(pcl::visualization::PCLVisualizer& viewer);

	//Draw on things
	static void drawSkeleton(pcl::visualization::PCLVisualizer& viewer, Skeleton& skel);
	static void drawBone(pcl::visualization::PCLVisualizer& viewer, Joint A, Joint B, const std::string& id);
	static void removeSkeleton(pcl::visualization::PCLVisualizer& viewer);
	
};

