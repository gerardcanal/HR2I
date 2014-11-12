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
	HR2ISceneViewer(std::string name);
	~HR2ISceneViewer();

	void setScene(static pcl::PointCloud<pcl::PointXYZ>::Ptr scene, static pcl::PointCloud<pcl::PointXYZ>::Ptr floor, const Skeleton& skel, pcl::PointXYZ pointpoint);
private:
	pcl::visualization::CloudViewer _viewer;
	//Show elements
	pcl::PointCloud<pcl::PointXYZ>::Ptr _scene;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _floor;
	Skeleton _person;
	pcl::PointXYZ _pointingPoint;
	std::mutex mtx;

	// Viewer loop callbacks
	void initScene(pcl::visualization::PCLVisualizer& viewer);
	void updateScene(pcl::visualization::PCLVisualizer& viewer);

	//Draw on things
	void drawSkeleton(pcl::visualization::PCLVisualizer& viewer, Skeleton& skel);
};

