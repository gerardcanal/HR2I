#pragma once
#undef max
#undef min
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "Kinect2Utils.h"
class K2PCL
{
public:
	/*K2PCL();
	~K2PCL();*/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cmapper);
	//static void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud, pcl::visualization::CloudViewer& viewer);
	//static pcl::visualization::CloudViewer getPCLViewer();
	
};

