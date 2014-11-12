#pragma once
#undef max // To avoid windows compilation problems in the pcl
#undef min // To avoid windows compilation problems in the pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>

#include "Kinect2Utils.h"

class K2PCL
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cmapper);
	static pcl::PointIndices::Ptr segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize);

};

