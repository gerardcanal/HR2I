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
#include <vector>
#include <utility>

#include "Kinect2Utils.h"

#define MAX_ITER_DEF 150
#define MAX_PLANES_ITER 5 // Number of planes to be segmented... Those will be the biggest ones
#define EQUAL_PLANE_TH 0.1

class K2PCL
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cmapper);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize);
	static std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int max_iter = MAX_ITER_DEF, bool verbose = false);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlaneByDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<float>& direction,
		                                                               const float equal_plane_th = EQUAL_PLANE_TH, const int max_planes = MAX_PLANES_ITER, const int max_iter = MAX_ITER_DEF);
}; 

