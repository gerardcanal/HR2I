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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>

#include <mutex>
#include <vector>
#include <utility>

#include "Kinect2Utils.h"

#define MAX_ITER_DEF    150
#define MAX_PLANES_ITER 5 // Number of planes to be segmented... Those will be the biggest ones
#define EQUAL_PLANE_TH  0.1
#define PLANE_DIST_TH   0.01

#define MIN_CLUSTER_SIZE 100
#define MAX_CLUSTER_SIZE 3000
#define CLUSTER_TOLERANCE 0.045 // default: 0.02 (2 cm)

class K2PCL
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cmapper);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr extractIndices(pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize);
	static std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double dist_thresh = PLANE_DIST_TH, int max_iter = MAX_ITER_DEF, bool verbose = false);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlaneByDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<float>& direction,
		                                                               const float equal_plane_th = EQUAL_PLANE_TH, const int max_planes = MAX_PLANES_ITER, const int max_iter = MAX_ITER_DEF);
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentObjectsFromScene(pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloud, int max_size = MAX_CLUSTER_SIZE, int min_size = MIN_CLUSTER_SIZE, double cluster_tolerance = CLUSTER_TOLERANCE);
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentObjectsNearPointFromScene(pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloud, const int distance, pcl::PointXYZ nearPoint,
																							 int max_size = MAX_CLUSTER_SIZE, int min_size = MIN_CLUSTER_SIZE, double cluster_tolerance = CLUSTER_TOLERANCE);

	static pcl::PointXYZ compute3DCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static std::pair<double, double> computeAreaVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static std::vector<float> pclPointToVector(pcl::PointXYZ& p);
}; 

