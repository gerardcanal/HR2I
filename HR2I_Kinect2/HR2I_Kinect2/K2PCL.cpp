// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#include "K2PCL.h"
#include "stdafx.h"

//#define MIRROR_PCL // To mirror de PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::depthFrameToPointCloud(IDepthFrame*& depthFrame, ICoordinateMapper* cMapper, bool release_depthframe) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
	UINT bsize = 0;
	UINT16* depthBuffer = NULL;
	HRESULT hr = depthFrame->AccessUnderlyingBuffer(&bsize, &depthBuffer);
	if (!SUCCEEDED(hr)) return pc;

	IFrameDescription* frameDesc = NULL;
	hr = depthFrame->get_FrameDescription(&frameDesc);
	if (!SUCCEEDED(hr)) return pc;
	int nDepthWidth, nDepthHeight, nPoints;
	frameDesc->get_Width(&nDepthWidth); frameDesc->get_Height(&nDepthHeight);
	nPoints = nDepthWidth * nDepthHeight;
	CameraSpacePoint* _cameraCoordinates = new CameraSpacePoint[nPoints];
	
	hr = cMapper->MapDepthFrameToCameraSpace(nPoints, depthBuffer, nPoints, _cameraCoordinates);
	
	if (release_depthframe) SafeRelease(depthFrame);
	
	if (!SUCCEEDED(hr)) return pc;
	int filtered = 0;
	for (int i = 0; i < nPoints; ++i) {
		// add cameraCoordinates[i] in the pointcloud
		if ((_cameraCoordinates[i].X < Utils::INF) && (_cameraCoordinates[i].Y < Utils::INF) && (_cameraCoordinates[i].Z < Utils::INF) &&
			(_cameraCoordinates[i].X > -Utils::INF) && (_cameraCoordinates[i].Y > -Utils::INF) && (_cameraCoordinates[i].Z > -Utils::INF)) {
			#ifdef MIRROR_PCL
				pcl::PointXYZ _point(-_cameraCoordinates[i].X, _cameraCoordinates[i].Y, _cameraCoordinates[i].Z);
			#else
				pcl::PointXYZ _point(_cameraCoordinates[i].X, _cameraCoordinates[i].Y, _cameraCoordinates[i].Z);
			#endif
			pc->push_back(_point);
		}
		else ++filtered;
	}
	/*pc->width = nDepthWidth;
	pc->height = (nPoints-filtered)/nDepthWidth;*/
	delete[] _cameraCoordinates;
	return pc;
}

/// Returns the pointindices of the segmented plane
std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> K2PCL::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double dist_th, int max_iter, bool verbose) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(max_iter);
	seg.setDistanceThreshold(dist_th);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return std::make_pair(inliers, coefficients);
	}

	if (verbose) std::cerr << "Model coefficients: " << coefficients->values[0] << " "
					<< coefficients->values[1] << " "
					<< coefficients->values[2] << " "
					<< coefficients->values[3] << std::endl;
	return std::make_pair(inliers, coefficients);
}

/// Cloud is the input cloud but after the method is the input cloud without the extracted points (which are returned by the method)
pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::extractIndices(pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>());
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Extract the inliers
	extract.setInputCloud(cloud);
	extract.setIndices(indices);
	extract.setNegative(false);
	extract.filter(*extracted);

	// Remove them from the input cloud
	extract.setNegative(true);
	extract.filter(*remaining);
	cloud.swap(remaining);
	return extracted;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(*cloud_filtered);
	return cloud_filtered;
}

/// Returns the point cloud with the plane points, and the pc is the original pointcloud without the plane
pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::segmentPlaneByDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<float>& direction, const float equal_plane_th, const int max_planes, const int max_iter) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr retPc(new pcl::PointCloud<pcl::PointXYZ>());
	bool found = false;
	std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> planeinfo;
	int i_planes = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ret_plane(new pcl::PointCloud<pcl::PointXYZ>());
	do {
		planeinfo = K2PCL::segmentPlane(pc);
		if (planeinfo.second->values.size() == 0) break;
		std::vector<float> n_plane(planeinfo.second->values.begin(), planeinfo.second->values.end() - 1); // Normal vector of the plane
		if (Utils::sameDirection(n_plane, direction, equal_plane_th)) {
			*ret_plane += *extractIndices(planeinfo.first, pc); // Add extracted clouds of the plane to the plane cloud
		}
		else {
			*retPc += *extractIndices(planeinfo.first, pc); // Add extracted clouds to retPc
		}
	} while (!found && (++i_planes < max_planes));

	*retPc += *pc; // Concatenate rest of pointcloud to get the final cloud without the image
	pc.swap(retPc); // Change retPc for pc to make it output parameter
	return ret_plane;
}

/// Returns the clusters found in the scene. The input cloud does not have the segmented clusters at the end of the function call
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> K2PCL::segmentObjectsFromScene(pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloud, int max_size, int min_size, double cluster_tolerance) {
	const float PERC_POINTS = 0.45; // original: 0.3. Working: 0.2
	// Add downsample if needed...

	//Segment planes to take them out of the clustering method
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractedPlanes(new pcl::PointCloud<pcl::PointXYZ>); // FIXME result in slow performance...
	int nr_points = (int)cloud->size();
	while (cloud->size() > PERC_POINTS*nr_points) {
		pcl::PointIndices::Ptr planeindices = K2PCL::segmentPlane(cloud, 0.02, 100).first;
		*extractedPlanes += *K2PCL::extractIndices(planeindices, cloud);
	}

	///////////////////////////////////////////////////
	// Segment objects from the resulting pointcloud //
	///////////////////////////////////////////////////

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	//Create the cluster extraction object and extract clusters from the resulting pointcloud
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_tolerance); // 2cm
	ec.setMinClusterSize(min_size);
	ec.setMaxClusterSize(max_size); //25000
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	pcl::PointIndices::Ptr indicesToExtract(new pcl::PointIndices);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(cluster_indices.size());
	for (int i = 0; i < cluster_indices.size(); ++i) {
		clusters[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); pit++)
			clusters[i]->points.push_back(cloud->points[*pit]);
		clusters[i]->width = clusters[i]->points.size();
		clusters[i]->height = 1;
		clusters[i]->is_dense = true;
		indicesToExtract->indices.insert(indicesToExtract->indices.end(), cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
	}
	K2PCL::extractIndices(indicesToExtract, cloud); // Remove the clusters from the input pointcloud
	*cloud += *extractedPlanes; // Add planes to restore scene
	return clusters;
}

/// Distance is the euclidean distance between nearPoint and the centroid of the object clusters. Those which are in a distance < distance
/// Returns the clusters found in the scene. The input cloud does not have the segmented clusters at the end of the function call
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> K2PCL::segmentObjectsNearPointFromScene(pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloud, const double distance_th, pcl::PointXYZ nearPoint, double cluster_tolerance, int max_size, int min_size) {
	/////////////////////////////
	/// Pick the region of the cloud based in the point with radius distance_th.
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr _tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	_tree->setInputCloud(cloud);
	pcl::PointIndices::Ptr subCloudIndices(new pcl::PointIndices);
	std::vector<float> k_sqr_distances; // Not actually used...
	_tree->radiusSearch(nearPoint, distance_th, subCloudIndices->indices, k_sqr_distances);

	// Extract the indices from the input cloud...
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointedRegion = K2PCL::extractIndices(subCloudIndices, cloud);
	///////////////////////

	// Segment the objects from the region
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = segmentObjectsFromScene(pointedRegion, max_size, min_size, cluster_tolerance);
	*cloud += *pointedRegion; // Return the points of the pointed region without clusters to the scene pointcloud
	//cloud.swap(pointedRegion); // To keep only the subcloud

	/* Inefficient version
	int i = 0;
	while (i < clusters.size()) {
		pcl::PointXYZ cluster_centroid = K2PCL::compute3DCentroid(clusters[i]);
		// if distance between the centroid and nearPoint is greater than the threshold, we remove the cluster and add it again to the scene cloud
		if (/*Utils::euclideanDistance(K2PCL::pclPointToVector(cluster_centroid), K2PCL::pclPointToVector(nearPoint)) > distance_th || * /
			K2PCL::computeAreaVolume(val).second < VOLUME_THRESHOLD) {
			*cloud += *clusters[i]; // Add the cluster to the scene again
			clusters.erase(clusters.begin() + i);
			std::cout << "erased " << i << " size: " << clusters.size() << std::endl;
		}
		else ++i;
	}*/

	clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
		[&](const pcl::PointCloud<pcl::PointXYZ>::Ptr val) -> bool {
			//std::cout << K2PCL::computeAreaVolume(val).second << std::endl;
			pcl::PointXYZ cluster_centroid = K2PCL::compute3DCentroid(val);
			if (/*Utils::euclideanDistance(K2PCL::pclPointToVector(cluster_centroid), K2PCL::pclPointToVector(nearPoint)) > distance_th ||*/
				K2PCL::computeAreaVolume(val).second < VOLUME_THRESHOLD) {
				*cloud += *val; // Add the cluster to the scene again
				return true;
			}
				return false;
		}),
		clusters.end());
	return clusters;
}

pcl::PointXYZ K2PCL::compute3DCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	Eigen::Vector4d centr;
	unsigned int res = pcl::compute3DCentroid(*cloud, centr);
	if (res == 0) return pcl::PointXYZ(0, 0, 0); // Error code...
	return pcl::PointXYZ(centr.x(), centr.y(), centr.z());
}


std::pair<double, double> K2PCL::computeAreaVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> ch;
	ch.setInputCloud(cloud);
	ch.setComputeAreaVolume(true);
	ch.reconstruct(*cloud_hull);
	return std::make_pair(ch.getTotalArea(), ch.getTotalVolume());
}

std::vector<float> K2PCL::pclPointToVector(pcl::PointXYZ& p) {
	std::vector<float> ret = { p.x, p.y, p.z };
	return ret;
}