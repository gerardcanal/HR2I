#include "K2PCL.h"

//#define MIRROR_PCL // To mirror de PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cMapper) {
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
std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> K2PCL::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_iter, bool verbose) {
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
	seg.setDistanceThreshold(0.01);

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
pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::segmentPlaneByDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<float> direction, const float equal_plane_th, const int max_planes, const int max_iter) {
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