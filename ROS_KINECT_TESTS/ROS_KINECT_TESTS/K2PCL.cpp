#include "K2PCL.h"


/*K2PCL::K2PCL()
{
}


K2PCL::~K2PCL()
{
}
*/


pcl::PointCloud<pcl::PointXYZ>::Ptr K2PCL::depthFrameToPointCloud(IDepthFrame* depthFrame, ICoordinateMapper* cMapper) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
	UINT bsize = 0;
	UINT16* depthBuffer = NULL;
	HRESULT hr = depthFrame->AccessUnderlyingBuffer(&bsize, &depthBuffer);
	if (!SUCCEEDED(hr)) return pc;

	IFrameDescription* frameDesc = NULL;
	hr = depthFrame->get_FrameDescription(&frameDesc);
	if (!SUCCEEDED(hr)) return pc;
	int nDepthWidth, nDepthHeight;
	frameDesc->get_Width(&nDepthWidth); frameDesc->get_Height(&nDepthHeight);
	CameraSpacePoint* _cameraCoordinates = new CameraSpacePoint[nDepthWidth * nDepthHeight];
	hr = cMapper->MapDepthFrameToCameraSpace(nDepthWidth * nDepthHeight, depthBuffer, nDepthWidth * nDepthHeight, _cameraCoordinates);
	if (!SUCCEEDED(hr)) return pc;
	for (int i = 0; i < (nDepthWidth * nDepthHeight); ++i) {
		// add cameraCoordinates[i] in the pointcloud
		if ((_cameraCoordinates[i].X < Utils::INF) && (_cameraCoordinates[i].Y < Utils::INF) && (_cameraCoordinates[i].Z < Utils::INF) &&
			(_cameraCoordinates[i].X > -Utils::INF) && (_cameraCoordinates[i].Y > -Utils::INF) && (_cameraCoordinates[i].Z > -Utils::INF)) {
			pcl::PointXYZ _point(_cameraCoordinates[i].X, _cameraCoordinates[i].Y, _cameraCoordinates[i].Z);
			pc->push_back(_point);
		}
	}
	pc->width = nDepthWidth;
	pc->height = nDepthHeight;
	delete[] _cameraCoordinates;
	return pc;
}

/*void K2PCL::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud, pcl::visualization::CloudViewer& viewer) {
	viewer.showCloud(m_ptrCloud, "cloud"); 
	int x;
	std::cin >> x;
}*/

/*pcl::visualization::CloudViewer K2PCL::getPCLViewer() {
	return pcl::visualization::CloudViewer("Simple Cloud Viewer");
}*/