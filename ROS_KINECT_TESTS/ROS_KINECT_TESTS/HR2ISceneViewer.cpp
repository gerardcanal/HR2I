#include "HR2ISceneViewer.h"


HR2ISceneViewer::HR2ISceneViewer(std::string name)
{
	_viewer = pcl::visualization::CloudViewer(name);
	_viewer.runOnVisualizationThreadOnce(this->initScene);
	_viewer.runOnVisualizationThread(this->updateScene);
	_pointingPoint = pcl::PointXYZ();
}


HR2ISceneViewer::~HR2ISceneViewer()
{
}

void HR2ISceneViewer::setScene(static pcl::PointCloud<pcl::PointXYZ>::Ptr scene, static pcl::PointCloud<pcl::PointXYZ>::Ptr floor, const Skeleton& skel, pcl::PointXYZ pointpoint) {
	mtx.lock();
	this->_person = skel;
	this->_floor = floor;
	this->_scene = scene;
	this->_pointingPoint = pointpoint;
	mtx.unlock();
}

void HR2ISceneViewer::initScene(pcl::visualization::PCLVisualizer& viewer) {
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
}


void HR2ISceneViewer::updateScene(pcl::visualization::PCLVisualizer& viewer) {
	mtx.lock();
	viewer.updatePointCloud(_scene, "scene");
	viewer.updatePointCloud(_floor, "floor");
	if (_person.getTrackingID != 0)
		drawSkeleton(viewer, _person);
	if (_pointingPoint.x != 0 && _pointingPoint.y != 0 && _pointingPoint.z != 0)
		int x; //drawPoint
	mtx.unlock();
}

void HR2ISceneViewer::drawSkeleton(pcl::visualization::PCLVisualizer& viewer, Skeleton& skel) {
	//TODO
}