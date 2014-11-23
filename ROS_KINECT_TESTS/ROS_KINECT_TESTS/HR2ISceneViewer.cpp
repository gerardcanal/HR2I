#include "HR2ISceneViewer.h"
#define DRAW_JOINTS 0
#define SPHERE_RADIUS 0.05

// Initialization of static members
bool HR2ISceneViewer::created = false;
pcl::PointXYZ HR2ISceneViewer::_pointingPoint = pcl::PointXYZ();
pcl::PointCloud<pcl::PointXYZ>::Ptr  HR2ISceneViewer::_scene;
pcl::PointCloud<pcl::PointXYZ>::Ptr  HR2ISceneViewer::_floor;
Skeleton  HR2ISceneViewer::_person = Skeleton();
std::mutex HR2ISceneViewer::mtx;
bool HR2ISceneViewer::scene_updated = true;
bool HR2ISceneViewer::body_updated = true;

// Methods
HR2ISceneViewer::HR2ISceneViewer(std::string name) : _viewer(name)
{
	if (created) {
		std::cerr << "A Scene viewer interface was already created!" << std::endl;
		throw std::exception("A Scene viewer interface was already created!");
	}
	else {
		created = true;
		_viewer.runOnVisualizationThreadOnce(&HR2ISceneViewer::initScene);
		_viewer.runOnVisualizationThread(&HR2ISceneViewer::updateScene);
		_pointingPoint = pcl::PointXYZ();
	}
}


HR2ISceneViewer::~HR2ISceneViewer()
{
}

void HR2ISceneViewer::setScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr floor) {
	mtx.lock();
	this->_floor = floor;
	this->_scene = scene;
	mtx.unlock();
	scene_updated = true;
}

void HR2ISceneViewer::setPerson(const Skeleton& skel) {
	mtx.lock();
	this->_person = skel;
	mtx.unlock();
	body_updated = true;
}
void HR2ISceneViewer::setPointingPoint(const pcl::PointXYZ& point) {
	mtx.lock();
	this->_pointingPoint = point;
	mtx.unlock();
}

void HR2ISceneViewer::initScene(pcl::visualization::PCLVisualizer& viewer) {
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	_floor = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	viewer.addPointCloud(_scene, "scene");
	viewer.addPointCloud(_floor, "floor");
	drawSkeleton(viewer, Skeleton()); removeSkeleton(viewer);
	viewer.removeAllCoordinateSystems();
	viewer.setCameraPosition(0, 0.2, -1, 0, 0.2, 0, 0, 1, 0);
	viewer.addSphere(pcl::PointXYZ(0, 0, -4), SPHERE_RADIUS, 0, 0, 255, "pointingPoint");
}


void HR2ISceneViewer::updateScene(pcl::visualization::PCLVisualizer& viewer) {
	if (scene_updated) {
		mtx.lock();
		// Scene
		viewer.updatePointCloud(_scene, "scene");
		viewer.updatePointCloud(_floor, "floor");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "floor");
		mtx.unlock();
		scene_updated = false;
	}

	if (body_updated) {
		// Person
		mtx.lock();
		if (_person.getTrackingID() != 0) {
			drawSkeleton(viewer, _person);
			_person = Skeleton();
		}
		else removeSkeleton(viewer);
		mtx.unlock();
		body_updated = false;
	}

	// Point
	mtx.lock();
	if (_pointingPoint.x != 0 || _pointingPoint.y != 0 || _pointingPoint.z != 0) {
		viewer.updateSphere(_pointingPoint, SPHERE_RADIUS, 0, 0, 255, "pointingPoint");
		if (body_updated) viewer.removeShape("PointArrow");
		if (_person.getTrackingID() != 0 && body_updated) {
			Joint finger = _person.getJoint(JointType_HandTipRight);
			pcl::PointXYZ pA(finger.Position.X, finger.Position.Y, finger.Position.Z);
			viewer.addArrow(_pointingPoint, pA, 1.0, 1.0, 0.0, false, "PointArrow");
		}
	}
	//else viewer.removePointCloud("pointingPoint");
	mtx.unlock();
}

void HR2ISceneViewer::drawSkeleton(pcl::visualization::PCLVisualizer& viewer, Skeleton& skel) {
	// Torso
	drawBone(viewer, skel.getJoint(JointType_Head), skel.getJoint(JointType_Neck), "HN");
	drawBone(viewer, skel.getJoint(JointType_Neck), skel.getJoint(JointType_SpineShoulder), "NSS");
	drawBone(viewer, skel.getJoint(JointType_SpineShoulder), skel.getJoint(JointType_SpineMid), "SSSM");
	drawBone(viewer, skel.getJoint(JointType_SpineMid), skel.getJoint(JointType_SpineBase), "SMSB");
	drawBone(viewer, skel.getJoint(JointType_SpineShoulder), skel.getJoint(JointType_ShoulderRight), "SSSR");
	drawBone(viewer, skel.getJoint(JointType_SpineShoulder), skel.getJoint(JointType_ShoulderLeft), "SSSL");
	drawBone(viewer, skel.getJoint(JointType_SpineBase), skel.getJoint(JointType_HipRight), "SBHR");
	drawBone(viewer, skel.getJoint(JointType_SpineBase), skel.getJoint(JointType_HipLeft), "SBHL");

	// Left arm
	drawBone(viewer, skel.getJoint(JointType_ShoulderLeft), skel.getJoint(JointType_ElbowLeft), "SLEL");
	drawBone(viewer, skel.getJoint(JointType_ElbowLeft), skel.getJoint(JointType_WristLeft), "ELWL");
	drawBone(viewer, skel.getJoint(JointType_WristLeft), skel.getJoint(JointType_HandLeft), "WLHL");
	drawBone(viewer, skel.getJoint(JointType_HandLeft), skel.getJoint(JointType_HandTipLeft), "HLHTL");
	drawBone(viewer, skel.getJoint(JointType_WristLeft), skel.getJoint(JointType_ThumbLeft), "WLTL");

	// Right Arm
	drawBone(viewer, skel.getJoint(JointType_ShoulderRight), skel.getJoint(JointType_ElbowRight), "SLER");
	drawBone(viewer, skel.getJoint(JointType_ElbowRight), skel.getJoint(JointType_WristRight), "ELWR");
	drawBone(viewer, skel.getJoint(JointType_WristRight), skel.getJoint(JointType_HandRight), "WLHR");
	drawBone(viewer, skel.getJoint(JointType_HandRight), skel.getJoint(JointType_HandTipRight), "HLHTR");
	drawBone(viewer, skel.getJoint(JointType_WristRight), skel.getJoint(JointType_ThumbRight), "WLTR");

	// Left leg
	drawBone(viewer, skel.getJoint(JointType_HipLeft), skel.getJoint(JointType_KneeLeft), "HLKL");
	drawBone(viewer, skel.getJoint(JointType_KneeLeft), skel.getJoint(JointType_AnkleLeft), "KLAL");
	drawBone(viewer, skel.getJoint(JointType_AnkleLeft), skel.getJoint(JointType_FootLeft), "ALFL");

	// Right leg
	drawBone(viewer, skel.getJoint(JointType_HipRight), skel.getJoint(JointType_KneeRight), "HLKR");
	drawBone(viewer, skel.getJoint(JointType_KneeRight), skel.getJoint(JointType_AnkleRight), "KLAR");
	drawBone(viewer, skel.getJoint(JointType_AnkleRight), skel.getJoint(JointType_FootRight), "ALFR");
}

void HR2ISceneViewer::drawBone(pcl::visualization::PCLVisualizer& viewer, Joint A, Joint B, const std::string& id) {
	viewer.removeShape(id);

	pcl::PointXYZ pA(A.Position.X, A.Position.Y, A.Position.Z);
	pcl::PointXYZ pB(B.Position.X, B.Position.Y, B.Position.Z);
	if (A.TrackingState == TrackingState_NotTracked && A.TrackingState == TrackingState_NotTracked) return;
	if (A.TrackingState == TrackingState_Inferred && A.TrackingState == TrackingState_Inferred) return;
	if (A.TrackingState == TrackingState_Tracked && A.TrackingState == TrackingState_Tracked) viewer.addLine(pA, pB, 0, 255, 0, id);
	else viewer.addLine(pA, pB, 0, 245, 255, id);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, id);

#if DRAW_JOINTS
	viewer.removePointCloud(id + "-PA");
	viewer.removePointCloud(id + "-PB");
	pcl::PointCloud<pcl::PointXYZ>::Ptr jA(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr jB(new pcl::PointCloud<pcl::PointXYZ>());
	jA->push_back(pA); jB->push_back(pB);
	viewer.addPointCloud(jA, id + "-PA");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id + "-PA");
	viewer.addPointCloud(jB, id + "-PB");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id + "-PB");

	if (A.TrackingState == TrackingState_Tracked) viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 150, 0, id+"-PA");
	else viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 150, 235, id + "-PA");

	if (B.TrackingState == TrackingState_Tracked) viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 150, 0, id + "-PB");
	else viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 150, 235, id + "-PB");
#endif
}


void HR2ISceneViewer::removeSkeleton(pcl::visualization::PCLVisualizer& viewer) {
	std::vector<std::string> ids = { "HN", "NSS", "SSSM", "SMSB", "SSSR", "SSSL",
									 "SBHR", "SBHL", "SLEL", "ELWL", "WLHL", "HLHTL",
									 "WLTL", "SLER", "ELWR", "WLHR", "HLHTR", "WLTR", "HLKL", 
									 "KLAL", "ALFL", "HLKR", "KLAR", "ALFR" };

	for (int i = 0; i < ids.size(); ++i) {
		viewer.removeShape(ids[i]);
#if DRAW_JOINTS
		viewer.removePointCloud(ids[i] + "-PA");
		viewer.removePointCloud(ids[i] + "-PB");
#endif
	}

}