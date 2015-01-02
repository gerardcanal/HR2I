#include "HR2ISceneViewer.h"
#define DRAW_JOINTS 0
#define SPHERE_RADIUS 0.07

// Initialization of static members
bool HR2ISceneViewer::created = false;
pcl::PointXYZ HR2ISceneViewer::_pointingPoint = pcl::PointXYZ();
pcl::PointCloud<pcl::PointXYZ>::Ptr  HR2ISceneViewer::_scene;
//pcl::PointCloud<pcl::PointXYZ>::Ptr  HR2ISceneViewer::_floor;
Skeleton  HR2ISceneViewer::_person = Skeleton();
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> HR2ISceneViewer::clusters;
std::mutex HR2ISceneViewer::scene_mtx;
std::mutex HR2ISceneViewer::person_mtx;
std::mutex HR2ISceneViewer::pointingpoint_mtx;
std::mutex HR2ISceneViewer::clusters_mtx;
bool HR2ISceneViewer::scene_updated = true;
bool HR2ISceneViewer::body_updated = true;
bool HR2ISceneViewer::clusters_updated = false;
int HR2ISceneViewer::added_clusters = 0;
/*boost::signals2::connection HR2ISceneViewer::pointpicker;
 bool HR2ISceneViewer::_pickpoints;*/
pcl::visualization::PCLVisualizer::Ptr HR2ISceneViewer::pclvisualizerPtr;
std::mutex HR2ISceneViewer::ppmtx;
bool HR2ISceneViewer::finishedPicking = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr HR2ISceneViewer::pickedPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
std::array<int, 2> HR2ISceneViewer::size;
std::array<int, 2> HR2ISceneViewer::position;
std::vector<float> HR2ISceneViewer::ground_coeffs;

// Methods
HR2ISceneViewer::HR2ISceneViewer(std::string name, bool pickpoints, std::array<int, 2> size, std::array<int, 2> position) : _viewer(name)
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
		this->size = size;
		this->position = position;
	}
}


HR2ISceneViewer::~HR2ISceneViewer()
{
}

void HR2ISceneViewer::setScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, bool downsample) {
	if (downsample) {
		scene = K2PCL::downSample(scene, 0.01f);
	}
	scene_mtx.lock();
	//this->_floor = floor;
	this->_scene = scene;
	scene_updated = true;	
	scene_mtx.unlock();

}

void HR2ISceneViewer::setPerson(const Skeleton& skel) {
	person_mtx.lock();
	this->_person = skel;
	body_updated = true;
	person_mtx.unlock();

}
void HR2ISceneViewer::setPointingPoint(const pcl::PointXYZ& point) {
	pointingpoint_mtx.lock();
	this->_pointingPoint = point;
	pointingpoint_mtx.unlock();
}

void HR2ISceneViewer::setSceneAndSegmentedClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr scene, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
	clusters_mtx.lock();
	this->clusters = clusters;
	clusters_updated = true;
	this->setScene(scene);
	clusters_mtx.unlock();
}

void HR2ISceneViewer::initScene(pcl::visualization::PCLVisualizer& viewer) {
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	//_floor = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	viewer.addPointCloud(_scene, "scene");
	viewer.addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()), "floor");
	drawSkeleton(viewer, Skeleton()); removeSkeleton(viewer);
	viewer.removeAllCoordinateSystems();
	viewer.setCameraPosition(0, 0.2, -1, 0, 0.2, 0, 0, 1, 0);
	//viewer.addSphere(pcl::PointXYZ(0, 0, -4), SPHERE_RADIUS, 0, 0, 255, "pointingPoint");
	pclvisualizerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
	viewer.setPosition(position[0], position[1]);
	viewer.setSize(size[0], size[1]);
}


void HR2ISceneViewer::updateScene(pcl::visualization::PCLVisualizer& viewer) {
	ppmtx.lock();
	if (finishedPicking) {
		viewer.removePointCloud("clicked_points");
		pickedPoints->clear();
	}
	ppmtx.unlock();

	// Point
	pointingpoint_mtx.lock();
	if (_pointingPoint.x != 0 || _pointingPoint.y != 0 || _pointingPoint.z != 0) {
		viewer.removeShape("pointingPoint");
		viewer.addSphere(_pointingPoint, SPHERE_RADIUS, 0, 0, 1, "pointingPoint");
		//viewer.updateSphere(_pointingPoint, SPHERE_RADIUS, 0, 0, 255, "pointingPoint");
		//if (body_updated) viewer.removeShape("PointArrow"); // Not really needed...
		if (_person.getTrackingID() != 0/* && body_updated*/) {
			Joint finger = _person.getJoint(JointType_HandTipRight);
			pcl::PointXYZ pA(finger.Position.X, finger.Position.Y, finger.Position.Z);
			viewer.addArrow(_pointingPoint, pA, 1.0, 1.0, 0.0, false, "PointArrow");
		}
	}
	else {
		viewer.removeShape("pointingPoint");
		viewer.removeShape("PointArrow");
	}
	pointingpoint_mtx.unlock();

	// Body
	if (body_updated) {
		// Person
		person_mtx.lock();
		Skeleton cppers = _person;
		body_updated = false;
		_person = Skeleton();
		person_mtx.unlock();
		if (cppers.getTrackingID() != 0) {
			drawSkeleton(viewer, cppers);
		}
		else {
			removeSkeleton(viewer);
			viewer.removeShape("PointArrow");
		}
	}

	if (scene_updated) {
		// Treat clusters first
		// Remove old clusters
		for (int i = 0; i < added_clusters; ++i) viewer.removePointCloud("cluster" + std::to_string(i));
		added_clusters = 0;
		if (clusters_updated) { // Add new clusters
			clusters_mtx.lock();
			clusters_updated = false;
			for (int i = 0; i < clusters.size(); ++i) {
				std::string cloudid = "cluster" + std::to_string(i);
				viewer.addPointCloud(clusters[i], cloudid);
				double r = std::max((double)rand() / RAND_MAX, 0.75); // So we don't get confused with the floor
				double g = (double)rand() / RAND_MAX;
				double b = (double)rand() / RAND_MAX;
				if ((r < 0.35 && g < 0.35 && b < 0.35) || (r > 0.75 && g > 0.75 && b > 0.75)) {
					r = std::max((double)rand() / RAND_MAX, 0.75); // So we don't get confused with the floor
					g = (double)rand() / RAND_MAX;
					b = (double)rand() / RAND_MAX;
				}
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudid);
			}
			added_clusters = clusters.size();
			clusters_mtx.unlock();
		}

		// Scene
		scene_mtx.lock();
		pcl::PointCloud<pcl::PointXYZ>::Ptr scenecp = _scene->makeShared();
		scene_updated = false;
		scene_mtx.unlock();
		// Scene
		if (ground_coeffs.size() > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr floor = K2PCL::segmentPlaneByDirection(scenecp, ground_coeffs);
			viewer.updatePointCloud(scenecp, "scene");
			viewer.updatePointCloud(floor, "floor");
		}
		else { viewer.updatePointCloud(scenecp, "scene"); }
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "floor");
	}

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


void HR2ISceneViewer::pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	ppmtx.lock();
	//struct HR2ISceneViewer::pp_callback_args* data = (struct HR2ISceneViewer::pp_callback_args *)args;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d = *reinterpret_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr*>(args);
	if (event.getPointIndex() == -1) {
		ppmtx.unlock();
		return;
	}
	pcl::PointXYZ current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	clicked_points_3d->push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	pclvisualizerPtr->removePointCloud("clicked_points");
	pclvisualizerPtr->addPointCloud(clicked_points_3d, red, "clicked_points");
	pclvisualizerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	//std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	ppmtx.unlock();
}

int HR2ISceneViewer::getNumPickedPoints() {
	int s;
	ppmtx.lock();
	s = pickedPoints->size();
	ppmtx.unlock();
	return s;
}

void HR2ISceneViewer::registerPointPickingCb() {
	finishedPicking = false;
	pickedPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pointpicker = _viewer.registerPointPickingCallback(&HR2ISceneViewer::pp_callback, (void*)&pickedPoints);
}

void HR2ISceneViewer::unregisterPointPickingCb() {
	ppmtx.lock();
	finishedPicking = true;
	pointpicker.disconnect();
	ppmtx.unlock();
}


pcl::PointCloud<pcl::PointXYZ>::Ptr HR2ISceneViewer::getPickedPointsCloud() {
	return pickedPoints;
}

void HR2ISceneViewer::setGroundCoeffs(const std::vector<float>& ground_coeffs)  {
	this->ground_coeffs = ground_coeffs;
}