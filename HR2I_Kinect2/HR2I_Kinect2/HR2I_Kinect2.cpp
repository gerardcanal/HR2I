#include "HR2I_Kinect2.h"

HR2I_Kinect2::HR2I_Kinect2(BodyRGBViewer* view, HR2ISceneViewer* pcl_viewer) {
	get_data = true;
	body_view = view;
	this->pcl_viewer = pcl_viewer;
}
HR2I_Kinect2::~HR2I_Kinect2() {}

void HR2I_Kinect2::dataGetter(Kinect2Utils* k2u, GestureRecognition* gr, GRParameters params) {
	bool rightBody = true;
	gr_mtx.lock();
	bool _work = get_data;
	bool first = true;
	UINT64 id = 0;
	gr_mtx.unlock();

	ICoordinateMapper* cmapper = NULL;
	k2u->getCoordinateMapper(cmapper);

	while (_work) {
		//IMultiSourceFrame* msf = k2u->getLastMultiSourceFrameFromDefault();
		//if (msf == NULL) continue;

		IDepthFrame* df = k2u->getLastDepthFrameFromDefault();
		IBodyFrame* bodyFrame = k2u->getLastBodyFrameFromDefault();

		if (bodyFrame) {
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, id, first);
			if (body_view != NULL) body_view->setBodyFrameToDraw(bodyFrame);
			if (pcl_viewer != NULL) pcl_viewer->setPerson(sk);
			/// temporal cheat -> Don't use person ID
			if (sk.getTrackingID() > 0) {
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			} // end of cheat
			else if (!first && sk.getTrackingID() == id) {
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
			else if (first && id != sk.getTrackingID()) {
				id = sk.getTrackingID(); // Even though the skeleton is empty -i.e. id == -1- this doesn't change nything
				first = false;
				gr->addFrame(sk.getDynamicGestureRecognitionFeatures(rightBody), sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
		}
		if (df) pcl_viewer->setScene(K2PCL::depthFrameToPointCloud(df, cmapper));
		SafeRelease(bodyFrame); // If not the bodyFrame is not get again
		SafeRelease(df);
		
		if (inputFrames.size() >= params.pointAtTh[2]) inputFrames.pop_front();
		gr_mtx.lock();
		_work = get_data;
		gr_mtx.unlock();
	}
}

hr2i_thesis::GestureRecognitionResult HR2I_Kinect2::recognizeGestures(const string& GRParams_path, const vector<vector<vector<float>>>& models, Kinect2Utils& k2u) {
	const float DESCEND_DIREC_TH = -0.05;
	inputFrames.clear();
	GestureRecognition gr;
	GRParameters params = GestureRecognition::readParameters(GRParams_path);
	hr2i_thesis::GestureRecognitionResult result;

	// Start data getter thread and start gesture recognition from this data
	get_data = true;
	thread datagetter(&HR2I_Kinect2::dataGetter, this, &k2u, &gr, params);
	Gesture gest = gr.RecognizeGesture(models, params);

	// Stop data getter thread as we have recognized a gesture
	gr_mtx.lock();
	get_data = false;
	gr_mtx.unlock();
	datagetter.join();

	// Process recognized gesture
	cout << "Recognized gesture: " << ((gest == SALUTE) ? "HELLO!" : "POINT_AT!") << endl;
	if (gest == POINT_AT) {
		// Take the mean joint points
		vector<float> Hand(3, 0.0);
		vector<float> Elbow(3, 0.0);
		for (int i = 0; i < inputFrames.size(); ++i) {
			CameraSpacePoint h = inputFrames[i].getJointPosition(JointType_HandRight);
			Hand[0] += h.X; Hand[1] += h.Y; Hand[2] += h.Z;
			CameraSpacePoint e = inputFrames[i].getJointPosition(JointType_ElbowRight);
			Elbow[0] += e.X; Elbow[1] += e.Y; Elbow[2] += e.Z;
		}
		Hand[0] /= inputFrames.size(); Hand[1] /= inputFrames.size(); Hand[2] /= inputFrames.size();
		Elbow[0] /= inputFrames.size(); Elbow[1] /= inputFrames.size(); Elbow[2] /= inputFrames.size();

		// Get intersection point
		vector<float> lineVector = Utils::subtract(Hand, Elbow); // Vector Elbow->Hand EH = H-E
		//cout << "\tDirection vector is: (" << lineVector[0] << ", " << lineVector[1] << ", " << lineVector[2] << ")" << endl;
		if (lineVector[1] < DESCEND_DIREC_TH) { // Direction of pointing is descendent. 0.05 To remove some errors...
			//vector<float> n = { 0, 1, 0 }; // Normal vector -> Vertical
			//vector<float> planePoint = { 0, -0.5, 0 }; // Point which corresponds to the floor plane
			vector<float> groundPoint = Utils::linePlaneIntersection(Hand, lineVector, groundplane_point, ground_coeffs); // Hand is a point of the line
			cout << "\tPointed point is: (" << groundPoint[0] << ", " << groundPoint[1] << ", " << groundPoint[2] << ")" << endl;
			result.gestureId = result.idPointAt;
			result.ground_point.x = groundPoint[0];
			result.ground_point.y = groundPoint[1];
			result.ground_point.z = groundPoint[2];
		}
		else {
			result.gestureId = -1; // not recognized
			cout << "\tPointing was not directed to the ground!!!" << endl;
		}
	}
	else if (gest == SALUTE) result.gestureId = result.idHello;
	else result.gestureId = -2; // Strange failure
	return result;
}

vector<vector<vector<float>>> HR2I_Kinect2::readDynamicModels(string gestPath) {
	std::vector<std::vector<std::vector<float>>> models(N_DYNAMIC_GESTURES);
	models[SALUTE] = Skeleton::gestureFeaturesFromCSV(gestPath + "HelloModel/HelloModel_features.csv");
	//models[POINT_AT] = Skeleton::gestureFeaturesFromCSV(gestPath + "PointAtModel/PointAtModel_features.csv");
	//models[POINT_AT] = GestureRecognition::addThirdFeature(models[POINT_AT]);
	return models;
}

/*Deprecated... void HR2I_Kinect2::setPCLScene(IDepthFrame* df, ICoordinateMapper* cmapper, std::vector<float> vec_g_coeffs) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr plane = K2PCL::segmentPlaneByDirection(pcPtr, vec_g_coeffs);
	pcl_viewer->setScene(pcPtr);
}*/

/// Check if ground coefficients are working (true) or need to be recalculated (false)
/// plane_out is an output parameter with the segmented plane
bool HR2I_Kinect2::checkGroundCoefficients(Kinect2Utils* k2u, std::vector<float> vec_g_coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_out) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = getOnePointCloudFromKinect(k2u);
	K2PCL::segmentPlaneByDirection(pcPtr, vec_g_coeffs).swap(plane_out);
	return plane_out->points.size() > 0;
}

std::vector<float> HR2I_Kinect2::computeGroundCoefficientsFromUser(Kinect2Utils* k2u) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = getOnePointCloudFromKinect(k2u);

	// Show scene
	pcl_viewer->setScene(pcPtr, false);

	// Get the three points from user
	pcl_viewer->registerPointPickingCb();
	std::cout << "Shift+click on three floor points (in a CLOCKWISE order)..." << std::endl;
	while (pcl_viewer->getNumPickedPoints() < 3); // Wait until we have enough points
	pcl::PointCloud<pcl::PointXYZ>::Ptr pickedPoints = pcl_viewer->getPickedPointsCloud();

	// Compute the ground plane model coefficients
	Eigen::VectorXf eigen_ground_coeffs;
	eigen_ground_coeffs.resize(4);
	std::vector<int> clicked_points_indices;
	for (unsigned int i = 0; i < pickedPoints->points.size(); i++)
		clicked_points_indices.push_back(i);
	pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(pickedPoints);
	model_plane.computeModelCoefficients(clicked_points_indices, eigen_ground_coeffs);
	std::cout << "\t Selected Ground plane: " << eigen_ground_coeffs(0) << " " << eigen_ground_coeffs(1) << " " << eigen_ground_coeffs(2) << " " << eigen_ground_coeffs(3) << std::endl;

	pcl_viewer->unregisterPointPickingCb();
	std::vector<float> vec_g_coeffs = { eigen_ground_coeffs[0], eigen_ground_coeffs[1], eigen_ground_coeffs[2] };
	this->ground_coeffs = vec_g_coeffs;
	return vec_g_coeffs;
}

void HR2I_Kinect2::writeGroundPlaneCoefficients(std::vector<float> vec_g_coeffs, string path) {
	std::ofstream ofs(path, std::ofstream::out);
	if (!ofs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	ofs << "Ground_Plane_Coefficients: " << vec_g_coeffs[0] << " " << vec_g_coeffs[1] << " " << vec_g_coeffs[2] << endl;
	ofs.close();
}

std::vector<float> HR2I_Kinect2::readGroundPlaneCoefficients(string path) {
	std::ifstream ifs(path, std::ifstream::in);
	if (!ifs.is_open()) {
		std::string err = "ERROR: ground plane coefficients file " + path + " could not be opened or was not found.";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::vector<float> vec_g_coeffs(3);
	string param;
	ifs >> param >> vec_g_coeffs[0] >> vec_g_coeffs[1] >> vec_g_coeffs[2];
	ifs.close(); 
	ground_coeffs = vec_g_coeffs;
	return vec_g_coeffs;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HR2I_Kinect2::getOnePointCloudFromKinect(Kinect2Utils* k2u) {
	ICoordinateMapper* cmapper = NULL;
	k2u->getCoordinateMapper(cmapper);

	// Get correct depth frame and create pointcloud
	IDepthFrame* df = NULL;
	int i = 0;
	while (df == NULL || ++i < 3) { // Enforce to pick 3 frames so we don't have an empty frame
		SafeRelease(df);
		df = k2u->getLastDepthFrameFromDefault();
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
	SafeRelease(df);
	return pcPtr;
}

void HR2I_Kinect2::setGroundInfo(const vector<float>& ground_coeffs, const vector<float>& groundPoint) {
	this->ground_coeffs = ground_coeffs;
	groundplane_point = groundPoint;
}

deque<Skeleton>* HR2I_Kinect2::getInputFrames() {
	return &inputFrames;
}

void HR2I_Kinect2::getAndDrawScene(Kinect2Utils* k2u, pcl::PointXYZ pointingPoint, bool drawBody, bool drawObjects, int obj_radius) {
	bool rightBody = true;

	pcl_viewer->setPointingPoint(pointingPoint);

	ICoordinateMapper* cmapper = NULL;
	k2u->getCoordinateMapper(cmapper);

	IDepthFrame* df = k2u->getLastDepthFrameFromDefault();
	if (drawBody) {
		IBodyFrame* bodyFrame = k2u->getLastBodyFrameFromDefault();

		if (bodyFrame) {
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, 0, true);
			if (body_view != NULL) body_view->setBodyFrameToDraw(bodyFrame);
			if (pcl_viewer != NULL) pcl_viewer->setPerson(sk);
		}
		SafeRelease(bodyFrame); // If not the bodyFrame is not get again
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
	if (drawObjects) {
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects = K2PCL::segmentObjectsNearPointFromScene(pcPtr, OBJECT_RADIUS, pointingPoint);
		pcl_viewer->setSegmentedClusters(objects);
	}
	
	if (df) pcl_viewer->setScene(pcPtr);
	
	SafeRelease(df);
}