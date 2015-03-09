// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#include "HR2I_Kinect2.h"

HR2I_Kinect2::HR2I_Kinect2(BodyRGBViewer* view, HR2ISceneViewer* pcl_viewer, Kinect2Utils* k2u, ros::NodeHandle* nh) {
	get_data = true;
	body_view = view;
	this->pcl_viewer = pcl_viewer;
	this->k2u = k2u;
	this->nh = nh;
	k2cmd.command = -1;
}
HR2I_Kinect2::~HR2I_Kinect2() {}

#define FRAMES_BETWEEN_DF 20
void HR2I_Kinect2::dataGetter(GestureRecognition* gr, GRParameters params) {
	bool rightBody = true;
	gr_mtx.lock();
	bool _work = get_data;
	bool first = true;
	UINT64 id = 0;
	gr_mtx.unlock();

	// Face features aux variables
	std::vector<float> yes_oldFaceOrient(3, 0);
	std::vector<float> no_oldFaceOrient(3, 0);
	int framenum = 0;
	int df_count = 0;

	ICoordinateMapper* cmapper = NULL;
	k2u->getCoordinateMapper(cmapper);

	while (_work) {
		IBodyFrame* bodyFrame = NULL;
		IFaceFrame* faceFrame = k2u->getLastFaceFrameFromDefault();

		std::vector<float> yes_face_frame_feat; // Face Frame features
		std::vector<float> no_face_frame_feat; // Face Frame features


		if (faceFrame) {
			Face f = Kinect2Utils::getFaceFromFaceFrame(faceFrame, false); // true - infrared, false color
			if (!f.getIsEmpty()) {
				if (body_view != NULL) body_view->setFaceFrameToDraw(f);
				//// Get features
				double pitch, yaw, roll;
				Utils::ExtractFaceRotationInDegrees(&f.getFaceRotation(), &pitch, &yaw, &roll);
				std::vector<float> orientation = { (float)pitch, (float)yaw, (float)roll };

				yes_face_frame_feat = Face::computeFrameFeatures(params.DynParams[NOD][0], params.DynParams[NOD][1], ++framenum, orientation, yes_oldFaceOrient);
				if (yes_face_frame_feat.size() > 0) yes_oldFaceOrient = orientation;

				no_face_frame_feat = Face::computeFrameFeatures(params.DynParams[NEGATE][0], params.DynParams[NEGATE][1], framenum, orientation, no_oldFaceOrient);
				if (no_face_frame_feat.size() > 0) no_oldFaceOrient = orientation;
				////
			}
			if (bodyFrame == NULL) {
				IBodyFrameReference* bfr = NULL;
				faceFrame->get_BodyFrameReference(&bfr);
				bfr->AcquireFrame(&bodyFrame);
				SafeRelease(bfr);
			}
		}
		/*else {
			bodyFrame = k2u->getLastBodyFrameFromDefault();
		}*/

		if (bodyFrame) {
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, id, first);
			if (body_view != NULL) body_view->setBodyFrameToDraw(bodyFrame);
			if (pcl_viewer != NULL) pcl_viewer->setPerson(sk);

			//std::cout << "sk trackid: " << sk.getTrackingID() << std::endl;

			/// temporal cheat -> Don't use person ID
			if (sk.getTrackingID() > 0) {
				k2u->setFaceTrackingId(sk.getTrackingID());
				gr->addFrame({ sk.getDynamicGestureRecognitionFeatures(rightBody), yes_face_frame_feat, no_face_frame_feat }, sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			} // end of cheat
			else if (!first && sk.getTrackingID() == id) {
				gr->addFrame({ sk.getDynamicGestureRecognitionFeatures(rightBody), yes_face_frame_feat, no_face_frame_feat }, sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
			else if (first && id != sk.getTrackingID()) {
				id = sk.getTrackingID(); // Even though the skeleton is empty -i.e. id == -1- this doesn't change nything
				first = false;
				gr->addFrame({ sk.getDynamicGestureRecognitionFeatures(rightBody), yes_face_frame_feat, no_face_frame_feat }, sk.getStaticGestureRecognitionFeatures(rightBody, true));
				inputFrames.push_back(sk);
			}
		}
		else if (yes_face_frame_feat.size() > 0) {
			// Add facial features but no skeletal ones
			gr->addFrame({ std::vector<float>(), yes_face_frame_feat, no_face_frame_feat }, std::vector<float>());
		}
		SafeRelease(faceFrame);
		SafeRelease(bodyFrame); // If not the bodyFrame is not get again

		if (++df_count % FRAMES_BETWEEN_DF == 0) {
			IDepthFrame* df = k2u->getLastDepthFrameFromDefault();
			if (df && pcl_viewer != NULL) pcl_viewer->setScene(K2PCL::depthFrameToPointCloud(df, cmapper));
			SafeRelease(df);
		}
		
		if (inputFrames.size() >= params.pointAtTh[2]) inputFrames.pop_front();
		gr_mtx.lock();
		_work = get_data;
		gr_mtx.unlock();
#ifdef USE_ROS_HR2I
		nh->spinOnce();// to make sure
#endif
	}
}

hr2i_thesis::GestureRecognitionResult HR2I_Kinect2::recognizeGestures(const vector<vector<vector<float>>>& models) {
	const float DESCEND_DIREC_TH = -0.05;
	inputFrames.clear();
	GestureRecognition gr;

	hr2i_thesis::GestureRecognitionResult result;

	// Start data getter thread and start gesture recognition from this data
	get_data = true;
	thread datagetter(&HR2I_Kinect2::dataGetter, this, &gr, params);
	Gesture gest = gr.RecognizeGesture(models, params);

	// Stop data getter thread as we have recognized a gesture
	gr_mtx.lock();
	get_data = false;
	gr_mtx.unlock();
	datagetter.join();

	// Process recognized gesture
	cout << "Recognized gesture: ";
	if (gest == POINT_AT) {
		cout << "POINT_AT!" << endl;
		// Take the mean joint points
		vector<float> Hand(3, 0.0);
		vector<float> Elbow(3, 0.0);
		vector<float> HandTip(3, 0.0);
		// Average the frames to have a medium vector
		int start = std::min(10, (int)inputFrames.size()), numEl = 10;
		int k = 0;
		for (int i = start; i < std::min(start+numEl, (int)inputFrames.size()); ++i) {
			CameraSpacePoint h = inputFrames[i].getJointPosition(JointType_HandRight);
			Hand[0] += h.X; Hand[1] += h.Y; Hand[2] += h.Z;
			CameraSpacePoint e = inputFrames[i].getJointPosition(JointType_ElbowRight);
			Elbow[0] += e.X; Elbow[1] += e.Y; Elbow[2] += e.Z;
			CameraSpacePoint ht = inputFrames[i].getJointPosition(JointType_HandTipRight);
			HandTip[0] += ht.X; HandTip[1] += ht.Y; HandTip[2] += ht.Z;
			k++;
		}
		std::cout << "\tMean point at direction used " << k << " frames." << std::endl;
		Hand[0] /= k; Hand[1] /= k; Hand[2] /= k;
		Elbow[0] /= k; Elbow[1] /= numEl; Elbow[2] /= numEl;
		HandTip[0] /= k; HandTip[1] /= k; HandTip[2] /= k;
		/*CameraSpacePoint h = inputFrames[inputFrames.size()-7].getJointPosition(JointType_HandRight); // 7 is the magic number
		Hand[0] += h.X; Hand[1] += h.Y; Hand[2] += h.Z;
		CameraSpacePoint e = inputFrames[inputFrames.size() - 7].getJointPosition(JointType_ElbowRight);
		Elbow[0] += e.X; Elbow[1] += e.Y; Elbow[2] += e.Z;
		CameraSpacePoint ht = inputFrames[inputFrames.size() - 7].getJointPosition(JointType_HandTipRight);
		HandTip[0] += ht.X; HandTip[1] += ht.Y; HandTip[2] += ht.Z;*/

		// Get intersection point
		vector<float> lineVector = Utils::subtract(Hand, Elbow); // Vector Elbow->Hand EH = H-E
		//vector<float> lineVector = Utils::subtract(HandTip, Elbow); // Vector Elbow->Hand EHt = Ht-E
		//vector<float> lineVector = Utils::subtract(HandTip, Hand); // Vector Hand->HandTip HHT = Ht-H
		//cout << "\tDirection vector is: (" << lineVector[0] << ", " << lineVector[1] << ", " << lineVector[2] << ")" << endl;
		if (lineVector[1] < DESCEND_DIREC_TH) { // Direction of pointing is descendent. 0.05 To remove some errors...
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
	else if (gest == WAVE) {
		cout << "WAVE!" << endl;
		result.gestureId = result.idHello;
	}
	else if (gest == NOD) {
		cout << "NOD!" << endl;
		result.gestureId = result.idNod;
	}
	else if (gest == NEGATE) {
		cout << "NEGATE!" << endl;
		result.gestureId = result.idNegate;
	}
	else result.gestureId = -2; // Strange failure
	result.header.frame_id = "Kinect2";

	// Person position will be the Spin Base join while doing the gesture
	int indx = max(0, (int)inputFrames.size() - 5);
	CameraSpacePoint ppos = inputFrames[indx].getJointPosition(JointType_SpineBase);
	result.person_position.x = ppos.X;
	result.person_position.y = ppos.Y;
	result.person_position.z = ppos.Z;
	return result;
}

void HR2I_Kinect2::loadParameters(const string& GRParams_path) {
	this->params = GestureRecognition::readParameters(GRParams_path);
}

vector<vector<vector<float>>> HR2I_Kinect2::readDynamicModels(string gestPath) {
	std::vector<std::vector<std::vector<float>>> models(N_DYNAMIC_GESTURES);
	// WAVE
	models[WAVE] = Skeleton::gestureFeaturesFromCSV(gestPath + "HelloModel/HelloModel_features.csv");
	
	// Regenerate facial features for the models
	std::vector<Face> gesture = Face::faceGestureFromCSV(gestPath + "YesFacialModel/YesFacialModel_faces.csv");
	models[NOD] = Face::getFeatures(params.DynParams[NOD][0], params.DynParams[NOD][1], gesture);
	gesture = Face::faceGestureFromCSV(gestPath + "YesFacialModel/YesFacialModel_faces.csv");
	models[NEGATE] = Face::getFeatures(params.DynParams[NEGATE][0], params.DynParams[NEGATE][1], gesture);

	// The precomputed features seem to work better than the actual ones...
	models[NOD] = Face::gestureFeaturesFromCSV(gestPath + "YesFacialModel/YesFacialModel_faces_features.csv");
	models[NEGATE] = Face::gestureFeaturesFromCSV(gestPath + "NoFacialModel/NoFacialModel_faces_features.csv");
	
	
	//models[POINT_AT] = Skeleton::gestureFeaturesFromCSV(gestPath + "PointAtModel/PointAtModel_features.csv");
	//models[POINT_AT] = GestureRecognition::addThirdFeature(models[POINT_AT]);
	
	
	this->_models = models;
	return models;
}

/*Deprecated... void HR2I_Kinect2::setPCLScene(IDepthFrame* df, ICoordinateMapper* cmapper, std::vector<float> vec_g_coeffs) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr plane = K2PCL::segmentPlaneByDirection(pcPtr, vec_g_coeffs);
	pcl_viewer->setScene(pcPtr);
}*/

/// Check if ground coefficients are working (true) or need to be recalculated (false)
/// plane_out is an output parameter with the segmented plane
bool HR2I_Kinect2::checkGroundCoefficients(std::vector<float> vec_g_coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_out) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = getOnePointCloudFromKinect();
	K2PCL::segmentPlaneByDirection(pcPtr, vec_g_coeffs).swap(plane_out);
	return plane_out->points.size() > 0;
}

std::vector<float> HR2I_Kinect2::computeGroundCoefficientsFromUser() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = getOnePointCloudFromKinect();

	// Show scene
	pcl_viewer->setScene(pcPtr, false);

	// Get the three points from user
	pcl_viewer->registerPointPickingCb();
	std::cout << "Shift+click on three floor points (in a CLOCKWISE order)..." << std::endl;
	while (pcl_viewer->getNumPickedPoints() < 3) nh->spinOnce(); // Wait until we have enough points
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

pcl::PointCloud<pcl::PointXYZ>::Ptr HR2I_Kinect2::getOnePointCloudFromKinect() {
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

/// ObjectsPtr is an output parameter with the segmented clusters
void HR2I_Kinect2::getAndDrawScene(pcl::PointXYZ pointingPoint, bool drawBody, bool drawObjects, double obj_radius, double clust_tol, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* objectsPtr) {
	bool rightBody = true;

	if (pcl_viewer != NULL) pcl_viewer->setPointingPoint(pointingPoint);

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
	if (df) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcPtr = K2PCL::depthFrameToPointCloud(df, cmapper);
		if (drawObjects) {
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _objects = K2PCL::segmentObjectsNearPointFromScene(pcPtr, obj_radius, pointingPoint, clust_tol);
			if (pcl_viewer != NULL) pcl_viewer->setSceneAndSegmentedClusters(pcPtr, _objects);
			std::cout << _objects.size() << " clusters have been segmented." << std::endl;
			if (objectsPtr != NULL) *objectsPtr = _objects;
		}
		else if (pcl_viewer != NULL) pcl_viewer->setScene(pcPtr);
	}
	
	SafeRelease(df);
}

#define Y_FACTOR 1.03 // 1.15 before
#define X_OFFSET 0.07 // To correct user deviations
#define Z_OFFSET 0.125 // To correct user deviations
// ROS handling and States
void HR2I_Kinect2::recognizeGestureState(const string& gr_models_path, ros::Publisher* gesture_pub) {
	currentState = GESTURE_RECOGNITION;
	cout << "State: Recognizing gestures..." << endl;
	// Call gesture recognition
	hr2i_thesis::GestureRecognitionResult gr_res = recognizeGestures(this->_models);
	gesture_pub->publish(&gr_res);
	currentState = WAITING; // As we already made the recognition
	if (gr_res.gestureId == gr_res.idPointAt) {
		this->pointingPoint = pcl::PointXYZ(gr_res.ground_point.x, gr_res.ground_point.y, gr_res.ground_point.z);
		
		/// UPDATE POINT TO CORRECT USER DEVIATIONS... 
		pcl::PointXYZ _newpoint = pointingPoint;
		_newpoint.x += X_OFFSET+0.03;
		_newpoint.z += Z_OFFSET+0.03;
		/// END USER CORRECTION

		getAndDrawScene(pointingPoint, true, true, OBJECT_RADIUS);
		// Sleep for some time unless we get the message... Sleep(SHOWING_GESTURE_TIME);
		for (int i = 0; i < SHOWING_GESTURE_TIME / 50; ++i) {
			roscmd_mtx.lock();
			if (k2cmd.command != -1) break;
			roscmd_mtx.unlock();
#ifdef USE_ROS_HR2I
			nh->spinOnce();
#endif
			Sleep(50);
		}
		if (pcl_viewer != NULL) pcl_viewer->setPointingPoint(pcl::PointXYZ(0, 0, 0));
	}
}

void HR2I_Kinect2::clusterObjectsState(ros::Publisher* clusters_pub) {
	currentState = CLUSTER_SEGMENTATION;
	cout << "State: Segmenting objects near a point..." << endl;
	// Get and update ppoint
	roscmd_mtx.lock();
	hr2i_thesis::Kinect2Command _k2cmdcp = k2cmd;
	roscmd_mtx.unlock();

	pcl::PointXYZ newPpoint = pointingPoint;
	if (strcmp(_k2cmdcp.header.frame_id, "wifibot") == 0) { // Convert pose from wifibot to Kinect: wb+x = kinect+z, wb+y = kinect+x 
		// Translate the point back to 0
		newPpoint.x = pointingPoint.x - _k2cmdcp.current_pose.y;
		newPpoint.y = pointingPoint.y*Y_FACTOR; // 1.15 before It's the same height, but as the kinect is inclined it looks like above. This is only for displaying purposes. 1.30 in the UPC
		newPpoint.z = pointingPoint.z - _k2cmdcp.current_pose.x;
		// Rotate the point
		_k2cmdcp.current_pose.theta = -_k2cmdcp.current_pose.theta/OVERANGLE_CORRECTION; // As the wifibot turns less than what it should, the angle is overincremented to make it reach goals.
		//So we have to correct the new angle to the real one
		newPpoint.z = newPpoint.z*cos(_k2cmdcp.current_pose.theta) - newPpoint.x*sin(_k2cmdcp.current_pose.theta);
		newPpoint.x = newPpoint.z*sin(_k2cmdcp.current_pose.theta) + newPpoint.x*cos(_k2cmdcp.current_pose.theta);

		// Correct deviations by moving point forward and right
		newPpoint.z += Z_OFFSET;
		newPpoint.x += X_OFFSET;
	}
	cout << "\tUpdated pointing location is: (" << newPpoint.x << ", " << newPpoint.y << ", " << newPpoint.z << ")" << endl;
	// Remove body
	if (pcl_viewer != NULL) pcl_viewer->setPerson(Skeleton()); // Set empty skeleton
	// Segment objects
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects;
	getAndDrawScene(newPpoint, false, true, OBJECT_RADIUS_CLOSE, CLOSE_CLUSTER_TOLERANCE, &objects);

	// Fill and send message
	hr2i_thesis::PointCloudClusterCentroids msg;
	const int numObjects = objects.size();
	msg.cluster_centroids_length = numObjects;
	msg.cluster_sizes_length = numObjects;
	geometry_msgs::Point* centroids = new geometry_msgs::Point[numObjects];
	float* sizes = new float[numObjects];
	for (int i = 0; i < numObjects; ++i) {
		pcl::PointXYZ cent = K2PCL::compute3DCentroid(objects[i]);
		centroids[i].x = cent.x; centroids[i].y = cent.y; centroids[i].z = cent.z;
		pair<double, double> areavol = K2PCL::computeAreaVolume(objects[i]);
		sizes[i] = areavol.first;
	}
	msg.cluster_centroids = centroids;
	msg.cluster_sizes = sizes;
	msg.header.frame_id = "Kinect2";
	clusters_pub->publish(&msg); // Send message

	// Show scene for a while...
	currentState = WAITING; // As we already sent the clustering
	for (int i = 0; i < SHOWING_CLUSTERS_TIME / 50; ++i) {
		roscmd_mtx.lock();
		if (k2cmd.command != -1) break;
		roscmd_mtx.unlock();
#ifdef USE_ROS_HR2I
		nh->spinOnce();
#endif
		Sleep(50);
	}
	if (pcl_viewer != NULL) pcl_viewer->setPointingPoint(pcl::PointXYZ(0, 0, 0));

	// Free memory (now just in case we free it before it has been sent or somehting like this)
	delete[] centroids; // Message is sent, we can free the memory
	delete[] sizes; // Message is sent, we can free the memory
}

hr2i_thesis::Kinect2Command HR2I_Kinect2::waitForCommandState() {
	currentState = WAITING; // As we already made the recognition
	cout << "State: Waiting for command from wifibot..." << endl;
	roscmd_mtx.lock();
	hr2i_thesis::Kinect2Command cmd = k2cmd;
	roscmd_mtx.unlock();
	ICoordinateMapper* cmapper = NULL;
	k2u->getCoordinateMapper(cmapper);

	while (cmd.command == -1) {
		// Display kinect PCL info
		IBodyFrame* bodyFrame = NULL;// k2u->getLastBodyFrameFromDefault();
		IFaceFrame* faceFrame = k2u->getLastFaceFrameFromDefault();

		if (faceFrame) {
			Face f = Kinect2Utils::getFaceFromFaceFrame(faceFrame, false); // true - infrared, false color
			if (body_view != NULL && !f.getIsEmpty()) body_view->setFaceFrameToDraw(f);
			if (bodyFrame == NULL) {
				IBodyFrameReference* bfr = NULL;
				faceFrame->get_BodyFrameReference(&bfr);
				bfr->AcquireFrame(&bodyFrame);
				SafeRelease(bfr);
			}
		}

		if (bodyFrame) {
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, 0, true);
			if (sk.getTrackingID() > 0) k2u->setFaceTrackingId(sk.getTrackingID());
			if (body_view != NULL) body_view->setBodyFrameToDraw(bodyFrame);
			if (pcl_viewer != NULL) pcl_viewer->setPerson(sk);
		}
		SafeRelease(faceFrame);
		SafeRelease(bodyFrame);

		IDepthFrame* df = k2u->getLastDepthFrameFromDefault();
		if (df && pcl_viewer != NULL) pcl_viewer->setScene(K2PCL::depthFrameToPointCloud(df, cmapper));
		SafeRelease(df);

#ifdef USE_ROS_HR2I
		nh->spinOnce();
#endif
		// Check if received
		roscmd_mtx.lock();
		cmd = k2cmd;
		roscmd_mtx.unlock();
	}
	roscmd_mtx.lock();
	k2cmd.command = -1; // We already read that command...
	roscmd_mtx.unlock();
	return cmd;
}


void HR2I_Kinect2::k2CommandReceivedCb(const hr2i_thesis::Kinect2Command& cmd) {
	roscmd_mtx.lock();
	k2cmd = hr2i_thesis::Kinect2Command(cmd);
	roscmd_mtx.unlock();
}

HR2I_Kinect2::State HR2I_Kinect2::getCurrentState() {
	return currentState;
}