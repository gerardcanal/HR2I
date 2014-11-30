#include "HR2I_Kinect.h"

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
	while (_work) {
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
		SafeRelease(bodyFrame); // If not the bodyFrame is not get again
		if (inputFrames.size() >= params.pointAtTh[2]) inputFrames.pop_front();
		gr_mtx.lock();
		_work = get_data;
		gr_mtx.unlock();
	}
}

hr2i_thesis::GestureRecognitionResult HR2I_Kinect2::recognizeGestures(const string& GRParams_path, const vector<vector<vector<float>>>& models, Kinect2Utils& k2u) {
	const float DESCEND_DIREC_TH = -0.05;
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
			vector<float> n = { 0, 1, 0 }; // Normal vector -> Vertical
			vector<float> planePoint = { 0, -0.5, 0 }; // Point which corresponds to the floor plane
			vector<float> groundPoint = Utils::linePlaneIntersection(Hand, lineVector, planePoint, n); // Hand is a point of the line
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