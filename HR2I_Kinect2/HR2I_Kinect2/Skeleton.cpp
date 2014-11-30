#include "stdafx.h"
#include "Skeleton.h"
#include <exception>
#include <assert.h>

Skeleton::Skeleton(HandState left, HandState right,
				   TrackingConfidence leftTC, TrackingConfidence rightTC,
				   BOOLEAN isTracked, UINT64 id,
				   JointOrientation jOrient[], Joint joints[]) {
	setHandState(left, right);
	setHandTrackingConfidence(leftTC, rightTC);
	setTracking(isTracked, id);
	setJointsInfo(jOrient, joints);
}

Skeleton::Skeleton() {
	trackingId = 0;
}

/*Skeleton::Skeleton(const Skeleton& s) {
	setHandState(s.leftHs, s.rightHs);
	setHandTrackingConfidence(s.leftTc, s.rightTc);
	setTracking(s.isTracked, s.trackingId);
	for (int i = 0; i < _countof(jointOrientations); ++i) {
		jointOrientations[i] = s.jointOrientations[i];
		joints[i] = s.joints[i];
	}
}*/

Skeleton::~Skeleton()
{
	/*delete[] joints;
	delete[] jointOrientations;
	int x; std::cout << "THIS IS THE SKELETON DESTROYER!!!" << std::endl;*/
}


void Skeleton::setHandState(HandState left, HandState right) {
	leftHs = left;
	rightHs = right;
}

void Skeleton::setHandTrackingConfidence(TrackingConfidence left, TrackingConfidence right) {
	leftTc = left;
	rightTc = right;
}

void Skeleton::setTracking(BOOLEAN isTracked, UINT64 id) {
	this->isTracked = isTracked;
	trackingId = id;
}

void Skeleton::setJointsInfo(JointOrientation jOrient[], Joint joints[]) {
	//assert(_countof(jointOrientations) == _countof(jOrient) && _countof(this->joints) == _countof(joints));
	for (int i = 0; i < _countof(jointOrientations); ++i) {
		jointOrientations[i] = jOrient[i];
		this->joints[i] = joints[i];
	}
}

/// Returns the left hand state and the right one, in an array
std::array<HandState, 2> Skeleton::getHandState() {
	std::array<HandState, 2> ret = { leftHs, rightHs };
	return ret;
}

// Returns left tracking confidence and right, in an array with this order
std::array<TrackingConfidence, 2> Skeleton::getHandTrackingConfidence(){
	std::array<TrackingConfidence, 2> ret = { leftTc, rightTc };
	return ret;
}

bool Skeleton::getIsTracked() {
	return isTracked;
}

UINT64 Skeleton::getTrackingID() {
	return trackingId;
}

JointOrientation* Skeleton::getJointOrientations() {
	return jointOrientations;
}

Joint* Skeleton::getJoints() {
	return joints;
}

CameraSpacePoint Skeleton::getJointPosition(JointType j) {
	return joints[j].Position;
}

Joint Skeleton::getJoint(JointType j) {
	return joints[j];
}

bool Skeleton::operator==(const Skeleton& b) {
	bool equal = (leftHs == b.leftHs) && (leftTc == b.leftTc) && (rightHs == b.rightHs) && (rightTc == b.rightTc);
	equal &= (isTracked == b.isTracked) && (trackingId == b.trackingId);
	
	//Joints
	for (int i = 0; i < nJoints && equal; ++i) {
		equal &= (joints[i].JointType == b.joints[i].JointType);
		equal &= (joints[i].Position.X == b.joints[i].Position.X) && (joints[i].Position.Y == b.joints[i].Position.Y) && (joints[i].Position.Z == b.joints[i].Position.Z);
		equal &= (joints[i].TrackingState == b.joints[i].TrackingState);

		// Orientations
		equal &= (jointOrientations[i].JointType == b.jointOrientations[i].JointType);
		equal &= (jointOrientations[i].Orientation.w == b.jointOrientations[i].Orientation.w) && (jointOrientations[i].Orientation.x == b.jointOrientations[i].Orientation.x)
			  && (jointOrientations[i].Orientation.y == b.jointOrientations[i].Orientation.y) && (jointOrientations[i].Orientation.z == b.jointOrientations[i].Orientation.z);
	}
	return equal;
}

std::ostream& operator<<(std::ostream& os, const Skeleton& sk) {
	os << sk.leftHs << ", " << sk.leftTc << ", " << sk.rightHs << ", " << sk.rightTc << ", ";
	os << sk.isTracked << ", " << sk.trackingId << ", ";
	
	os << std::setprecision(std::numeric_limits<float>::max_digits10); // Set max precision
	//Joints
	for (int i = 0; i < sk.nJoints; ++i) 
		os << sk.joints[i].JointType << " " << sk.joints[i].Position.X << " " << sk.joints[i].Position.Y << " " << sk.joints[i].Position.Z << " " << sk.joints[i].TrackingState << ", ";

	// Joint orientations
	for (int i = 0; i < sk.nJoints; ++i) {
		os << sk.jointOrientations[i].JointType << " " << sk.jointOrientations[i].Orientation.w << " " << sk.jointOrientations[i].Orientation.x << " ";
		os << sk.jointOrientations[i].Orientation.y << " " << sk.jointOrientations[i].Orientation.z << ((i + 1 == sk.nJoints) ? "" : ", ");
	}
	os << std::endl;
	return os;
}

void Skeleton::gestureToCSV(std::vector<Skeleton> gesture, std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ofstream ofs(path, std::ofstream::out);
	if (!ofs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}

	// Write header
	ofs << "sep=, " << std::endl;
	ofs << "LeftHandState, LeftHandTrackingConfidence, RightHandState, RightHandTrackingConfidence, TrackingState, TrackingID, ";
	// TODO change joint number by name?
	int i = 0;
	for (i = 0; i < nJoints; ++i) ofs << "Joint" << i << ", ";
	for (i = 0; i < nJoints; ++i) ofs << "JointOrientation" << i << ((i + 1 == nJoints)? "" : ", ");
	ofs << std::endl;

	// Add the elements for each gesture
	for (int i = 0; i < gesture.size(); ++i) ofs << gesture[i];
	ofs.close();
}

std::vector<Skeleton> Skeleton::gestureFromCSV(std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ifstream ifs(path, std::ifstream::in);
	if (!ifs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::vector<Skeleton> gesture;
	std::string line;
	std::getline(ifs, line); // Read header line
	while (line.find("sep") != line.npos) 
		std::getline(ifs, line);

	std::string delim;
	int aux;
	while (std::getline(ifs, line)) {
		Skeleton frame;
		std::istringstream iss(line);

		// Hands
		iss >> aux >> delim;
		assert(delim == ","); // Just to avoid errors while programming...
		frame.leftHs = static_cast<HandState>(aux);
		iss >> aux >> delim;
		assert(delim == ",");
		frame.leftTc = static_cast<TrackingConfidence>(aux);
		iss >> aux >> delim;
		assert(delim == ",");
		frame.rightHs = static_cast<HandState>(aux);
		iss >> aux >> delim;
		assert(delim == ",");
		frame.rightTc = static_cast<TrackingConfidence>(aux);

		// Tracking
		bool trstate;
		iss >> trstate >> delim;
		assert(delim == ",");
		frame.isTracked = static_cast<BOOLEAN>(trstate);
		iss >> frame.trackingId >> delim;
		assert(delim == ",");

		// Joints
		for (int i = 0; i < frame.nJoints; ++i) {
			iss >> aux >> frame.joints[i].Position.X >> frame.joints[i].Position.Y >> frame.joints[i].Position.Z;
			frame.joints[i].JointType = static_cast<JointType>(aux);
			iss >> aux >> delim;
			assert(delim == ",");
			frame.joints[i].TrackingState = static_cast<TrackingState>(aux);
		}
		for (int i = 0; i < frame.nJoints; ++i) {
			iss >> aux >> frame.jointOrientations[i].Orientation.w >> frame.jointOrientations[i].Orientation.x >> frame.jointOrientations[i].Orientation.y;
			iss >> frame.jointOrientations[i].Orientation.z;
			frame.jointOrientations[i].JointType = static_cast<JointType>(aux);
			if (i + 1 < frame.nJoints) {
				iss >> delim;
				assert(delim == ",");
			}
		}

		// Store this frame
		gesture.push_back(frame);
	}


	ifs.close();
	return gesture;
}


void Skeleton::gestureFeaturesToCSV(std::vector<Skeleton> gesture, std::string path) {
	bool rightBody = true;
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ofstream ofs(path, std::ofstream::out);
	if (!ofs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}

	// Write header
	ofs << "sep=, " << std::endl;
	ofs << "Hand-Neck Distance (m), Elbow Angle (rad)" << std::endl; // Maybe should be adapted in case we have more features
	ofs << std::setprecision(std::numeric_limits<float>::max_digits10); // Set max precision
	for (int i = 0; i < gesture.size(); ++i) {
		std::vector<float> feat = gesture[i].getDynamicGestureRecognitionFeatures(rightBody);
		for (int j = 0; j < feat.size(); ++j) ofs << feat[j] << ((j + 1 == feat.size()) ? "": ", ");
		ofs << std::endl;
	}
	ofs.close();
}

std::vector<std::vector<float>> Skeleton::gestureFeaturesFromCSV(std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ifstream ifs(path, std::ifstream::in);
	if (!ifs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::vector<std::vector<float>> gestureFeatures;
	std::string line;
	std::getline(ifs, line); // Read header line
	while (line.find("sep") != line.npos)
		std::getline(ifs, line);

	std::string delim;
	float aux;
	while (std::getline(ifs, line)) {
		std::vector<float> feat;
		std::istringstream iss(line);

		iss >> aux >> delim; // First feature
		assert(delim == ","); // Just to avoid errors while programming...
		feat.push_back(aux); // Second feature
		iss >> aux;
		feat.push_back(aux);
		gestureFeatures.push_back(feat);
	}
	ifs.close();
	return gestureFeatures;
}

/// Returns two features: the distance between neck and hand and the angle of the arm and the elbow
std::vector<float> Skeleton::getDynamicGestureRecognitionFeatures(bool rightBody) {
	float hand_neck_dist, shoulder_elbow_hand_angle;
	if (rightBody) {
		hand_neck_dist = Utils::euclideanDistance(joints[JointType_HandRight], joints[JointType_Neck]);
		shoulder_elbow_hand_angle = Utils::getAngleBetween(joints[JointType_ShoulderRight], joints[JointType_ElbowRight], joints[JointType_HandRight], true);
	}
	else {
		hand_neck_dist = Utils::euclideanDistance(joints[JointType_HandLeft], joints[JointType_Neck]);
		shoulder_elbow_hand_angle = Utils::getAngleBetween(joints[JointType_ShoulderLeft], joints[JointType_ElbowLeft], joints[JointType_HandLeft], true);
	}
	std::vector<float> feature = { hand_neck_dist, shoulder_elbow_hand_angle };
	return feature;
}

/// Returns two features: the distance between hand and hip and the angle of the arm and the elbow.
/// If addHandPose is true, it adds the position of the hand
std::vector<float> Skeleton::getStaticGestureRecognitionFeatures(bool rightBody, bool addHandPose) {
	float hand_hip_dist, shoulder_elbow_hand_angle;
	if (rightBody) {
		hand_hip_dist = Utils::euclideanDistance(joints[JointType_HandRight], joints[JointType_HipRight]);
		shoulder_elbow_hand_angle = Utils::getAngleBetween(joints[JointType_ShoulderRight], joints[JointType_ElbowRight], joints[JointType_HandRight], true);
	}
	else {
		hand_hip_dist = Utils::euclideanDistance(joints[JointType_HandLeft], joints[JointType_HipLeft]);
		shoulder_elbow_hand_angle = Utils::getAngleBetween(joints[JointType_ShoulderLeft], joints[JointType_ElbowLeft], joints[JointType_HandLeft], true);
	}
	std::vector<float> feature = { hand_hip_dist, shoulder_elbow_hand_angle };

	if (addHandPose) {
		if (rightBody) {
			feature.push_back(joints[JointType_HandRight].Position.X);
			feature.push_back(joints[JointType_HandRight].Position.Y);
			feature.push_back(joints[JointType_HandRight].Position.Z);
		}
		else {
			feature.push_back(joints[JointType_HandLeft].Position.X);
			feature.push_back(joints[JointType_HandLeft].Position.Y);
			feature.push_back(joints[JointType_HandLeft].Position.Z);
		}
	}

	return feature;
}

/*/// Same as Skeleton::getGestureRecognitionFeatures but adds distance from elbow to spin_mid as the third feature
/// used in the "POINT_AT" gesture to disambiguate with resting position
std::vector<float> Skeleton::getExtendedGestureRecognitionFeatures(bool rightBody, float th) {
	std::vector<float> feature = getGestureRecognitionFeatures(rightBody);
	float dist_elbow_spinmid = Utils::euclideanDistance(joints[JointType_SpineMid], (rightBody)? joints[JointType_ElbowRight] : joints[JointType_ElbowLeft]);
	feature.push_back((dist_elbow_spinmid < th)? 1.0f : 0.0f);
	return feature;
}

void Skeleton::addExtendedGRFeature(std::vector<float>& feature, bool rightBody, float th) {
	float dist_elbow_spinmid = Utils::euclideanDistance(joints[JointType_SpineMid], (rightBody)? joints[JointType_ElbowRight] : joints[JointType_ElbowLeft]);
	feature.push_back((dist_elbow_spinmid < th) ? 1.0f : 0.0f);
}

float Skeleton::getElbowSpineDistance(bool rightBody) {
	return Utils::euclideanDistance(joints[JointType_SpineMid], (rightBody) ? joints[JointType_ElbowRight] : joints[JointType_ElbowLeft]);
}*/