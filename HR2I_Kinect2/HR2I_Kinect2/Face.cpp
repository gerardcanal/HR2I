// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#include "stdafx.h"
#include "Face.h"


Face::Face(RectI box, PointF points[], Vector4 rotation, DetectionResult properties[], bool _inInfraredSpace) {
	isEmpty = false;
	this->inInfraredSpace = _inInfraredSpace;
	setBoundingBox(box);
	setFacePoints(points, _inInfraredSpace);
	setFaceProperties(properties);
	setFaceRotation(rotation);
	if (checkAllZero()) isEmpty = true;
}

Face::Face() {
	isEmpty = true;
	inInfraredSpace = true;
}


Face::~Face() {
}

// Setters
void Face::setBoundingBox(RectI bbox) { faceBBox = bbox; }
void Face::setFacePoints(PointF facePoints[], bool inInfraredSpace) {
	this->inInfraredSpace = inInfraredSpace;
	for (int i = 0; i < FacePointType::FacePointType_Count; ++i) {
		this->facePoints[i] = facePoints[i];
	}
	isEmpty = false;
}
void Face::setFaceRotation(Vector4 faceRotation) { this->faceRotation = faceRotation; }
void Face::setFaceProperties(DetectionResult dres[]) {
	for (int i = 0; i < FaceProperty::FaceProperty_Count; ++i) {
		faceProperties[i] = dres[i];
	}
}

// Getters
RectI Face::getBoundingBox() { return faceBBox; }
PointF* Face::getFacePoints() { return facePoints; }
Vector4 Face::getFaceRotation() { return faceRotation; }
DetectionResult* Face::getFaceProperties() { return faceProperties; }
bool Face::getIsEmpty() { return isEmpty; }
bool Face::getIsInInfraredSpace() { return inInfraredSpace; }

bool Face::checkAllZero() {
	if (faceBBox.Bottom != 0 || faceBBox.Top != 0 || faceBBox.Left != 0 || faceBBox.Right != 0) return false;
	for (int i = 0; i < FacePointType::FacePointType_Count; ++i) {
		if (facePoints[i].X != 0 || facePoints[i].Y != 0) return false;
	}
	return true;
}

std::ostream& operator<<(std::ostream& os, const Face& f) {
	os << std::setprecision(std::numeric_limits<float>::max_digits10); // Set max precision
	// Space
	os << f.inInfraredSpace << ", ";

	// Bounding Box
	os << f.faceBBox.Top << " " << f.faceBBox.Bottom << " " << f.faceBBox.Left << " " << f.faceBBox.Right << ", ";

	// Face points
	for (int i = 0; i < FacePointType::FacePointType_Count; ++i) {
		os << f.facePoints[i].X << " " << f.facePoints[i].Y << ", ";
	}

	// Face Rotation
	os << f.faceRotation.x << " " << f.faceRotation.y << " " << f.faceRotation.z << " " << f.faceRotation.w << ", ";

	// Face Properties
	for (int i = 0; i < FaceProperty::FaceProperty_Count; ++i) {
		os << f.faceProperties[i] << ((i + 1 == FaceProperty::FaceProperty_Count) ? "" : ", ");
	}
	os << std::endl;
	return os;
}

void Face::faceGestureToCSV(std::vector<Face> gesture, std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ofstream ofs(path, std::ofstream::out);
	if (!ofs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}

	// Write header
	ofs << "sep=, " << std::endl;
	ofs << "InInfraredSpace, Bounding Box, ";
	for (int i = 0; i < FacePointType::FacePointType_Count; ++i) ofs << "FacePoint " << i << ", ";
	ofs << "FaceRotation, ";
	for (int i = 0; i < FaceProperty::FaceProperty_Count; ++i) ofs << "FaceProperty " << i << ", ";

	ofs << std::endl;

	// Add the elements for each gesture
	for (int i = 0; i < gesture.size(); ++i) {
		if (gesture[i].isEmpty) gesture[i].setToZero();
		ofs << gesture[i];
	}
	ofs.close();
}
std::vector<Face> Face::faceGestureFromCSV(std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ifstream ifs(path, std::ifstream::in);
	if (!ifs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::vector<Face> gesture;
	std::string line;
	std::getline(ifs, line); // Read header line
	while (line.find("sep") != line.npos)
		std::getline(ifs, line);

	std::string delim;
	while (std::getline(ifs, line)) {
		Face frame;
		std::istringstream iss(line);

		// Space
		iss >> frame.inInfraredSpace >> delim;

		// Bounding Box
		RectI bbox;
		iss >> bbox.Top >> bbox.Bottom >> bbox.Left >> bbox.Right >> delim;
		frame.faceBBox = bbox;

		// Face points
		for (int i = 0; i < FacePointType::FacePointType_Count; ++i) {
			iss >> frame.facePoints[i].X >> frame.facePoints[i].Y >> delim;
		}

		// Face Rotation
		iss >> frame.faceRotation.x >> frame.faceRotation.y >> frame.faceRotation.z >> frame.faceRotation.w >> delim;

		// Face Properties
		int aux;
		for (int i = 0; i < FaceProperty::FaceProperty_Count; ++i) {
			iss >> aux >> delim;
			frame.faceProperties[i] = static_cast<DetectionResult>(aux);
		}
		iss >> delim;

		// Not empty
		if (!frame.checkAllZero()) frame.isEmpty = false;
		else frame.isEmpty = true;

		// Store this frame
		gesture.push_back(frame);
	}


	ifs.close();
	return gesture;
}


void Face::setToZero() {
	// Bounding Box
	faceBBox.Top = faceBBox.Bottom = faceBBox.Left = faceBBox.Right = 0;

	// Face points
	for (int i = 0; i < FacePointType::FacePointType_Count; ++i) {
		facePoints[i].X = facePoints[i].Y = 0;
	}

	// Face Rotation
	faceRotation.x = faceRotation.y = faceRotation.z = faceRotation.w = 0;

	// Face Properties
	for (int i = 0; i < FaceProperty::FaceProperty_Count; ++i) {
		faceProperties[i] = DetectionResult::DetectionResult_No;
	}
}

std::vector<float> Face::computeFrameFeatures(int dbf, int numoff, int lastusedframe, const std::vector<float>& orientation_pyr, const std::vector<float>& oldorient_pyr) {
	std::vector<float> feats;
	if (lastusedframe%numoff == 0){ // Every numoff frames

		double difpitch = orientation_pyr[0] - oldorient_pyr[0]; //pitch - oldpitch;
		double difyaw = orientation_pyr[1] - oldorient_pyr[1]; //yaw - oldyaw;
		double difroll = orientation_pyr[2] - oldorient_pyr[2]; //roll - oldroll;

		if (abs(difroll) >= dbf) difroll = (0 < difroll) - (difroll < 0); // It is actually the same as difroll = Utils::sgn(difroll);
		else difroll = 0;
		if (abs(difyaw) >= dbf) difyaw = (0 < difyaw) - (difyaw < 0);
		else difyaw = 0;
		if (abs(difpitch) >= dbf) difpitch = (0 < difpitch) - (difpitch < 0);
		else difpitch = 0;

		feats = { (float)difpitch, (float)difyaw, (float)difroll };
	}
	return feats;
}

std::vector<std::vector<float>> Face::getFeatures(int dbf, int numoff, std::vector<Face>& seq) { // Distance between two used frames to be considered as diferences, number of frames to wait before using another frame (we pick a frame every numoff frames)
	int lastFrame = 0;
	std::vector<std::vector<float>> feat;
	//double oldpitch = 0, oldyaw = 0, oldroll = 0;
	std::vector<float> oldorient(3, 0);
	for (int i = 0; i < seq.size(); ++i) {
		double pitch, yaw, roll;
		Utils::ExtractFaceRotationInDegrees(&seq[i].getFaceRotation(), &pitch, &yaw, &roll);
		std::vector<float> orientation = { (float)pitch, (float)yaw, (float)roll };
		std::vector<float> frame_feat = computeFrameFeatures(dbf, numoff, ++lastFrame, orientation, oldorient);
		if (frame_feat.size() > 0) {
			feat.push_back(frame_feat);
			oldorient = orientation;
		}

		/*if (++lastFrame%numoff == 0){ // Every numoff frames
			double pitch, yaw, roll;
			Utils::ExtractFaceRotationInDegrees(&seq[i].getFaceRotation(), &pitch, &yaw, &roll);

			double difroll = roll - oldroll;
			double difyaw = yaw - oldyaw;
			double difpitch = pitch - oldpitch;

			if (abs(difroll) >= dbf) difroll = (0 < difroll) - (difroll < 0);
			else difroll = 0;
			if (abs(difyaw) >= dbf) difyaw = (0 < difyaw) - (difyaw < 0);
			else difyaw = 0;
			if (abs(difpitch) >= dbf) difpitch = (0 < difpitch) - (difpitch < 0);
			else difpitch = 0;

			feat.push_back({ (float)difpitch, (float)difyaw, (float)difroll });

			oldyaw = yaw; oldroll = roll; oldpitch = pitch;

			//lastFrame = 0;
		}*/
	}
	return feat;
}

void Face::gestureFeaturesToCSV(std::vector<Face>& gesture, int dbf, int numoff, std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ofstream ofs(path, std::ofstream::out);
	if (!ofs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}

	std::vector<std::vector<float>> features = Face::getFeatures(dbf, numoff, gesture);
	// Write header
	ofs << "sep=, " << std::endl;
	for (int j = 0; j < features[0].size(); ++j) ofs << "Feature" << j << ((j + 1 == features[0].size()) ? "" : ", ");
	ofs << std::endl;
	ofs << std::setprecision(std::numeric_limits<float>::max_digits10); // Set max precision
	for (int i = 0; i < features.size(); ++i) {
		for (int j = 0; j < features[i].size(); ++j) ofs << features[i][j] << ((j + 1 == features[i].size()) ? "" : ", ");
		ofs << std::endl;
	}
	ofs.close();
}

std::vector<std::vector<float>> Face::gestureFeaturesFromCSV(std::string path) {
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
		while (iss) {
			iss >> aux >> delim; // First feature
			assert(delim == ","); // Just to avoid errors while programming...
			feat.push_back(aux); // Second feature
		}
		gestureFeatures.push_back(feat);
	}
	ifs.close();
	return gestureFeatures;
}