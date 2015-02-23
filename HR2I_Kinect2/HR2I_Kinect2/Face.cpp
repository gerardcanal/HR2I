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

	// Face Propertoes
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
	for (int i = 0; i < gesture.size(); ++i) ofs << gesture[i];
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
			iss >> aux;
			frame.faceProperties[i] = static_cast<DetectionResult>(aux);
		}
		iss >> delim;

		// Not empty
		if (!frame.checkAllZero()) frame.isEmpty = false;

		// Store this frame
		gesture.push_back(frame);
	}


	ifs.close();
	return gesture;
}
