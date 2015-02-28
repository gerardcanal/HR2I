// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#pragma once
#include "Kinect.Face.h"
#include <vector>
#include <iostream>
#include <limits>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include "Utils.h"


class Face
{
public:
	Face(RectI box, PointF points[], Vector4 rotation, DetectionResult properties[], bool inInfraredSpace);
	Face();
	~Face();

	// Setters
	void setBoundingBox(RectI bbox);
	void setFacePoints(PointF facePoints[], bool inInfraRedSpace);
	void setFaceRotation(Vector4 faceRotation);
	void setFaceProperties(DetectionResult dres[]);

	// Getters
	RectI getBoundingBox();
	PointF* getFacePoints();
	Vector4 getFaceRotation();
	DetectionResult* getFaceProperties();
	bool getIsEmpty();
	bool getIsInInfraredSpace();

	// Methods
	static void faceGestureToCSV(std::vector<Face> gesture, std::string path);
	static std::vector<Face> faceGestureFromCSV(std::string path);

	static std::vector<std::vector<float>> getFeatures(int difbetweenframes, int framesbeforesampling, std::vector<Face>& seq);
	static void gestureFeaturesToCSV(std::vector<Face>& gesture, int difbetweenframes, int framesbeforesampling, std::string path);
	static std::vector<std::vector<float>> gestureFeaturesFromCSV(std::string path);
	static std::vector<float> computeFrameFeatures(int difbetweenframes, int framesbeforesampling, int lastusedframe, const std::vector<float>& orientation_pyr, const std::vector<float>& oldorient_pyr);

	friend std::ostream& operator<<(std::ostream& os, const Face& f);

private:
	RectI faceBBox; // Face bounding box
	PointF facePoints[FacePointType::FacePointType_Count];
	Vector4 faceRotation;
	DetectionResult faceProperties[FaceProperty::FaceProperty_Count];

	bool inInfraredSpace; // Space in which the points are
	bool isEmpty;

	bool checkAllZero();
	void setToZero();
};
