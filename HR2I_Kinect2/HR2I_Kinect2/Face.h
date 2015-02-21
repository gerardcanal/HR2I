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


class Face
{
public:
	Face(RectI box, PointF points[], Vector4 rotation, DetectionResult properties[]);
	Face();
	~Face();

	// Setters
	void setBoundingBox(RectI bbox);
	void setFacePoints(PointF facePoints[]);
	void setFaceRotation(Vector4 faceRotation);
	void setFaceProperties(DetectionResult dres[]);

	// Getters
	RectI getBoundingBox();
	PointF* getFacePoints();
	Vector4 getFaceRotation();
	DetectionResult* getFaceProperties();
	bool getIsEmpty();

	// Methods
	static void faceGestureToCSV(std::vector<Face> gesture, std::string path);
	static std::vector<Face> faceGestureFromCSV(std::string path);


	friend std::ostream& operator<<(std::ostream& os, const Face& f);

	//void getFaceFeatures(); TODO

private:
	RectI faceBBox; // Face bounding box
	PointF facePoints[FacePointType::FacePointType_Count];
	Vector4 faceRotation;
	DetectionResult faceProperties[FaceProperty::FaceProperty_Count];

	bool isEmpty;
};

