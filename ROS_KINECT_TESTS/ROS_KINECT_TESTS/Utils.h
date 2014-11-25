#pragma once
#include "Kinect.h"
#define _USE_MATH_DEFINES // To have PI and other constants
#include <math.h>
#include <iostream>
#include <vector>
#include <set>

class Utils
{
public:
	Utils();
	~Utils();
	static float euclideanDistance(Joint a, Joint b);
	static float euclideanDistance(std::vector<float> a, std::vector<float> b);
	static float L1Distance(std::vector<float> a, std::vector<float> b);
	static float L1Distance(std::vector<float> a, std::vector<float> b, float alpha);
	static float getAngleBetween(Joint a, Joint mid, Joint c, bool rad);
	static float magnitude(float vec[3]);
	static float magnitude(std::vector<float> vec);
	static void printPercentage(int cur, int total);
	static float overlap(const std::set<int>& detections, const std::set<int>& gt);
	static std::vector<float> linePlaneIntersection(std::vector<float> l0, std::vector<float> l, std::vector<float> p0, std::vector<float> n);
	static float dotProduct(std::vector<float> a, std::vector<float> b);
	static std::vector<float> subtract(std::vector<float> a, std::vector<float> b);
	static bool sameDirection(std::vector<float> a, std::vector<float> b, float threshold);

	static const double INF;
};


