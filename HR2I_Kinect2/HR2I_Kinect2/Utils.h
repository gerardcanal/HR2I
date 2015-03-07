// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#pragma once
#include "Kinect.h"
#define _USE_MATH_DEFINES // To have PI and other constants
#include <math.h>
#include <iostream>
#include <vector>
#include <set>
#include <assert.h>

class Utils
{
public:
	Utils();
	~Utils();
	static float euclideanDistance(const Joint& a, const Joint& b);
	static float euclideanDistance(const std::vector<float>& a, const std::vector<float>& b);
	static float L1Distance(const std::vector<float>& a, const std::vector<float>& b);
	static float L1Distance(const std::vector<float>& a, const std::vector<float>& b, float alpha);
	template<typename T>
	static float HammingDistance(const std::vector<T>& a, const std::vector<T>& b);
	
	static float getAngleBetween(const Joint& a, const Joint& mid, const Joint& c, bool rad);
	static float magnitude(float vec[3]);
	static float magnitude(const std::vector<float>& vec);
	static void printPercentage(int cur, int total);
	static float overlap(const std::set<int>& detections, const std::set<int>& gt);
	static float overlap(int firstgt, int lastgt, int firstdet, int lastdet);
	static std::vector<float> linePlaneIntersection(const std::vector<float>& l0, const std::vector<float>& l, const std::vector<float>& p0, const std::vector<float>& n);
	static float dotProduct(const std::vector<float>& a, const std::vector<float>& b);
	static std::vector<float> subtract(const std::vector<float>& a, const std::vector<float>& b);
	static bool sameDirection(const std::vector<float>& a, const std::vector<float>& b, float threshold);

	static void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, double* pPitch, double* pYaw, double* pRoll);

	static float F1measure(int TP, int FP, int FN);
	static std::vector<std::pair<int, int>> detectionsUnion(const std::vector<std::pair<int, int>>& detections);

	template <typename T> 
	static int sgn(T val); 

	static const double INF;
};


template<typename T>
int Utils::sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template<typename T>
float Utils::HammingDistance(const std::vector<T>& a, const std::vector<T>& b) {
	assert(a.size() == b.size());
	float hd = 0;
	for (int i = 0; i < a.size(); ++i) {
		hd += a[i] != b[i];
	}
	return hd;
}
