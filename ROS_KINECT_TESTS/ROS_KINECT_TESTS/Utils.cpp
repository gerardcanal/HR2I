#include "stdafx.h"
#include "Utils.h"
#include <assert.h>
#include <algorithm>
#include <iterator>

Utils::Utils()
{
}


Utils::~Utils()
{
}

float Utils::euclideanDistance(std::vector<float> a, std::vector<float> b) {
	assert(a.size() == b.size());
	float _sum = 0;
	for (int i = 0; i < a.size(); ++i) {
		float dif = a[i] - b[i];
		_sum += dif*dif;
	}
	return sqrt(_sum);
}

float Utils::L1Distance(std::vector<float> a, std::vector<float> b) {
	assert(a.size() == b.size());
	float _sum = 0;
	for (int i = 0; i < a.size(); ++i) _sum += abs(a[i] - b[i]);
	return _sum;
}

float Utils::L1Distance(std::vector<float> a, std::vector<float> b, float alpha) {
	assert(a.size() == b.size());
	float _sum = abs(a[0] - b[0])*alpha;
	for (int i = 1; i < a.size(); ++i) _sum += abs(a[i] - b[i]);

	return _sum;
}

float Utils::euclideanDistance(Joint a, Joint b) {
	float sq = pow(a.Position.X - b.Position.X, 2);
	sq += pow(a.Position.Y - b.Position.Y, 2);
	sq += pow(a.Position.Z - b.Position.Z, 2);
	return sqrt(sq);
}

/// Returns de angle between a->mid and mid->c. For example shoulder->elbow->hand
float Utils::getAngleBetween(Joint a, Joint mid, Joint c, bool rad) {
	// Vector mid->a
	float vec1[3] = { a.Position.X - mid.Position.X,
					  a.Position.Y - mid.Position.Y,
					  a.Position.Z - mid.Position.Z };
	// Vector mid->c
	float vec2[3] = { c.Position.X - mid.Position.X,
					  c.Position.Y - mid.Position.Y,
					  c.Position.Z - mid.Position.Z };

	float dotProd = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
	float cosAngle = dotProd / (magnitude(vec1) * magnitude(vec2));
	if (rad)
		return acos(cosAngle);
	return acos(cosAngle) * 180.0 / M_PI;
}

float Utils::magnitude(float vec[3]) {
	return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void Utils::printPercentage(int cur, int total) {
	float perc = 100*((float)cur / total);
	float prev = 100*((float)(cur - 1) / total);
	std::streamsize precis = std::cout.precision();

	std::cout.precision(2);
	std::cout << std::fixed << "\b\b\b\b\b"; // Remove number, decimals, point and %
	if (prev >= 10) std::cout << "\b"; // Remove desen digit
	if (prev >= 100) std::cout << "\b"; // Remove centen digit
	std::cout << perc << "%" << std::defaultfloat;
	std::cout.precision(precis);
}

/// Jaccard index
float Utils::overlap(const std::set<int>& detections, const std::set<int>& gt) {
	std::vector<int> _union, _intersection;
	std::set_union(detections.begin(), detections.end(),
		           gt.begin(), gt.end(),
				   std::inserter(_union, _union.begin()));

	std::set_intersection(detections.begin(), detections.end(),
					      gt.begin(), gt.end(),
						  std::inserter(_intersection, _intersection.begin()));
	if (_union.size() == 0) return 1.0; // Both are empty... so overlap is total
	return float(_intersection.size()) / _union.size();
}


float Utils::dotProduct(std::vector<float> a, std::vector<float> b) {
	assert(a.size() == b.size());
	float dp = 0;
	for (int i = 0; i < a.size(); ++i) 
		dp += a[i] * b[i];
	return dp;
}

std::vector<float> Utils::subtract(std::vector<float> a, std::vector<float> b) {
	assert(a.size() == b.size());
	std::vector<float> s(a.size());
	for (int i = 0; i < a.size(); ++i) s[i] = a[i] - b[i];
	return s;
}

/// Line-Plane intersection
/// l0 is a point of the line. l is the vector of the line
/// p0 is a point of the plane. n is the normal vector to the plane
std::vector<float> Utils::linePlaneIntersection(std::vector<float> l0, std::vector<float> l, std::vector<float> p0, std::vector<float> n) {
	float ln = Utils::dotProduct(l, n);
	if (ln == 0) return std::vector<float>();
	float ppn = Utils::dotProduct(Utils::subtract(p0, l0), n);
	float k = ppn / ln;

	std::vector<float> point(l0.size());
	for (int i = 0; i < point.size(); ++i) point[i] = l0[i] + k*l[i];
	return point;
}