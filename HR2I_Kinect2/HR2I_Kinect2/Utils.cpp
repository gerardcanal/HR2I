// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#include "stdafx.h"
#include "Utils.h"
#include <algorithm>
#include <iterator>
#include <limits>

const double Utils::INF = std::numeric_limits<float>::infinity();

Utils::Utils()
{
}


Utils::~Utils()
{
}


// Distances
float Utils::euclideanDistance(const std::vector<float>& a, const std::vector<float>& b) {
	assert(a.size() == b.size());
	float _sum = 0;
	for (int i = 0; i < a.size(); ++i) {
		float dif = a[i] - b[i];
		_sum += dif*dif;
	}
	return sqrt(_sum);
}

float Utils::L1Distance(const std::vector<float>& a, const std::vector<float>& b) {
	assert(a.size() == b.size());
	float _sum = 0;
	for (int i = 0; i < a.size(); ++i) _sum += abs(a[i] - b[i]);
	return _sum;
}

float Utils::L1Distance(const std::vector<float>& a, const std::vector<float>& b, float alpha) {
	assert(a.size() == b.size());
	float _sum = abs(a[0] - b[0])*alpha;
	for (int i = 1; i < a.size(); ++i) _sum += abs(a[i] - b[i]);

	return _sum;
}

float Utils::euclideanDistance(const Joint& a, const Joint& b) {
	float sq = pow(a.Position.X - b.Position.X, 2);
	sq += pow(a.Position.Y - b.Position.Y, 2);
	sq += pow(a.Position.Z - b.Position.Z, 2);
	return sqrt(sq);
}


//Geometry
/// Returns de angle between a->mid and mid->c. For example shoulder->elbow->hand
float Utils::getAngleBetween(const Joint& a, const Joint& mid, const Joint& c, bool rad) {
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

float Utils::magnitude(const std::vector<float>& vec) {
	float mag = 0;
	for (int i = 0; i < vec.size(); ++i) mag += (vec[i] * vec[i]);
	return sqrt(mag);
}

void Utils::printPercentage(int cur, int total) {
	float perc = 100*((float)cur / total);
	float prev = 100*((float)(cur - 1) / total);
	float prev_2dec = roundf(prev * 100) / 100; // 2 decimals
	std::streamsize precis = std::cout.precision();

	std::cout.precision(2);
	std::cout << std::fixed << "\b\b\b\b\b"; // Remove number, decimals, point and %
	if (prev_2dec >= 10) std::cout << "\b"; // Remove desen digit
	if (prev_2dec >= 100) std::cout << "\b"; // Remove centen digit
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


float Utils::dotProduct(const std::vector<float>& a, const std::vector<float>& b) {
	assert(a.size() == b.size());
	float dp = 0;
	for (int i = 0; i < a.size(); ++i) 
		dp += a[i] * b[i];
	return dp;
}

std::vector<float> Utils::subtract(const std::vector<float>& a, const std::vector<float>& b) {
	assert(a.size() == b.size());
	std::vector<float> s(a.size());
	for (int i = 0; i < a.size(); ++i) s[i] = a[i] - b[i];
	return s;
}

/// Line-Plane intersection
/// l0 is a point of the line. l is the vector of the line
/// p0 is a point of the plane. n is the normal vector to the plane
std::vector<float> Utils::linePlaneIntersection(const std::vector<float>& l0, const std::vector<float>& l, const std::vector<float>& p0, const std::vector<float>& n) {
	float ln = Utils::dotProduct(l, n);
	if (ln == 0) return std::vector<float>();
	float ppn = Utils::dotProduct(Utils::subtract(p0, l0), n);
	float k = ppn / ln;

	std::vector<float> point(l0.size());
	for (int i = 0; i < point.size(); ++i) point[i] = l0[i] + k*l[i];
	return point;
}


bool Utils::sameDirection(const std::vector<float>& a, const std::vector<float>& b, float threshold) {
	float cosangle = Utils::dotProduct(a, b)/(Utils::magnitude(a)*Utils::magnitude(b));
	return acos(cosangle) <= threshold;
}


/// <summary>
/// (From the FaceBasics-D2D example)
/// Converts rotation quaternion to Euler angles 
/// And then maps them to a specified range of values to control the refresh rate
/// </summary>
/// <param name="pQuaternion">face rotation quaternion</param>
/// <param name="pPitch">rotation about the X-axis</param>
/// <param name="pYaw">rotation about the Y-axis</param>
/// <param name="pRoll">rotation about the Z-axis</param>
void Utils::ExtractFaceRotationInDegrees(const Vector4* pQuaternion, double* pPitch, double* pYaw, double* pRoll) {
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;


	// convert face rotation quaternion to Euler angles in degrees		
	double dPitch, dYaw, dRoll;
	dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
	dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;

	/**pPitch = dPitch;
	*pYaw = dYaw;
	*pRoll = dRoll;*/

	// clamp rotation values in degrees to a specified range of values to control the refresh rate
	static const double c_FaceRotationIncrementInDegrees = 5.0f; // From the face example. Not sure where does it come from, but means increment in angles from 5 to 5
	double increment = c_FaceRotationIncrementInDegrees;
	*pPitch = static_cast<int>(floor((dPitch + increment / 2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pYaw = static_cast<int>(floor((dYaw + increment / 2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pRoll = static_cast<int>(floor((dRoll + increment / 2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}

float Utils::F1measure(int TP, int FP, int FN) {
	return float(2 * TP) / float(2 * TP + FP + FN);
}