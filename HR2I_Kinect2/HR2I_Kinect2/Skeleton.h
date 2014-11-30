#pragma once
#include "Kinect.h"
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include "Utils.h"

/// Class to manage a Skeleton object, it's mainly a wrapper class
class Skeleton
{
public:
	Skeleton(HandState left, HandState right,
						TrackingConfidence leftTC, TrackingConfidence rightTC,
						BOOLEAN isTracked, UINT64 id,
						JointOrientation jOrient[], Joint joints[]);
	//Skeleton(const Skeleton &s);
	Skeleton();
	~Skeleton();

	//Setters
	void setHandState(HandState left, HandState right);
	void setHandTrackingConfidence(TrackingConfidence left, TrackingConfidence right);
	void setTracking(BOOLEAN isTracked, UINT64 id);
	void setJointsInfo(JointOrientation jOrient[], Joint joints[]);

	//Getters
	std::array<HandState, 2> getHandState();
	std::array<TrackingConfidence, 2> getHandTrackingConfidence();
	bool getIsTracked();
	UINT64 getTrackingID();
	JointOrientation* getJointOrientations();
	Joint* getJoints();
	CameraSpacePoint getJointPosition(JointType j);
	Joint getJoint(JointType j);

	static void gestureToCSV(std::vector<Skeleton> gesture, std::string path);
	static std::vector<Skeleton> gestureFromCSV(std::string path);
	static void gestureFeaturesToCSV(std::vector<Skeleton> gesture, std::string path);
	static std::vector<std::vector<float>> gestureFeaturesFromCSV(std::string path);
	
	friend std::ostream& operator<<(std::ostream& os, const Skeleton& sk);
	bool operator==(const Skeleton& b);

	std::vector<float> getDynamicGestureRecognitionFeatures(bool rightBody);
	std::vector<float> getStaticGestureRecognitionFeatures(bool rightBody, bool addHandPose);
	//std::vector<float> getExtendedGestureRecognitionFeatures(bool rightBody, float th);
	//void addExtendedGRFeature(std::vector<float>& feature, bool rightBody, float th);
	//float getElbowSpineDistance(bool rightBody);

private:
	// Info about the body
	TrackingConfidence	leftTc, rightTc;
	HandState			leftHs, rightHs;
	bool				isTracked;
	UINT64				trackingId;
	JointOrientation	jointOrientations[JointType_Count];
	Joint				joints[JointType_Count];
	static const int	nJoints = JointType_Count; // Number of Joints
};

