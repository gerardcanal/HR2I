#include "stdafx.h"
#include "Kinect2Utils.h"
#include <iostream>

Kinect2Utils::Kinect2Utils()
{
	default_sensor = NULL;
	bfReader = NULL;
	cfReader = NULL;
	dfReader = NULL;
	msfReader = NULL;
}


Kinect2Utils::~Kinect2Utils()
{
	closeDefaultSensor();
	SafeRelease(bfReader);
	SafeRelease(cfReader);
	SafeRelease(dfReader);
}


/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>A pointer to the IKinectSensor or NULL if it could not be obtained.</returns>
HRESULT Kinect2Utils::initDefaultKinectSensor(bool open) {
	HRESULT hr = GetDefaultKinectSensor(&default_sensor);
	if (SUCCEEDED(hr) && open && default_sensor) {
		boolean isOpen;
		default_sensor->get_IsOpen(&isOpen);
		if (!isOpen) hr = default_sensor->Open();
	}
	return hr;
}

/// <summary>
/// Releases the sensor
/// </summary>
/// <param name="sensor">Pointer to IKinectSensor which is going to be released</param>
HRESULT Kinect2Utils::closeSensor(IKinectSensor* sensor) {
	HRESULT hr;
	if (sensor) {
		hr = sensor->Close();
		SafeRelease(sensor);
	}
	return hr;
}

HRESULT Kinect2Utils::closeDefaultSensor() {
	return closeSensor(default_sensor);
}

HRESULT Kinect2Utils::OpenDefaultSensor() {
	HRESULT hr;
	boolean isOpen;
	default_sensor->get_IsOpen(&isOpen);
	if (!isOpen) hr = default_sensor->Open();
	return hr;
}

HRESULT Kinect2Utils::openBodyFrameReader() {
	if (!bfReader) { //bfReader is NULL
		IBodyFrameSource* bfSource = NULL;
		HRESULT hr = default_sensor->get_BodyFrameSource(&bfSource);
		if (SUCCEEDED(hr))
			hr = bfSource->OpenReader(&bfReader);
		SafeRelease(bfSource); // We don't need the body frame source anymore.
		return hr;
	}
	return S_OK; //Already opened
}

HRESULT Kinect2Utils::openColorFrameReader() {
	if (!cfReader) { //bfReader is NULL
		IColorFrameSource* cfSource = NULL;
		HRESULT hr = default_sensor->get_ColorFrameSource(&cfSource);
		if (SUCCEEDED(hr))
			hr = cfSource->OpenReader(&cfReader);
		SafeRelease(cfSource); // We don't need the body frame source anymore.
		return hr;
	}
	return S_OK; //Already opened
}

HRESULT Kinect2Utils::openDepthFrameReader() {
	if (!dfReader) { //bfReader is NULL
		IDepthFrameSource* dfSource = NULL;
		HRESULT hr = default_sensor->get_DepthFrameSource(&dfSource);
		if (SUCCEEDED(hr))
			hr = dfSource->OpenReader(&dfReader);
		SafeRelease(dfSource); // We don't need the body frame source anymore.
		return hr;
	}
	return S_OK; //Already opened
}

/// Types = FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body
HRESULT Kinect2Utils::openMultiSourceFrameReader(DWORD types) {
	if (!msfReader) { //bfReader is NULL
		HRESULT hr = default_sensor->OpenMultiSourceFrameReader(types, &msfReader);
		return hr;
	}
	return S_OK; //Already opened
}

/// <summary> Returns the last body frame. The bfReaer MUST be called before this, or it will return NULL 
/// WARNING: if the bodyframe returned is not released (SafeRelease) it won't return more bodyframes!!</summary>
///
IBodyFrame* Kinect2Utils::getLastBodyFrameFromDefault() {
	if (!bfReader) return NULL;
	IBodyFrame* bodyFrame = NULL;
	HRESULT hr = bfReader->AcquireLatestFrame(&bodyFrame);
	if (!SUCCEEDED(hr)) return NULL;
	return bodyFrame;
}

/// <summary> Returns the last color frame. The cfReaer MUST be called before this, or it will return NULL </summary>
IColorFrame* Kinect2Utils::getLastColorFrameFromDefault() {
	if (!cfReader) return NULL;
	IColorFrame* colorFrame = NULL;
	HRESULT hr = cfReader->AcquireLatestFrame(&colorFrame);
	if (!SUCCEEDED(hr)) return NULL;
	return colorFrame;
}

/// <summary> Returns the last depth frame. The dfReaer MUST be called before this, or it will return NULL </summary>
IDepthFrame* Kinect2Utils::getLastDepthFrameFromDefault() {
	if (!dfReader) return NULL;
	IDepthFrame* depthFrame = NULL;
	HRESULT hr = dfReader->AcquireLatestFrame(&depthFrame);
	if (!SUCCEEDED(hr)) return NULL;
	return depthFrame;
}

/// <summary> Returns the last depth frame. The dfReaer MUST be called before this, or it will return NULL </summary>
IMultiSourceFrame* Kinect2Utils::getLastMultiSourceFrameFromDefault() {
	if (!msfReader) return NULL;
	IMultiSourceFrame* msFrame = NULL;
	HRESULT hr = msfReader->AcquireLatestFrame(&msFrame);
	if (!SUCCEEDED(hr)) return NULL;
	return msFrame;
}

HRESULT Kinect2Utils::getCoordinateMapper(ICoordinateMapper* &cmapper) {
	return default_sensor->get_CoordinateMapper(&cmapper);
}

std::vector<Skeleton> Kinect2Utils::getSkeletonsFromBodyFrame(IBodyFrame* bodyFrame) {
	const int N_BODIES = BODY_COUNT;
	INT64 nTime = 0;
	HRESULT hr;
	hr = bodyFrame->get_RelativeTime(&nTime);
	IBody* bodies[N_BODIES] = { NULL };

	std::vector<Skeleton> skeletons;

	if (SUCCEEDED(hr)) hr = bodyFrame->GetAndRefreshBodyData(N_BODIES, bodies);
	if (SUCCEEDED(hr)) {
		for (int i = 0; i < N_BODIES; ++i) {
			IBody* body = bodies[i];
			if (body) {
				BOOLEAN tracked = false;
				body->get_IsTracked(&tracked);
				if (tracked) {
					Skeleton auxSkel = Kinect2Utils::IBodyToSkeleton(body);
					skeletons.push_back(auxSkel);
				}
			}
			SafeRelease(body);
		}
	}

	return skeletons;
}

/// Return the skeleton identified by the id in the bodyFrame. If boolean first it returns the first one.
Skeleton Kinect2Utils::getTrackedSkeleton(IBodyFrame* bodyFrame, UINT64 id, bool first) {
	std::vector<Skeleton> skels = getSkeletonsFromBodyFrame(bodyFrame);
	if (skels.size() > 0) {
		if (first) return skels[0];
		for (int i = 0; i < skels.size(); ++i) {
			if (id == skels[i].getTrackingID()) return skels[i];
		}
	}
	return Skeleton();
}

Skeleton Kinect2Utils::IBodyToSkeleton(IBody* body) {
	//Hands
	TrackingConfidence	leftTc, rightTc;
	body->get_HandLeftConfidence(&leftTc);
	body->get_HandRightConfidence(&rightTc);

	HandState leftHs = HandState_Unknown, rightHs = HandState_Unknown;
	body->get_HandLeftState(&leftHs);
	body->get_HandRightState(&rightHs);

	//Tracking
	BOOLEAN	isTracked;
	UINT64 trackingId;
	body->get_TrackingId(&trackingId);
	body->get_IsTracked(&isTracked);

	//Joints
	JointOrientation jointOrientations[JointType_Count];
	Joint joints[JointType_Count];
	body->GetJointOrientations(_countof(jointOrientations), jointOrientations);
	body->GetJoints(_countof(joints), joints);

	return Skeleton(leftHs, rightHs, leftTc, rightTc, isTracked, trackingId, jointOrientations, joints);
}


IBodyFrame* Kinect2Utils::getBodyFrame(IMultiSourceFrame* msf) {
	IBodyFrameReference* bfRef = NULL;
	msf->get_BodyFrameReference(&bfRef);
	IBodyFrame* bf = NULL;
	HRESULT hr = bfRef->AcquireFrame(&bf);
	if (!SUCCEEDED(hr)) return NULL;
	return bf;
}

IColorFrame* Kinect2Utils::getColorFrame(IMultiSourceFrame* msf) {
	IColorFrameReference* cfRef = NULL;
	msf->get_ColorFrameReference(&cfRef);
	IColorFrame* cf = NULL;
	HRESULT hr = cfRef->AcquireFrame(&cf);
	if (!SUCCEEDED(hr)) return NULL;
	return cf;
}

IDepthFrame* Kinect2Utils::getDepthFrame(IMultiSourceFrame* msf) {
	IDepthFrameReference* dfRef = NULL;
	msf->get_DepthFrameReference(&dfRef);
	IDepthFrame* df = NULL;
	HRESULT hr = dfRef->AcquireFrame(&df);
	if (!SUCCEEDED(hr)) return NULL;
	return df;
}