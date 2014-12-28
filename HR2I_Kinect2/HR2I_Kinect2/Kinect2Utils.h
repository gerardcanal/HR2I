#pragma once
#include "Kinect.h"
#include "Skeleton.h"
#include <vector>

/// Class to handle Kinect v2's things like connection and so.
// TODO. This class may need to be a singleton one
class Kinect2Utils
{
public:
	Kinect2Utils();
	~Kinect2Utils();
	HRESULT initDefaultKinectSensor(bool open);
	static HRESULT Kinect2Utils::closeSensor(IKinectSensor* sensor);
	HRESULT closeDefaultSensor();
	HRESULT OpenDefaultSensor();
	HRESULT openBodyFrameReader();
	HRESULT openColorFrameReader();
	HRESULT openDepthFrameReader();
	HRESULT openMultiSourceFrameReader(DWORD types);
	IBodyFrame* getLastBodyFrameFromDefault();
	IColorFrame* getLastColorFrameFromDefault();
	IDepthFrame* getLastDepthFrameFromDefault();
	IMultiSourceFrame* getLastMultiSourceFrameFromDefault();
	HRESULT getCoordinateMapper(ICoordinateMapper*&);

	IBodyFrame* getBodyFrame(IMultiSourceFrame* msf);
	IColorFrame* getColorFrame(IMultiSourceFrame* msf);
	IDepthFrame* getDepthFrame(IMultiSourceFrame* msf);
	
	static Skeleton IBodyToSkeleton(IBody* body);
	static std::vector<Skeleton> getSkeletonsFromBodyFrame(IBodyFrame* bodyFrame);
	static Skeleton getTrackedSkeleton(IBodyFrame* bodyFrame, UINT64 id, bool first);


private:
	IKinectSensor* default_sensor; // Default sensor FIXME should be static
	IBodyFrameReader* bfReader;
	IColorFrameReader* cfReader;
	IDepthFrameReader* dfReader;
	IMultiSourceFrameReader* msfReader;
};
