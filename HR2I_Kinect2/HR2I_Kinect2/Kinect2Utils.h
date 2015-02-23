// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#pragma once
#include "Kinect.h"
#include <Kinect.Face.h>
#include "Face.h"
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
	HRESULT openFaceFrameReader(DWORD faceFrameFeatures = c_FaceFrameFeatures);
	HRESULT openMultiSourceFrameReader(DWORD types);
	IBodyFrame* getLastBodyFrameFromDefault();
	IColorFrame* getLastColorFrameFromDefault();
	IDepthFrame* getLastDepthFrameFromDefault();
	IFaceFrame* getLastFaceFrameFromDefault();
	IMultiSourceFrame* getLastMultiSourceFrameFromDefault();
	HRESULT getCoordinateMapper(ICoordinateMapper*&);

	IBodyFrame* getBodyFrame(IMultiSourceFrame* msf);
	IColorFrame* getColorFrame(IMultiSourceFrame* msf);
	IDepthFrame* getDepthFrame(IMultiSourceFrame* msf);
	
	static Skeleton IBodyToSkeleton(IBody* body);
	static std::vector<Skeleton> getSkeletonsFromBodyFrame(IBodyFrame* bodyFrame);
	static Skeleton getTrackedSkeleton(IBodyFrame* bodyFrame, UINT64 id, bool first);
	
	static Face faceFrameResultToFace(IFaceFrameResult* ffr, bool infraredSpace = true);
	static Face getFaceFromFaceFrame(IFaceFrame* fframe, bool infraredSpace = true);
	void setFaceTrackingId(UINT64 trid);


private:
	IKinectSensor* default_sensor; // Default sensor FIXME should be static
	IBodyFrameReader* bfReader;
	IColorFrameReader* cfReader;
	IDepthFrameReader* dfReader;
	IFaceFrameReader* ffReader;
	IMultiSourceFrameReader* msfReader;

	IFaceFrameSource* ffSource;

	static const DWORD c_FaceFrameFeatures =
		FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;
};

