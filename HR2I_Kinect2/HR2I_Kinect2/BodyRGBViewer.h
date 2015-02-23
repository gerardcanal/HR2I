// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
// Based on the BodyViewer and RGBViewer examples from the Kinect v2 SDK
#pragma once
#include "windows.h"
#include <thread>
#include <vector>
#include "Skeleton.h"
#include <mutex>
#include <array>
#include <time.h>

// Direct2D Header Files
#include <d2d1.h>

#include "Kinect2Utils.h"

//Resources like the app icon...
#include "resource.h"

#pragma comment (lib, "d2d1.lib")

/// Class to handle and show a windows with the RGB image from the Kinect with the Skeletons overlayed.
/// This class was made by using the code of the BodyBasics sample from the Kinect v2 SDK. 
class BodyRGBViewer
{
public:
	BodyRGBViewer();
	BodyRGBViewer(Kinect2Utils*);
	~BodyRGBViewer();
	int Run(int sRGB_Depth, bool sSKel, bool autoSkel);
	std::thread RunThreaded(int sRGB_Depth, bool sSKel, bool autoSkel);
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	void setK2U(Kinect2Utils*);
	bool isRunning();
	void setBodyFrameToDraw(IBodyFrame*);
	void setFaceFrameToDraw(Face& f);
	void closeWindow();
	void playGesture(std::vector<Skeleton> gesture, bool enableContols, bool closeAfterPlaying);
	void nextGestureFrame();
	void previousGestureFrame();
	void changeMode(int showRGB_Depth, bool showSkel);
	bool getWindowSize(int& horizontal, int& vertical);
	bool getViewPortSize(int& horizontal, int& vertical);

	enum sRGB {
		show_NONE,
		show_RGB,
		show_DEPTH
	};

private:
	int						showRGB_Depth; // 0 = None, 1 = RGB, 2 = DEPTH
	bool					showSkeleton;
	bool					running;
	static bool				createdWindow;
	int						playerControl; // -1: left, 0 stop, 1 right
	bool					playingGesture;

	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	DWORD                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	Kinect2Utils*			K2U;
	ICoordinateMapper*      m_pCoordinateMapper;


	// Direct2D
	ID2D1Factory*           m_pD2DFactory;

	// Body/hand drawing
	ID2D1HwndRenderTarget*  m_pRenderTarget;
	ID2D1SolidColorBrush*   m_pBrushJointTracked;
	ID2D1SolidColorBrush*   m_pBrushJointInferred;
	ID2D1SolidColorBrush*   m_pBrushBoneTracked;
	ID2D1SolidColorBrush*   m_pBrushBoneInferred;
	ID2D1SolidColorBrush*   m_pBrushHandClosed;
	ID2D1SolidColorBrush*   m_pBrushHandOpen;
	ID2D1SolidColorBrush*   m_pBrushHandLasso;
	ID2D1SolidColorBrush*   m_pFaceBrush;


	HRESULT EnsureDirect2DResources();
	void DiscardDirect2DResources();
	void ProcessAndPaintBody(INT64 nTime, int nBodyCount, IBody** ppBodies, Skeleton* body);
	bool SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);
	D2D1_POINT_2F BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
	void DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);
	void DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
	void DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

	// RGB
	RGBQUAD*                m_pColorRGBX;
	RGBQUAD*                m_pDepthRGBX;
	ID2D1Bitmap*            m_pBitmap;

	bool UpdateSkeleton(bool);
	void UpdateRGB();
	void UpdateDepth();
	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);
	HRESULT RenderImage(BYTE* pImage, unsigned long cbImage);

	// Thread safety
	std::mutex mtx;
	std::mutex mtx_changeMode;
	IBody* ppBodiesToDraw[BODY_COUNT]; //Bodies to to draw from the outside
	Face  faceToDraw;
	void paintJointsAndHands(Joint joints[], HandState rightHandState, HandState leftHandState, int width, int height);


	//Faces
	void BodyRGBViewer::DrawFaceFrameResults(const RectI* pFaceBox, const PointF* pFacePoints);
	bool BodyRGBViewer::ValidateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints);

};

