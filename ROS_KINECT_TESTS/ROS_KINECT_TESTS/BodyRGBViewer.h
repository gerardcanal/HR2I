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
	int Run(bool sRGB, bool sSKel, bool autoSkel);
	std::thread RunThreaded(bool sRGB, bool sSKel, bool autoSkel);
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	void setK2U(Kinect2Utils*);
	bool isRunning();
	void setBodyFrameToDraw(IBodyFrame*);
	void closeWindow();
	void playGesture(std::vector<Skeleton> gesture, bool enableContols, bool closeAfterPlaying);
	void nextGestureFrame();
	void previousGestureFrame();

private:
	bool					showRGB;
	bool					showSkeleton;
	bool					running;
	static bool				createdWindow;
	int						playerControl; // -1: left, 0 stop, 1 right

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
	ID2D1Bitmap*             m_pBitmap;

	bool UpdateSkeleton(bool);
	void UpdateRGB();
	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);
	HRESULT RenderImage(BYTE* pImage, unsigned long cbImage);

	// Thread safety
	std::mutex mtx;
	IBody* ppBodiesToDraw[BODY_COUNT]; //Bodies to to draw from the outside
	void paintJointsAndHands(Joint joints[], HandState rightHandState, HandState leftHandState, int width, int height);

};

