// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
// Based on the BodyViewer and RGBViewer examples from the Kinect v2 SDK
#include "stdafx.h"
#include "BodyRGBViewer.h"
#include "Kinect.h"
#include <strsafe.h>

// Display consts - C++ Standard doesn't allow to initialize them inclass as they're floats.
static const float c_JointThickness = 4.0f;//3.0f;
static const float c_TrackedBoneThickness = 8.0f;//6.0f;
static const float c_InferredBoneThickness = 2.0f;//1.0f;
static const float c_HandSize = 40.0f;//30.0f;

static const int        cDepthWidthSkel = 640;// 512;
static const int        cDepthHeightSkel = 360;// 424;

static const int        cDepthWidth = 512;// 512;
static const int        cDepthHeight = 424;// 424;

static const int        cColorWidth = 1920;
static const int        cColorHeight = 1080;
bool BodyRGBViewer::createdWindow = false;


// Methods definitions
BodyRGBViewer::BodyRGBViewer() : 
	m_pD2DFactory(NULL),
	m_hWnd(NULL), 
	K2U(NULL),
	m_pCoordinateMapper(NULL),
	m_pRenderTarget(NULL),
	m_pBrushJointTracked(NULL),
	m_pBrushJointInferred(NULL),
	m_pBrushBoneTracked(NULL),
	m_pBrushBoneInferred(NULL),
	m_pBrushHandClosed(NULL),
	m_pBrushHandOpen(NULL),
	m_nNextStatusTime(0),
	m_pBrushHandLasso(NULL),
	m_pColorRGBX(NULL)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	for (int i = 0; i < BODY_COUNT; ++i) ppBodiesToDraw[i] = NULL;
	playingGesture = false;
}

BodyRGBViewer::BodyRGBViewer(Kinect2Utils* k2u_) : 
	m_pD2DFactory(NULL),
	m_hWnd(NULL),
	K2U(NULL),
	m_pCoordinateMapper(NULL),
	m_nNextStatusTime(0),
	m_pRenderTarget(NULL),
	m_pBrushJointTracked(NULL),
	m_pBrushJointInferred(NULL),
	m_pBrushBoneTracked(NULL),
	m_pBrushBoneInferred(NULL),
	m_pBrushHandClosed(NULL),
	m_pBrushHandOpen(NULL),
	m_pBrushHandLasso(NULL),
	m_pColorRGBX(NULL)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	K2U = k2u_;
	for (int i = 0; i < BODY_COUNT; ++i) ppBodiesToDraw[i] = NULL;
	playingGesture = false;
}

BodyRGBViewer::~BodyRGBViewer()
{
	DiscardDirect2DResources();


	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);
}

void BodyRGBViewer::setK2U(Kinect2Utils* k2u_) {
	K2U = k2u_;
}

void BodyRGBViewer::changeMode(int sRGB_Depth, bool showSkel) {
	if (playingGesture) return;
	mtx_changeMode.lock();
	showRGB_Depth = sRGB_Depth;
	showSkeleton = showSkel;
	DiscardDirect2DResources(); // So they are created again
	mtx_changeMode.unlock();
}

std::thread BodyRGBViewer::RunThreaded(int sRGB_Depth, bool sSkel, bool autoSkel) {
	running = true;
	std::thread t1(&BodyRGBViewer::Run, this, sRGB_Depth, sSkel, autoSkel);
	return t1;
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="sRGB">whether to display the RGB image or not.</param>
/// <param name="sSkel">whether to display the Skeletons or not.</param>
//<param name="autoSkel">whether to pick automatically the Skeletons from the kinect or if the user will call to the updateSkeleton from outside</param>
int BodyRGBViewer::Run(int sRGB_Depth, bool sSkel, bool autoSkel)
{
	running = true;
	showRGB_Depth = sRGB_Depth;
	showSkeleton = sSkel;
	HINSTANCE hInstance = GetModuleHandle(NULL);


	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"BodyRGBViewerAppDlgWndClass";

	if (!createdWindow) {
		if (!RegisterClassW(&wc))
		{
			DWORD err = GetLastError();
			return 0;
		}
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP),
		NULL,
		(DLGPROC)BodyRGBViewer::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, SW_SHOWDEFAULT);
	SetForegroundWindow(hWndApp);
	createdWindow = true;
	
	if (K2U) {
		HRESULT hr = K2U->getCoordinateMapper(m_pCoordinateMapper);
	}

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		bool paintedSkeleton = true;
		mtx_changeMode.lock();
		if (sSkel) paintedSkeleton = UpdateSkeleton(autoSkel);
		if (((showRGB_Depth > 0) && !sSkel) || ((showRGB_Depth > 0) && !paintedSkeleton)) {
			showSkeleton = false; //To force the updateRGB paint the skeleton.
			if (showRGB_Depth == 1) UpdateRGB(); // If only RGB has to be painted, paint it. Alternatevily, if the skeleton was not painted and the RGB should, paint it anyway.
			else if (showRGB_Depth == 2) UpdateDepth();
			showSkeleton = sSkel; // Restore to normal status
		}
		mtx_changeMode.unlock();

		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	return static_cast<int>(msg.wParam);
}


/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK BodyRGBViewer::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	BodyRGBViewer* pThis = NULL;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<BodyRGBViewer*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<BodyRGBViewer*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK BodyRGBViewer::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
	{
		// Bind application window handle
		m_hWnd = hWnd;

		// Init Direct2D
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		// Create and initialize a new Direct2D image renderer
		// We'll use this to draw the data we receive from the Kinect to the screen
		m_pD2DFactory->AddRef();


	}
		break;

	// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		running = false;
		DestroyWindow(hWnd);
		break;

	case WM_DESTROY:
		running = false;
		// Quit the main message pump
		PostQuitMessage(0);
		break;

	}


	return FALSE;
}

void BodyRGBViewer::closeWindow() {
	PostMessage(m_hWnd, WM_CLOSE, 0, 0);
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT BodyRGBViewer::EnsureDirect2DResources()
{
	HRESULT hr = S_OK;

	if (m_pD2DFactory && !m_pRenderTarget)
	{
		RECT rc;
		GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);

		int width = rc.right - rc.left;
		int height = rc.bottom - rc.top;
		D2D1_SIZE_U size = D2D1::SizeU(width, height);

		if (showRGB_Depth == 1) size = D2D1::SizeU(cColorWidth, cColorHeight);
		else if (showRGB_Depth == 2) size = D2D1::SizeU(cDepthWidth, cDepthHeight);

		D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
		rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
		rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

		// Create a hWnd render target, in order to render to the window set in initialize
		hr = m_pD2DFactory->CreateHwndRenderTarget(
			rtProps,
			D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
			&m_pRenderTarget
			);

		if (FAILED(hr))
		{
			SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
			return hr;
		}

		if (showSkeleton) {
			// light green
			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
			m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
		}
		if (showRGB_Depth > 0) {
			// Create a bitmap that we can copy image data into and then render to the target
			hr = m_pRenderTarget->CreateBitmap(
				size,
				D2D1::BitmapProperties(D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE)),
				&m_pBitmap
				);

			if (FAILED(hr))
			{
				SetStatusMessage(L"Couldn't create Direct2D Bitmap!", 10000, true);
				SafeRelease(m_pRenderTarget);
				return hr;
			}
		}
	}

	return hr;
}

/// <summary>
/// Dispose Direct2d resources
/// </summary>
void BodyRGBViewer::DiscardDirect2DResources()
{
	if (showSkeleton) {
		SafeRelease(m_pRenderTarget);

		SafeRelease(m_pBrushJointTracked);
		SafeRelease(m_pBrushJointInferred);
		SafeRelease(m_pBrushBoneTracked);
		SafeRelease(m_pBrushBoneInferred);

		SafeRelease(m_pBrushHandClosed);
		SafeRelease(m_pBrushHandOpen);
		SafeRelease(m_pBrushHandLasso);
	}
	if (showRGB_Depth > 0) {
		SafeRelease(m_pRenderTarget);
		SafeRelease(m_pBitmap);
	}
}


/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool BodyRGBViewer::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
	DWORD now = GetTickCount();

	if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
	{
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}

/// <summary>
/// Main processing function
/// </summary>
// <param name="pickBodyFrame">Whether it the bodyframe from the Kinect sensor</param>
//<return> Returns true if painted, false if no draw operation was done </return>
bool BodyRGBViewer::UpdateSkeleton(bool pickBodyFrame)
{
	if (!showSkeleton) return false; // To avoid strange behaviours
	
	mtx.lock(); // Begin to work with the bodyframe to draw
	if ((pickBodyFrame && !K2U) || (!pickBodyFrame && !ppBodiesToDraw)) {
		mtx.unlock();
		return false;
	}

	IBodyFrame* pBodyFrame = NULL;
	INT64 nTime = 0;

	if (pickBodyFrame) {
		pBodyFrame = NULL;

		HRESULT hr = K2U->openBodyFrameReader();
		if (SUCCEEDED(hr)) pBodyFrame = K2U->getLastBodyFrameFromDefault();
		if (!SUCCEEDED(hr) || !pBodyFrame) {
			mtx.unlock();
			return false;
		}

		hr = pBodyFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { NULL };

		if (SUCCEEDED(hr)) {
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr)) {
			ProcessAndPaintBody(nTime, BODY_COUNT, ppBodies, NULL);
		}

		for (int i = 0; i < _countof(ppBodies); ++i) {
			SafeRelease(ppBodies[i]);
		}
	}
	else {
		ProcessAndPaintBody(nTime, BODY_COUNT, ppBodiesToDraw, NULL);

		for (int i = 0; i < _countof(ppBodiesToDraw); ++i) {
			SafeRelease(ppBodiesToDraw[i]);
		}
	}

	if (pickBodyFrame) SafeRelease(pBodyFrame); // To avoid releasing the pointer if the app have to use it
	mtx.unlock();
	return true;
}


/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// <param name=body">body data in skeleton mode to draw in frame. ppBodies is ignored if it is not NULL</param>
/// </summary>
void BodyRGBViewer::ProcessAndPaintBody(INT64 nTime, int nBodyCount, IBody** ppBodies, Skeleton* body)
{
	if (m_hWnd)
	{
		HRESULT hr = EnsureDirect2DResources();

		if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
		{
			m_pRenderTarget->BeginDraw();
			if (showRGB_Depth == 0) m_pRenderTarget->Clear(); // To avoid flashes in RGB mode and non updating of the buffer in skeleton mode
			else if (showRGB_Depth == 1) UpdateRGB();
			else UpdateDepth(); //paint image

			RECT rct;
			GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
			int width = rct.right;
			int height = rct.bottom;
			if (showRGB_Depth == 1) {
				width = cColorWidth;
				height = cColorHeight;
			}
			else if (showRGB_Depth == 2) {
				width = cDepthWidth;
				height = cDepthHeight;
			}

			if (body) {
				std::array<HandState, 2> hs = body->getHandState();
				paintJointsAndHands(body->getJoints(), hs[1], hs[0], width, height);
			}
			else {
				for (int i = 0; i < nBodyCount; ++i)
				{
					IBody* pBody = ppBodies[i];
					if (pBody)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						if (SUCCEEDED(hr) && bTracked)
						{
							Joint joints[JointType_Count];
							D2D1_POINT_2F jointPoints[JointType_Count];
							HandState leftHandState = HandState_Unknown;
							HandState rightHandState = HandState_Unknown;

							pBody->get_HandLeftState(&leftHandState);
							pBody->get_HandRightState(&rightHandState);

							hr = pBody->GetJoints(_countof(joints), joints);
							if (SUCCEEDED(hr))
							{
								paintJointsAndHands(joints, rightHandState, leftHandState, width, height);
							}
						}
					}
				}
			}

			hr = m_pRenderTarget->EndDraw();

			// Device lost, need to recreate the render target
			// We'll dispose it now and retry drawing
			if (D2DERR_RECREATE_TARGET == hr)
			{
				hr = S_OK;
				DiscardDirect2DResources();
			}
		}
		if (showRGB_Depth == 0) {
			if (!m_nStartTime)
			{
				m_nStartTime = nTime;
			}

			double fps = 0.0;

			LARGE_INTEGER qpcNow = { 0 };
			if (m_fFreq)
			{
				if (QueryPerformanceCounter(&qpcNow))
				{
					if (m_nLastCounter)
					{
						m_nFramesSinceUpdate++;
						fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
					}
				}
			}

			WCHAR szStatusMessage[64];
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

			if (SetStatusMessage(szStatusMessage, 1000, false))
			{
				m_nLastCounter = qpcNow.QuadPart;
				m_nFramesSinceUpdate = 0;
			}
		}
	}
}


void BodyRGBViewer::paintJointsAndHands(Joint joints[], HandState rightHandState, HandState leftHandState, int width, int height) {
	D2D1_POINT_2F jointPoints[JointType_Count];
	for (int j = 0; j < _countof(jointPoints); ++j)
	{
		jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
	}

	DrawBody(joints, jointPoints);

	DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
	DrawHand(rightHandState, jointPoints[JointType_HandRight]);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F BodyRGBViewer::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	float screenPointX, screenPointY;
	if (showRGB_Depth != 1) { // For depth and none cases
		DepthSpacePoint depthPoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

		screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
		screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;
	}
	else {
		ColorSpacePoint colorPoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);

		screenPointX = static_cast<float>(colorPoint.X * width) / cColorWidth;
		screenPointY = static_cast<float>(colorPoint.Y * height) / cColorHeight;
	}

	return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void BodyRGBViewer::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
	// Draw the bones

	// Torso
	DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
	DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
	DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
	DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
	DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{
		D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
		}
	}
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void BodyRGBViewer::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
	}
	else
	{
		m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
	}
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void BodyRGBViewer::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
	D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

	switch (handState)
	{
	case HandState_Closed:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
		break;

	case HandState_Open:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
		break;

	case HandState_Lasso:
		m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
		break;
	}
}


/// <summary>
/// Main processing function
/// </summary>
void BodyRGBViewer::UpdateRGB()
{
	if (!K2U)
	{
		return;
	}

	IColorFrame* pColorFrame = NULL;
	HRESULT hr = K2U->openColorFrameReader();	
	if (SUCCEEDED(hr)) pColorFrame = K2U->getLastColorFrameFromDefault();
	if (!SUCCEEDED(hr) || !pColorFrame) return;


	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = NULL;

		hr = pColorFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr))
		{
			ProcessColor(nTime, pBuffer, nWidth, nHeight);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
}


/// <summary>
/// Main processing function
/// </summary>
void BodyRGBViewer::UpdateDepth()
{
	if (!K2U)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;
	HRESULT hr = K2U->openDepthFrameReader();
	if (SUCCEEDED(hr)) pDepthFrame = K2U->getLastDepthFrameFromDefault();
	if (!SUCCEEDED(hr) || !pDepthFrame) return;


	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		hr = pDepthFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{
			ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
}


/// <summary>
/// Handle new color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void BodyRGBViewer::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}

	// Make sure we've received valid data
	if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
	{
		// Draw the data with Direct2D
		RenderImage(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));

	}
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void BodyRGBViewer::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}

	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);
			BYTE intensity = depth < nMinDepth || depth > nMaxDepth ? 0 : (BYTE)(((float)depth / nMaxDepth) * 255.0f);

			pRGBX->rgbRed = intensity;
			pRGBX->rgbGreen = intensity;
			pRGBX->rgbBlue = intensity;

			++pRGBX;
			++pBuffer;
		}

		// Draw the data with Direct2D
		RenderImage(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
	}

}


/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
/// </summary>
/// <param name="pImage">image data in RGBX format</param>
/// <param name="cbImage">size of image data in bytes</param>
/// <returns>indicates success or failure</returns>
#define MIRROR_RGB // If defined RGB is mirrored
HRESULT BodyRGBViewer::RenderImage(BYTE* pImage, unsigned long cbImage)
{
	// incorrectly sized image data passed in
	int m_sourceStride= (showRGB_Depth == 1)? cColorWidth * sizeof(RGBQUAD) : cDepthWidth * sizeof(RGBQUAD);
	if ((showRGB_Depth == 1) && (cbImage < ((cColorHeight - 1) * m_sourceStride) + (cColorWidth * 4)))
	{
		return E_INVALIDARG;
	}
	else if ((showRGB_Depth == 2) && (cbImage < ((cDepthHeight - 1) * m_sourceStride) + (cDepthWidth * 4)))
	{
		return E_INVALIDARG;
	}

	// create the resources for this draw device
	// they will be recreated if previously lost
	HRESULT hr = EnsureDirect2DResources();

	if (FAILED(hr))
	{
		return hr;
	}

	// Copy the image that was passed in into the direct2d bitmap
	hr = m_pBitmap->CopyFromMemory(NULL, pImage, m_sourceStride);

	if (FAILED(hr))
	{
		return hr;
	}

	if (!showSkeleton) m_pRenderTarget->BeginDraw();
	
	m_pRenderTarget->Clear();
	
	// Draw the bitmap stretched to the size of the window
	m_pRenderTarget->DrawBitmap(m_pBitmap);

	if (!showSkeleton) hr = m_pRenderTarget->EndDraw();

	#ifdef MIRROR_RGB
	D2D1_SIZE_F sze = m_pRenderTarget->GetSize();
		D2D1_MATRIX_3X2_F m;
		m._11 = -1; m._12 = 0;
		m._21 = 0;  m._22 = 1;
		m._31 = sze.width; m._32 = 0;
		m_pRenderTarget->SetTransform(m);
	#endif

	// Device lost, need to recreate the render target
	// We'll dispose it now and retry drawing
	if (hr == D2DERR_RECREATE_TARGET)
	{
		hr = S_OK;
		DiscardDirect2DResources();
	}

	return hr;
}

bool BodyRGBViewer::isRunning() {
	return running;
}

void BodyRGBViewer::setBodyFrameToDraw(IBodyFrame* bf) {
	mtx.lock();
	INT64 relTime;
	HRESULT hr = bf->get_RelativeTime(&relTime);

	if (SUCCEEDED(hr)) {
		hr = bf->GetAndRefreshBodyData(_countof(ppBodiesToDraw), ppBodiesToDraw);
	}
	mtx.unlock();
}

void BodyRGBViewer::playGesture(std::vector<Skeleton> gesture, bool enableControls, bool closeAfterPlaying) {
	playingGesture = true;
	if (!running) { // Window is closed, create it for the first time...
		running = true;
		showRGB_Depth = 0;
		showSkeleton = true;

		// Prepare window
		HINSTANCE hInstance = GetModuleHandle(NULL);

		
		WNDCLASS  wc;

		// Dialog custom window class
		ZeroMemory(&wc, sizeof(wc));
		wc.style = CS_HREDRAW | CS_VREDRAW;
		wc.cbWndExtra = DLGWINDOWEXTRA;
		wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
		wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
		wc.lpfnWndProc = DefDlgProcW;
		wc.lpszClassName = L"BodyRGBViewerAppDlgWndClass";

		if (!createdWindow) {
			if (!RegisterClassW(&wc))
			{
				DWORD err = GetLastError();
				return;
			}
		}

		// Create main application window
		HWND hWndApp = CreateDialogParamW(
			NULL,
			MAKEINTRESOURCE(IDD_APP),
			NULL,
			(DLGPROC)BodyRGBViewer::MessageRouter,
			reinterpret_cast<LPARAM>(this));

		// Show window
		ShowWindow(hWndApp, SW_SHOWDEFAULT);
		SetForegroundWindow(hWndApp);
		createdWindow = true;

		if (K2U) {
			HRESULT hr = K2U->getCoordinateMapper(m_pCoordinateMapper);
		}
	}
	MSG       msg = { 0 };

	int i = 0;
	float FPS = 31.5;

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		// Paint frame
		if (i < gesture.size()) {
			int _time = time(NULL);
			ProcessAndPaintBody(_time, 1, NULL, &gesture[i]);
			
			if (enableControls) {
				// Print which frame is displayed:
				//WCHAR szText[128] = L"";
				//GetDlgItemText(m_hWnd, IDC_STATUS, szText, 128);
				WCHAR szText2[48] = L"";
				StringCchPrintf(szText2, _countof(szText2), L"    Frame = %d/%d", i + 1, gesture.size());
				//StringCchCat(szText, _countof(szText), szText2);
				SetStatusMessage(szText2, 10000, true);

				mtx.lock();
				if (playerControl > 0) ++i;
				else if (playerControl < 0 && i > 0) --i;
				playerControl = 0;
				mtx.unlock();
			}
			else ++i;
			int delta_t = time(NULL) - _time;
			int sleepTime = (1000 / FPS+1) - delta_t;
			if (sleepTime > 10) Sleep(sleepTime);
		}
		else {
			if (closeAfterPlaying) {
				playingGesture = false;
				closeWindow();
			}
			else break; // Window won't be closed
		}

		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_KEYDOWN) {
				switch (msg.wParam) {
				case VK_LEFT:
					previousGestureFrame();
					break;

				case VK_RIGHT:
					nextGestureFrame();
					break;
				}
			}
			// If a dialog message will be taken care of by the dialog proc
			if (m_hWnd && IsDialogMessageW(m_hWnd, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}
}

void BodyRGBViewer::nextGestureFrame() {
	mtx.lock();
	playerControl = 1;
	mtx.unlock();
}

void BodyRGBViewer::previousGestureFrame() {
	mtx.lock();
	playerControl = -1;
	mtx.unlock();
}


bool BodyRGBViewer::getWindowSize(int& horizontal, int& vertical)
{
	if (m_hWnd == NULL) return false;
	RECT window;
	GetWindowRect(m_hWnd, &window);
	horizontal = window.right;
	vertical = window.bottom;
	return true;
}

bool BodyRGBViewer::getViewPortSize(int& horizontal, int& vertical)
{
	if (m_hWnd == NULL) return false;
	RECT vport;
	GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &vport);
	horizontal = vport.right;
	vertical = vport.bottom;
	return true;
}