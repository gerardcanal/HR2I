#include "stdafx.h"
// Std
#include <thread>
#include <mutex>
#include <vector>

// Project inclusions
#include "Kinect2Utils.h"
#include "GestureRecognition.h"
#include "BodyRGBViewer.h"
#include "hr2i_thesis/GestureRecognitionResult.h"

using namespace std;

class HR2I_Kinect2 {
public:
	HR2I_Kinect2();
	~HR2I_Kinect2();
	hr2i_thesis::GestureRecognitionResult recognizeGestures(const string& GRParams_path, const vector<vector<vector<float>>>& models, Kinect2Utils& k2u, BodyRGBViewer& view);
	vector<vector<vector<float>>> readDynamicModels(string gestPath = "..\\..\\GestureRecorder\\GestureRecorder\\gestures\\");

private:
	// Vars
	mutex gr_mtx;
	bool get_data;
	deque<Skeleton> inputFrames;

	// Methods
	void dataGetter(Kinect2Utils* k2u, GestureRecognition* gr, BodyRGBViewer* view, GRParameters params);

};