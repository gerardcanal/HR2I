#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include "SlidingMatrix.h"
#include <queue>
#include "Utils.h"
#include "Skeleton.h"
#include <omp.h>

#define WAIT_FRAME_SLEEP_MS 10

static enum Gesture {
	/// Add here the Dynamic Gestures
	SALUTE,
	N_DYNAMIC_GESTURES = (SALUTE + 1), // SHOULD BE <<LAST DYNAMIC GESTURE>> + 1

	/// Add here the Static Gestures
	POINT_AT = N_DYNAMIC_GESTURES, // FIRST STATIC GESTURE! DO NOT CHANGE!! To make the gesture sequence continuous, it's equal to the last dynamic gesture + 1
	//NEW_STATIC_GEST, // Example of ne gesture, below N_STATIC should be updated
	N_STATIC_GESTURES = (POINT_AT - N_DYNAMIC_GESTURES + 1 ),  // SHOULD BE <<LAST STATIC GESTURE>> - N_DYNAMIC_GESTURES +1

	/// Total number of Gestures
	N_GESTURES = (N_DYNAMIC_GESTURES + N_STATIC_GESTURES)
};

struct GroundTruth {
	int firstFrame;
	int lastFrame;
	Gesture type;
	GroundTruth(int first, int last, Gesture g) : firstFrame(first), lastFrame(last), type(g) {}
};

struct GRParameters {
	float ALPHA[N_DYNAMIC_GESTURES]; // L1 distance threshold
	float pointAtTh[3]; // SIGMA. [0] = hand-hip distance, [1] = elbow angle and [3] consecutive frames -> test values 0.25, 2.8, 25
	float gestTh[N_DYNAMIC_GESTURES]; // MU - threshold per each gesture type for the DTW
	float ovlps[N_DYNAMIC_GESTURES]; // Overlap per gesture
	float bestOvlp; // Best overlap
};

/// Class to handle Gesture Recognition Algorithms
class GestureRecognition
{
public:
	GestureRecognition();
	~GestureRecognition();

	Gesture RecognizeGesture(std::vector<std::vector<std::vector<float>>> models, GRParameters params);
	//void addFrame(const std::vector<float>& frame);
	//void addFrame(const std::vector<float>& feat, const std::vector<float>& extendedFeat);
	void addFrame(const std::vector<float>& Dynamic_feat, const std::vector<float>& Static_Feat);
	//void addFrames(std::vector<std::vector<float>> framelist);
	GRParameters trainThresholds(std::vector<std::vector<std::vector<float>>> models, std::vector<std::vector<std::vector<float>>> Inputsequences,
						         std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, std::vector<float> alphas,
								 std::vector<std::vector<float>> gestTh, bool verbose);

	float LOOCV(const std::vector<std::vector<std::vector<float>>>& models, const std::vector<std::vector<std::vector<float>>>& Inputsequences,
				std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, const std::vector<float>& alphas,
				const std::vector<std::vector<float>>& gestTh, bool verbose);

	//static std::vector<std::vector<float>> addThirdFeature(std::vector<std::vector<float>> model);
	static void writeParameters(GRParameters params, std::string path);
	static GRParameters readParameters(std::string path);
	static std::vector<GroundTruth> readGrountTruth(std::string path);

	std::vector<std::vector<float>> conventionalDTW(const std::vector<std::vector<float>>& model, std::vector<std::vector<float>> input, float ALPHA);
	std::deque<int> getWPath(const std::vector<std::vector<float>> &M, int t);

private:
	float RealTimeDTW(int gestureId, const std::vector<std::vector<float>>& model, int nInputframes, float ALPHA, float MU);
	std::deque<int> getWPath(const SlidingMatrix<float> &M, int t);
	float staticGetureRecognition(int gestureId, float pointAtTh[3]);

	std::vector<float> getNextFrame(int gestId);
	void resetCurrentFrames();
	void clearFrames();

	float getSequencesOverlap(int gestureId, const std::vector<std::vector<float>>& model, const std::vector<std::vector<std::vector<float>>>& sequences,
							  const std::vector<std::vector<std::set<int>>>& gt, float ALPHA, float MU);
	float getSequenceOverlap(const std::vector<std::vector<float>>& model, const std::vector<std::vector<float>>& sequence,
							 const std::set<int>& gt, float ALPHA, float MU);

	// Fields
	std::vector<std::queue<std::vector<float>>> inputFrames; // One frame per gesture
	std::vector<int> currentFrames; // Current frame id of each gesture
	omp_lock_t omp_lock; // Lock to avoid race conditions...
	
	bool gestureFound;
	static const float INF;
};

