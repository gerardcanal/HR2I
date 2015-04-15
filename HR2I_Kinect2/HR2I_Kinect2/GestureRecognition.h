// Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include "SlidingMatrix.h"
#include <queue>
#include "Utils.h"
#include "Skeleton.h"
#include <omp.h>
#include<map>

#define WAIT_FRAME_SLEEP_MS 10

static enum Gesture {
	/// Add here the Dynamic Gestures
	WAVE,
	NOD,
	NEGATE,
	N_DYNAMIC_GESTURES = (NEGATE + 1), // SHOULD BE <<LAST DYNAMIC GESTURE>> + 1

	/// Add here the Static Gestures
	POINT_AT = N_DYNAMIC_GESTURES, // FIRST STATIC GESTURE! DO NOT CHANGE!! To make the gesture sequence continuous, it's equal to the last dynamic gesture + 1
	//NEW_STATIC_GEST, // Example of ne gesture, below N_STATIC should be updated
	N_STATIC_GESTURES = (POINT_AT - N_DYNAMIC_GESTURES + 1 ),  // SHOULD BE <<LAST STATIC GESTURE>> - N_DYNAMIC_GESTURES +1

	/// Total number of Gestures
	N_GESTURES = (N_DYNAMIC_GESTURES + N_STATIC_GESTURES)
};

// Available distance metrics
static enum DistanceMetrics {
	L1, // L1 Distance Metric
	HAMMING // Hamming distance metric
};

// Indicates which distance metric is to be used with each DYNAMIC gesture
static const std::map<Gesture, DistanceMetrics> Gesture2Metric = {
	{ WAVE, L1 },
	{ NOD, HAMMING},
	{ NEGATE, HAMMING }
};

struct GroundTruth {
	int userId;
	int firstFrame;
	int lastFrame;
	Gesture type;
	GroundTruth(int userid, int first, int last, Gesture g) : userId(userid), firstFrame(first), lastFrame(last), type(g) {}
};

struct GRParameters {
	std::vector<float> DynParams[N_DYNAMIC_GESTURES]; // Dynamic Gesture parameters: L1 distance threshold for WAVE, feature params for facial gestures (in order: difference between frames and number of frames between samplings)...
	float pointAtTh[3]; // SIGMA. [0] = hand-hip distance, [1] = elbow angle and [3] consecutive frames -> test values 0.25, 2.8, 25
	float gestMU[N_DYNAMIC_GESTURES]; // MU - threshold per each gesture type for the DTW
	float scores[N_GESTURES]; // Score per gesture
	float bestScore; // Best general score (mean per gesture)
	bool f1score; // If the scores are f1 (true) or overlaps (false)
};

/// Class to handle Gesture Recognition Algorithms
class GestureRecognition
{
public:
	GestureRecognition();
	~GestureRecognition();

	Gesture RecognizeGesture(std::vector<std::vector<std::vector<float>>> models, GRParameters params);
	void addFrame(const std::vector<std::vector<float>>& Dynamic_feat, const std::vector<float>& Static_Feat);
	
	void setGestureFound();
	int getMaxInputFramesSize();

	// Training
	std::vector<std::vector<std::set<int>>> constructGTsets(int nSequences, const std::vector<std::vector<GroundTruth>>& gt);
	GRParameters trainDynamicThresholds(std::vector<std::vector<std::vector<float>>> models, std::vector<std::vector<std::vector<float>>> Inputsequences,
						         std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, std::vector<float> alphas,
								 std::vector<std::vector<float>> gestTh, bool verbose);
	GRParameters trainStaticThresholds(std::vector<std::vector<std::vector<float>>> Inputsequences, const std::vector<std::vector<GroundTruth>>& gt,
									   std::vector<float> handhipdists, std::vector<float> elbowAngles, std::vector<float> nframes, bool verbose);

	float LOOCV(const std::vector<std::vector<std::vector<float>>>& models,
				std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, const std::vector<float>& alphas,
				const std::vector<std::vector<float>>& gestTh, const std::vector<float>& handhipdists, const std::vector<float>& elbowAngles, const std::vector<float>& nframes, bool verbose);

	// I/O
	static void writeParameters(GRParameters params, std::string path);
	static GRParameters readParameters(std::string path);

	static std::vector<std::vector<float>> conventionalDTW(int gestureId, const std::vector<std::vector<float>>& model, std::vector<std::vector<float>> input, float ALPHA);
	static std::deque<int> getWPath(const std::vector<std::vector<float>> &M, int t);

private:
	std::pair<Gesture, float> RealTimeDTW(const std::vector<int>& gIds, const std::vector<std::vector<std::vector<float>>*>& models, int nInputframes, std::vector<float> ALPHA, std::vector<float> MU);
	float _RealTimeDTW(int gestureId, const std::vector<std::vector<float>>& model, int nInputframes, float ALPHA, float MU);

	std::deque<int> getWPath(const SlidingMatrix<float> &M, int t);
	float staticGetureRecognition(int gestureId, float pointAtTh[3]);

	std::vector<float> getNextFrame(int gestId);
	void resetCurrentFrames();
	void clearFrames();

	// Private training utilities
	float getDynamicSequencesOverlap(int gestureId, const std::vector<std::vector<float>>& model, const std::vector<std::vector<std::vector<float>>>& sequences,
							  const std::vector<std::vector<std::set<int>>>& gt, float ALPHA, float MU);
	float getDynamicSequenceOverlap(int gestureId, const std::vector<std::vector<float>>& model, const std::vector<std::vector<float>>& sequence,
							 const std::set<int>& gt, float ALPHA, float MU);
	float getStaticSequenceOverlap(const std::vector<std::vector<float>>& sequence, const std::set<int>& gt, std::vector<float>& pointAtTh);
	float getStaticSequencesOverlap(int gestureId, const std::vector<std::vector<std::vector<float>>>& sequences, const std::vector<std::vector<std::set<int>>>& gt, std::vector<float> staticThresholds);



	// Fields
	std::vector<std::queue<std::vector<float>>> inputFrames; // One frame per gesture
	std::vector<int> currentFrames; // Current frame id of each gesture
	omp_lock_t omp_lock; // Lock to avoid race conditions...
	
	bool gestureFound;
	static const float INF;
};

