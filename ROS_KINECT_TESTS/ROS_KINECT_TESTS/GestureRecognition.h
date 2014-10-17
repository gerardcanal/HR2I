#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include "SlidingMatrix.h"
#include <queue>
#include "Utils.h"
#include "Skeleton.h"
#include <omp.h>

#define ACC_DIST 5 // distance between the detected frame and the ground truth one

static enum Gesture {
	SALUTE,
	POINT_AT,
	N_GESTURES = (POINT_AT + 1)
};

struct GroundTruth {
	int firstFrame;
	int lastFrame;
	Gesture type;
	GroundTruth(int first, int last, Gesture g) : firstFrame(first), lastFrame(last), type(g) {}
};

struct GRParameters {
	float ALPHA; // L1 distance threshold
	float restTh; // Resting threshold
	float gestTh[N_GESTURES]; // MU - threshold per each gesture type for the DTW
	float ovlps[N_GESTURES]; // Overlap per gesture
	float bestOvlp; // Best overlap
};

/// Class to handle Gesture Recognition Algorithms
class GestureRecognition
{
public:
	GestureRecognition();
	~GestureRecognition();

	Gesture DTW(std::vector<std::vector<std::vector<float>>> models, GRParameters params);
	void addFrame(const std::vector<float>& frame);
	void addFrame(const std::vector<float>& feat, const std::vector<float>& extendedFeat);
	void addFrames(std::vector<std::vector<float>> framelist);
	GRParameters trainThresholds(std::vector<std::vector<std::vector<float>>> models, std::vector<std::vector<std::vector<float>>> Inputsequences,
						         std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, std::vector<float> alphas,
								 std::vector<std::vector<float>> gestTh, std::vector<float> restTh, bool verbose);

	float LOOCV(const std::vector<std::vector<std::vector<float>>>& models, const std::vector<std::vector<std::vector<float>>>& Inputsequences,
				std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, const std::vector<float>& alphas,
				const std::vector<std::vector<float>>& gestTh, const std::vector<float>& restTh, bool verbose);

	static std::vector<std::vector<float>> addThirdFeature(std::vector<std::vector<float>> model);
	static void writeParameters(GRParameters params, std::string path);
	static GRParameters readParameters(std::string path);
	static std::vector<GroundTruth> readGrountTruth(std::string path);

	std::vector<std::vector<float>> conventionalDTW(const std::vector<std::vector<float>>& model, std::vector<std::vector<float>> input, float ALPHA);
	std::deque<int> getWPath(const std::vector<std::vector<float>> &M, int t);

private:
	float RealTimeDTW(int gestureId, const std::vector<std::vector<float>>& model, int nInputframes, float ALPHA, float MU);
	std::deque<int> getWPath(const SlidingMatrix<float> &M, int t);

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

