#pragma once
#include "GestureRecognition.h"
#include "Face.h"
#include "Skeleton.h"
#include <vector>
#include <filesystem>
#include <iostream>
#include <string>
#include <assert.h>
#include <algorithm>
#include <numeric>

typedef std::vector<std::vector<std::vector<std::vector<int>>>> binaryGTseqs; // user, sequence, gesture, binary sequence
typedef std::vector<std::vector<std::vector<std::vector<int>>>> dontcaremasks;


class GRValidation
{
public:
	void testovlp();
	GRValidation();
	~GRValidation();
	void loadData(std::string path, std::string gt_path, bool filterunkown = true, bool verbose = true);
	void constructBinaryGTsequences(int donotcare, bool verbose = true);

	GRParameters bestParametersSelection(bool usef1, float ovlp_th, const std::vector<std::vector<float>>& wave_model, std::vector<std::vector<Face>>& face_models,
										 const std::vector<std::vector<std::vector<float>>>& dyn_params, const std::vector<std::vector<float>>& dyn_mu,
										 const std::vector<std::vector<float>>& st_params, bool verbose, bool printPercent = true);

	void LOSOCV(bool usef1, float ovlp_th, const std::vector<std::vector<float>>& wave_model, std::vector<std::vector<Face>>& face_models,
		        const std::vector<std::vector<std::vector<float>>>& dyn_params, const std::vector<std::vector<float>>& dyn_mu,
		        const std::vector<std::vector<float>>& st_params, bool verbose);

	static void computeAndWriteBestParams(bool usef1, std::string seqpath, std::string gtpath, std::string modelspath, float ovlp_th=0.05);
	static void exhaustiveLOSOCV(std::string seqpath, std::string gtpath, std::string modelspath);
	static std::vector<GroundTruth> readGroundTruthFile(std::string path, bool filterunknown);
private:
	// Methods
	float binaryOverlap(const std::vector<int>& gt, const std::vector<int>& det, const std::vector<int>& mask);
	float sequenceF1(int user, int seq, int gest, const std::vector<std::pair<int, int>>& detections, float ovlp_th);

	void syncFaceFrames(int fbs, int user, int seq, int gest);

	float getDynamicSequenceScore(bool usef1, int gestureId, int user, int seq, const std::vector<std::vector<float>>& model, const std::vector<std::vector<float>>& sequence, float ALPHA, float MU, float ovlp_th);
	float getAllDynamicSequencesScore(bool usef1, int gestureId, const std::vector<std::vector<float>>& model, float ALPHA, float MU, float ovlp_th);
	float getStaticSequenceScore(bool usef1, int user, int seq, const std::vector<std::vector<float>>& sequence, std::vector<float>& pointAtTh, float ovlp_th);
	float getAllStaticSequencesScore(bool usef1, int gestureId, std::vector<float>& pointAtTh, float ovlp_th);

	static void getTestParams(std::vector<std::vector<std::vector<float>>>& dyn_params, std::vector<std::vector<float>>& dyn_mu, std::vector<std::vector<float>>& st_params);

	// Fields
	std::vector<std::vector<std::vector<GroundTruth>>> gt; //For each user, for each sequence, list of gesture's GT
	std::vector<std::vector<std::vector<Skeleton>>> sk_seqs; // For each user, for each sequence, list of skeletons (a sequence)
	std::vector<std::vector<std::vector<Face>>> face_seqs; // For each user, for each sequence, list of faces (a sequence)
	std::vector<std::vector<std::vector<std::vector<std::vector<float>>>>> feat; // For each user, for each sequence, for each gesture, list of features (a list of floats)


	binaryGTseqs gt_seqs; // Binary sequences with the ground truth for each user, sequence and gesture (1s where 0s)
	dontcaremasks dc_masks; // Binary sequences masks for the do not care approach (1s where the gesture must remain the same, 0 in the do not care parts)
	int dc_frames;

	std::vector<bool> use_user; // Vector which indicates which user has to be used to compute the scores
};

