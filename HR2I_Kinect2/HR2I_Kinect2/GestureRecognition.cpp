#include "GestureRecognition.h"
#include <assert.h>
#include <numeric>
#include <ctime>
#include <iostream>
#include <fstream>

const float GestureRecognition::INF = std::numeric_limits<float>::infinity();

GestureRecognition::GestureRecognition() {
	inputFrames = std::vector<std::queue<std::vector<float>>>(N_GESTURES);
	omp_init_lock(&omp_lock);
	gestureFound = false;
	currentFrames = std::vector<int>(N_GESTURES, 0);
}


GestureRecognition::~GestureRecognition()
{
	omp_destroy_lock(&omp_lock);
}

///Returns the gesture ID, from the Gesture enum.
Gesture GestureRecognition::RecognizeGesture(std::vector<std::vector<std::vector<float>>> models, GRParameters params) {
	assert(models.size() == N_DYNAMIC_GESTURES);
	#ifndef _OPENMP
		std::cerr << "OpenMP is required to execute the code. Please enable it." << std::endl;
		throw std::exception("OpenMP is required to execute the code. Please enable it.");
	#endif
	
	gestureFound = false;
	const int iFrames = 30;
	std::vector<float> gest(static_cast<int>(N_GESTURES), INF);

	#pragma omp parallel for num_threads(N_GESTURES) schedule(static, 1) shared(gest) // Should create only one thread per iteration/gesture
	for (int i = 0; i < N_GESTURES; ++i) {
		float cost;
		if (static_cast<Gesture>(i) == POINT_AT) cost = staticGetureRecognition(i, params.pointAtTh);
		else cost = RealTimeDTW(i, models[i], iFrames, params.ALPHA[i], params.gestTh[i]);
		
		#pragma omp critical (gesturefound) // So the gestureFound variable is safely updated for the other threads to see
		{
			gestureFound = gestureFound || cost < INF; // If gestureFound was set to true, leave it to true to force all the threads to stop
			gest[i] = cost;
			#pragma omp flush //Ensure memory consistency
		}
	}

	int i_min = std::distance(gest.begin(), std::min_element(gest.begin(), gest.end()));

	return static_cast<Gesture>(i_min);
}

#define DIST_TH	0.45
float GestureRecognition::staticGetureRecognition(int gestureId, float pointAtTh[3]) {
	bool found = false;
	int consFrames = 0; // consecutive frames
	float dist = 0; // Dist of the hand position during the recognition frames
	std::vector<float> lastHandPose;
	while (!found) { // For t = 0..INF
		while (inputFrames[gestureId].empty()); // Wait until we have a new frame...
		std::vector<float> input = getNextFrame(gestureId);
		if (input[0] == -1.0 && input[1] == -1) break; // Frame which means end of sequence... nothing was recognized

		if (input[0] > pointAtTh[0] && input[1] > pointAtTh[1]) {
			++consFrames;
			std::vector<float> handPose(input.begin() + 2, input.begin() + 5);
			if (consFrames > 1) dist += Utils::euclideanDistance(lastHandPose, handPose);
			lastHandPose = handPose;
			if (consFrames > pointAtTh[2]) {
				if (dist < DIST_TH) return 0.0;
				else {
					dist = 0;
					consFrames = 0;
				}
			}
		}
		else {
			consFrames = 0;
			dist = 0;
		}

		#pragma omp critical (gesturefound)
			found = gestureFound; // To avoid race conditions
	}
	return INF; // Not found!
}

float GestureRecognition::RealTimeDTW(int gestureId, const std::vector<std::vector<float>>& model, int nInputFrames, float ALPHA, float MU) {
	int NM = model.size() + 1; // rows
	int NI = nInputFrames; // columns

	// Initialize matrix M
	SlidingMatrix<float> M(NM, NI, INF); // Access in the matrix is by M[column][row]
	for (int j = 0; j < NI; ++j) M[j][0] = 0; // First row set to 0

	// Computing M matrix
	int t = 1; bool slide = false; bool found = false;
	while (!found) { // For t = 0..INF
		while (inputFrames[gestureId].empty()); // Wait until we have a new frame...
		std::vector<float> input = getNextFrame(gestureId);
		if (input[0] == -1.0 && input[1] == -1) break; // Frame which means end of sequence... nothing was recognized

		for (int i = 1; i < NM; ++i) {
			float neighbors[] = { M[t][i - 1], M[t - 1][i - 1], M[t - 1][i] }; // Upper, upper-left and left neighbors
			M[t][i] = Utils::L1Distance(model[i-1], input, ALPHA) + *std::min_element(neighbors, neighbors + 3); // i-1 as i goes from 1 to N, and the indices in model from 0 to N-1
		}

		// Check for gesture recognition
		if (M[t][NM-1] < MU) return M[t][NM-1];

		// End of loop checks
		if (slide)  // Reached the limit of the matrix...
			M.slide();
		else {
			slide = t == (NI - 1);
			t = (slide)? NI-1 : t+1;
		}
		#pragma omp critical (gesturefound)
			found = gestureFound; // To avoid race conditions
	}

	return INF;
}

std::vector<std::vector<float>> GestureRecognition::conventionalDTW(const std::vector<std::vector<float>>& model, std::vector<std::vector<float>> input, float ALPHA) {
	int NM = model.size() + 1;
	int NI = input.size() + 1;

	// Initialize matrix M
	std::vector<std::vector<float>> M(NM, std::vector<float>(NI, INF));
	for (int j = 0; j < NI; ++j) M[0][j] = 0;

	// Computing M matrix
	for (int t = 1; t < NI; ++t) {
		for (int i = 1; i < NM; ++i) {
			float neighbors[] = {M[i-1][t], M[i-1][t-1], M[i][t-1]}; // Upper, upper-left and left neighbors
			float min = *std::min_element(neighbors, neighbors + 3);
			M[i][t] = Utils::L1Distance(model[i - 1], input[t-1], ALPHA) + *std::min_element(neighbors, neighbors + 3); // i-1 as i goes from 1 to N, and the indices in model from 0 to N-1
		}
	}
	return M;
}

std::deque<int> GestureRecognition::getWPath(const std::vector<std::vector<float>> &M, int t) {
	std::deque<int> path;
	int i = M.size()-1; // Last row
	int j = t;
	while (i > 1 && j > 0) {
		float _min = min(M[i - 1][j], min(M[i - 1][j - 1], M[i][j - 1])); // Min between Upper, upper-left and left neighbors
		path.push_front(j-1); // -1 because the matrix begans in 1 (has an extra column) so we have to align it with the gesture
		if (_min == M[i - 1][j - 1]) {
			j--;
			i--;
		}
		else if (_min == M[i - 1][j]) {
			i--;
		}
		else { // _min == M[i][j - 1]
			j--;
		}
	}
	path.push_front(j-1); // Last element
	return path;
}

std::deque<int> GestureRecognition::getWPath(const SlidingMatrix<float> &M, int t) {
	// Remember acces is M[column][row]
	std::deque<int> path;
	int i = M.rows()-1; // Last row
	int j = t;
	while (i > 1 && j > 0) {
		path.push_front(j-1);// -1 because the matrix begans in 1 (has an extra column) so we have to align it with the gesture
		float _min = min(M[j][i - 1], min(M[j - 1][i - 1], M[j - 1][i])); // Min between Upper, upper-left and left neighbors
		if (_min == M[j - 1][i - 1]) {
			j--;
			i--;
		}
		else if (_min == M[j][i - 1]) {
			i--;
		}
		else { // _min == M[j - 1][i]
			j--;
		}
	}
	path.push_front(j-1); // Last element
	return path;
}

void GestureRecognition::addFrame(const std::vector<float>& Dynamic_feat, const std::vector<float>& Static_feat) {
	omp_set_lock(&omp_lock);
	// Dynamic frames
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) {
		inputFrames[i].push(Dynamic_feat);
	}
	// Static frames
	for (int i = POINT_AT; i < (POINT_AT + N_STATIC_GESTURES); ++i) {
		inputFrames[i].push(Static_feat);
	}
	omp_unset_lock(&omp_lock);
}

/*void GestureRecognition::addFrame(const std::vector<float>& frame) {
	omp_set_lock(&omp_lock);
	for (int i = 0; i < N_GESTURES; ++i) 
		inputFrames[i].push(frame);
	omp_unset_lock(&omp_lock);
}

void GestureRecognition::addFrame(const std::vector<float>& feat, const std::vector<float>& extendedFeat) {
	omp_set_lock(&omp_lock);
	for (int i = 0; i < N_GESTURES; ++i) {
		if (static_cast<Gesture>(i) == POINT_AT) inputFrames[i].push(extendedFeat);
		else inputFrames[i].push(feat);
	}
	omp_unset_lock(&omp_lock);
}

void GestureRecognition::addFrames(std::vector<std::vector<float>> framelist) {
	// It gets the locks as having frames is essential for the working threads to make any job...
	// Also it'll mainly be used to initialize the class from a frame sequence
	omp_set_lock(&omp_lock); 
	for (int f = 0; f < framelist.size(); ++f) {
		for (int i = 0; i < N_GESTURES; ++i)
			inputFrames[i].push(framelist[f]);
	}
	omp_unset_lock(&omp_lock);
	#pragma omp flush
}*/

void GestureRecognition::clearFrames() {
	omp_set_lock(&omp_lock);
	for (int i = 0; i < inputFrames.size(); ++i) {
		inputFrames[i] = std::queue<std::vector<float>>();
	}
	omp_unset_lock(&omp_lock);
	#pragma omp flush
}

std::vector<float> GestureRecognition::getNextFrame(int gestureId) {
	omp_set_lock(&omp_lock);
	std::vector<float> next = inputFrames[gestureId].front();
	inputFrames[gestureId].pop();
	currentFrames[gestureId]++;
	omp_unset_lock(&omp_lock);
	return next;
}

void GestureRecognition::resetCurrentFrames() {
	omp_set_lock(&omp_lock); // Just to make sure...
	currentFrames = std::vector<int>(N_GESTURES, 0);
	omp_unset_lock(&omp_lock);
}

float GestureRecognition::getSequenceOverlap(const std::vector<std::vector<float>>& model, const std::vector<std::vector<float>>& sequence,
											 const std::set<int>& gt, float ALPHA, float MU) {
	std::set<int> detectedFrames;
	// Get M
	std::vector<std::vector<float>> M = conventionalDTW(model, sequence, ALPHA);
	// Iterate over the last row to find all paths < threshold
	int NM = M.size() - 1; // Num of rows
	for (int j = 0; j < M[0].size(); ++j) { // for each column...
		if (M[NM][j] < MU) {
			std::deque<int> W = getWPath(M, j); // Warping path
			// Add wpath to the set... it'll remove repetitions
			std::copy(W.begin(), W.end(), std::inserter(detectedFrames, detectedFrames.end()));
		}
	}
	return Utils::overlap(detectedFrames, gt);
}

float GestureRecognition::getSequencesOverlap(int gestureId, const std::vector<std::vector<float>>& model, const std::vector<std::vector<std::vector<float>>>& sequences, 
											  const std::vector<std::vector<std::set<int>>>& gt, float ALPHA, float MU) {
	float seq_ovlp = 0;
	for (int s = 0; s < sequences.size(); ++s) { // And do it for each sequence...

		seq_ovlp += getSequenceOverlap(model, sequences[s], gt[s][gestureId], ALPHA, MU);

	}
	return seq_ovlp/sequences.size();
}

GRParameters GestureRecognition::trainThresholds(std::vector<std::vector<std::vector<float>>> models, std::vector<std::vector<std::vector<float>>> Inputsequences,
												 std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, std::vector<float> alphas,
												 std::vector<std::vector<float>> gestTh, bool verbose) {
	// Initializations
	assert(N_DYNAMIC_GESTURES == gestTh.size());
	time_t begin = time(NULL);
	const bool rightBody = true;

	// Count iterations and prepare output in verbose case
	int aux = 0;
	for (int i = 0; i < gestTh.size(); ++i) {
		/*if (static_cast<Gesture>(i) == POINT_AT) aux += gestTh[i].size()*restTh.size();
		else*/ aux += gestTh[i].size();
	}
	const int totalIter = alphas.size()*aux;
	int perc = 0;
	if (verbose) std::cout << "The number to iterations to perform is " << totalIter << std::endl << "\tWorking... 0.00%";

	// Construct sets of gt frames for each sequence and each gesture for faster overlap computation
	std::vector<std::vector<std::set<int>>> gt_sets(Inputsequences.size(), std::vector<std::set<int>>(N_DYNAMIC_GESTURES));
	for (int i = 0; i < Inputsequences.size(); ++i) {
		for (int j = 0; j < N_DYNAMIC_GESTURES; ++j) {
			for (int k = 0; k < gt[i].size(); ++k) {
				if (j == gt[i][k].type) {
					//Construct range
					std::vector<int> _v(gt[i][k].lastFrame - gt[i][k].firstFrame + 1);
					std::iota(_v.begin(), _v.end(), gt[i][k].firstFrame);
					// Add to set
					std::copy(_v.begin(), _v.end(), std::inserter(gt_sets[i][j], gt_sets[i][j].end()));
				}
			}
		}
	}

	//Set models for the point at
	//models[POINT_AT] = addThirdFeature(models[POINT_AT]);

	// Return variable:
	GRParameters params;
	params.bestOvlp = -1; // Best overlap
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) params.ovlps[i] = -1; // Best overlap for each gesture
	
	omp_set_num_threads(omp_get_max_threads()); // Force use of max number of threads
	#pragma omp parallel for
	for (int a = 0; a < alphas.size(); ++a) {// For each ALPHA - distance threshold
		// For each type of gesture
		for (int g = 0; g < gestTh.size(); ++g) {
			// Check all the thresholds of the gesture with current ALPHA
			for (int t = 0; t < gestTh[g].size(); ++t) {
				if (static_cast<Gesture>(g) == POINT_AT) { /*// If the gesture is a Point At we have to look for the restTh
					for (int r = 0; r < restTh.size(); ++r) { // For each restTh
						// Add the third feature
						std::vector<std::vector<std::vector<float>>> seqs = Inputsequences;
						for (int s = 0; s < Inputsequences.size(); ++s) {
							for (int k = 0; k < seqs[s].size(); ++k) inputSkeletons[s][k].addExtendedGRFeature(seqs[s][k], rightBody, restTh[r]);
						}
						float ovlp = getSequencesOverlap(g, models[g], seqs, gt_sets, alphas[a], gestTh[g][t]);
						#pragma omp critical (paramsUpdate)
						{
							if (ovlp > params.ovlps[g]) {
								params.ovlps[g] = ovlp;
								params.gestTh[g] = gestTh[g][t];
								params.restTh = restTh[r];
								params.ALPHA[g] = alphas[a];
								#pragma omp flush
							}
						}
						if (verbose) {
							#pragma omp critical (print)
							Utils::printPercentage(++perc, totalIter);
						}
					}*/
					continue;
				}
				else {
					float ovlp = getSequencesOverlap(g, models[g], Inputsequences, gt_sets, alphas[a], gestTh[g][t]);
					#pragma omp critical (paramsUpdate)
					{
						if (ovlp > params.ovlps[g]) {
							params.ovlps[g] = ovlp;
							params.gestTh[g] = gestTh[g][t];
							params.ALPHA[g] = alphas[a];
							#pragma omp flush
						}
					}
					if (verbose) {
						#pragma omp critical (print)
						Utils::printPercentage(++perc, totalIter);
					}
				}
			}
		}
		float Ovlp = std::accumulate(params.ovlps, params.ovlps + N_DYNAMIC_GESTURES, 0.0f) / N_DYNAMIC_GESTURES;
		#pragma omp critical (paramsUpdate)
		{
			if (Ovlp > params.bestOvlp) {
				params.bestOvlp = Ovlp;
				#pragma omp flush
			}
		}
	}
	if (verbose) {
		std::cout << "\nDone! Parameters are:" << std::endl;
		for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) std::cout << "\tALPHA for gesture " << i << ": " << params.ALPHA[i] << std::endl;
		for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) std::cout << "\tMU for gesture " << i << ": " << params.gestTh[i] << std::endl;
		//std::cout << "\trestThreshold: " << params.restTh << std::endl;
		for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) std::cout << "\tOverlap for gesture " << i << ": " << params.ovlps[i] << std::endl;
		std::cout << "\tBest overlap: " << params.bestOvlp << std::endl;
		std::cout << "It took " << float(time(NULL) - begin)/60.0 << " minutes." << std::endl;
	}
	return params;
}

float GestureRecognition::LOOCV(const std::vector<std::vector<std::vector<float>>>& models, const std::vector<std::vector<std::vector<float>>>& Inputsequences,
								std::vector<std::vector<Skeleton>>& inputSkeletons, const std::vector<std::vector<GroundTruth>>& gt, const std::vector<float>& alphas,
								const std::vector<std::vector<float>>& gestTh, bool verbose) {
	time_t begin = time(NULL);
	float overlap = 0;
	bool rightBody = true;

	// Construct sets of gt frames for each sequence and each gesture for faster overlap computation
	std::vector<std::vector<std::set<int>>> gt_sets(Inputsequences.size(), std::vector<std::set<int>>(N_DYNAMIC_GESTURES));
	for (int i = 0; i < Inputsequences.size(); ++i) {
		for (int j = 0; j < N_DYNAMIC_GESTURES; ++j) {
			for (int k = 0; k < gt[i].size(); ++k) {
				if (j == gt[i][k].type) {
					//Construct range
					std::vector<int> _v(gt[i][k].lastFrame - gt[i][k].firstFrame + 1);
					std::iota(_v.begin(), _v.end(), gt[i][k].firstFrame);
					// Add to set
					std::copy(_v.begin(), _v.end(), std::inserter(gt_sets[i][j], gt_sets[i][j].end()));
				}
			}
		}
	}

	std::vector<std::vector<std::vector<float>>> _models = models;
	//_models[POINT_AT] = addThirdFeature(_models[POINT_AT]);
	for (int i = 0; i < Inputsequences.size(); ++i) { // i will be the test sequence
		// Fast and dirty way... copy all the vectors and remove the selected one...
		std::vector<std::vector<std::vector<float>>> _sequences = Inputsequences;
		std::vector<std::vector<Skeleton>> _skels = inputSkeletons;
		std::vector<std::vector<GroundTruth>> _gt = gt;

		// Remove the test fold...
		_sequences.erase(_sequences.begin() + i);
		_skels.erase(_skels.begin() + i);
		_gt.erase(_gt.begin() + i);

		// Train
		if (verbose) std::cout << "LOOCV fold number " << i << std::endl << "\t";
		GRParameters gr = trainThresholds(models, _sequences, _skels, _gt, alphas, gestTh, verbose); // Note trainThresholds also adds the third feature... 
		float gest_overlap = 0;
		for (int j = 0; j < N_DYNAMIC_GESTURES; ++j) {
			if (static_cast<Gesture>(j) == POINT_AT) { /*// add third feature...
				std::vector<std::vector<float>> seq = Inputsequences[i];
				for (int k = 0; k < seq.size(); ++k) inputSkeletons[i][k].addExtendedGRFeature(seq[k], rightBody, gr.restTh);
				gest_overlap += getSequenceOverlap(_models[j], seq, gt_sets[i][j], gr.ALPHA[j], gr.gestTh[j]);*/
				continue;
			}
			else gest_overlap += getSequenceOverlap(_models[j], Inputsequences[i], gt_sets[i][j], gr.ALPHA[j], gr.gestTh[j]);
		}
		overlap += gest_overlap / N_DYNAMIC_GESTURES;
	}
	overlap = overlap / Inputsequences.size();
	if (verbose) {
		std::cout << "Resulting mean overlap of the " << Inputsequences.size() << " sequences is: " << overlap << std::endl;
		std::cout << "All the LOOCV took " << float(time(NULL) - begin)/60.0 << " minutes." << std::endl;
	}
	return overlap;
}

/*std::vector<std::vector<float>> GestureRecognition::addThirdFeature(std::vector<std::vector<float>> model) {
	// For the Point At gesture...
	for (int i = 0; i < model.size(); ++i) model[i].push_back(0.0f);
	return model;
}*/

void GestureRecognition::writeParameters(GRParameters params, std::string path) {
	std::ofstream of;
	of.open(path);
	if (!of.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	of << "ALPHA = ";
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) of << " " << params.ALPHA[i]; 
	of << std::endl;
	of << "PointAtThresholds =";
	for (int i = 0; i < _countof(params.pointAtTh); ++i) of << " " << params.pointAtTh[i];
	of << std::endl;
	of << "MU =";
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) of << " " << params.gestTh[i];
	of << std::endl;
	of << "gestOverlapings =";
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) of << " " << params.ovlps[i];
	of << std::endl;
	of << "bestOverlapping = " << params.bestOvlp << std::endl;
	of.close();
}

GRParameters GestureRecognition::readParameters(std::string path) {
	GRParameters params;
	std::ifstream ifs;
	ifs.open(path);
	if (!ifs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::string s;
	// ALPHA
	ifs >> s >> s; // ALPHA =
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) ifs >> params.ALPHA[i]; //values
	
	// POINT AT
	ifs >> s >> s;
	for (int i = 0; i < _countof(params.pointAtTh); ++i) ifs >> params.pointAtTh[i];

	// MU
	ifs >> s >> s; // MU =
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) ifs >> params.gestTh[i]; //values

	// ovlps
	ifs >> s >> s;
	for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) ifs >> params.ovlps[i];

	// ovl
	ifs >> s >> s >> params.bestOvlp;
	return params;
}

std::vector<GroundTruth> GestureRecognition::readGrountTruth(std::string path) {
	if (path.substr(path.size() - 4, 4) != ".csv") path = path + ".csv";
	std::ifstream ifs(path, std::ifstream::in);
	if (!ifs.is_open()) {
		std::string err = "ERROR: file " + path + " could not be opened. Is the path okay?";
		std::cerr << err << std::endl;
		throw std::exception(err.c_str());
	}
	std::vector<GroundTruth> gt;
	std::string line;
	std::getline(ifs, line); // Read header line
	while (line.find("sep") != line.npos)
		std::getline(ifs, line);

	std::string delim; // comma consumer
	while (std::getline(ifs, line)) {
		std::istringstream iss(line);
		int first, last, gest;
		iss >> gest >> delim >> first >> delim >> last;
		gt.push_back(GroundTruth(first-1, last-1, static_cast<Gesture>(gest))); // -1 as the gt frames go from 1 to N.
	}
	return gt;
}