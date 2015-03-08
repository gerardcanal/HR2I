#include "stdafx.h"
#include "GRValidation.h"

GRValidation::GRValidation() {}
GRValidation::~GRValidation() {}

void GRValidation::constructBinaryGTsequences(int donotcare, bool verbose) {
	gt_seqs = binaryGTseqs(gt.size());
	dc_masks = dontcaremasks(gt.size(), std::vector<std::vector<std::vector<int>>>());
	
	int perc_count = 0;
	int num_its = 0;
	if (verbose) { // Count number of iterations to perform
		for (int u = 0; u < gt.size(); ++u)
			for (int s = 0; s < gt[u].size(); ++s)
				for (int g = 0; g < gt[u][s].size(); ++g) ++num_its;
		std::cout << "Constructing binary ground truth data and masks... 0.00%";
	}

	for (int u = 0; u < gt.size(); ++u) { // User loop
		gt_seqs[u] = std::vector<std::vector<std::vector<int>>>(gt[u].size());
		dc_masks[u] = std::vector<std::vector<std::vector<int>>>(gt[u].size());
		for (int s = 0; s < gt[u].size(); ++s) { // Sequence loop
			gt_seqs[u][s] = std::vector<std::vector<int>>(N_GESTURES, std::vector<int>(sk_seqs[u][s].size(), 0));
			dc_masks[u][s] = std::vector<std::vector<int>>(N_GESTURES, std::vector<int>(sk_seqs[u][s].size(), 1)); // Mask at 1 so we'll put 0s at what needs to be changed
			// gt[u][s] is a vector with the gt of the gestures of the sth sequence of the uth user
			for (int g = 0; g < gt[u][s].size(); ++g) { // For each gesture of the sequence
				int gid = gt[u][s][g].type;
				int nframes = sk_seqs[u][s].size();
				for (int i = gt[u][s][g].firstFrame; i <= gt[u][s][g].lastFrame; ++i) {
					gt_seqs[u][s][gid][i] = 1;
				}

				// Prepare dont care mask
				if (donotcare > 0) {
					for (int j = max(0, gt[u][s][g].firstFrame - int(donotcare / 2.0)); j <= min((gt[u][s][g].firstFrame + int(donotcare / 2.0)), dc_masks[u][s][gid].size()-1); ++j) {
						dc_masks[u][s][gid][j] = 0; // Set the surroundings to 0
					}
					for (int j = max(0, gt[u][s][g].lastFrame - int(donotcare / 2.0)); j <= min((gt[u][s][g].lastFrame + int(donotcare / 2.0)), dc_masks[u][s][gid].size()-1); ++j) {
						dc_masks[u][s][gid][j] = 0; // Set the surroundings to 0
					}
				}
				if (verbose) Utils::printPercentage(++perc_count, num_its);
			}
		}
	}
	dc_frames = donotcare;
	if (verbose) std::cout << std::endl;
}




void GRValidation::loadData(std::string path, std::string gt_path, bool filterunknown, bool verbose) {
	bool rightBody = true;
	std::tr2::sys::directory_iterator it(path);
	int nSeqs = count_if(it, std::tr2::sys::directory_iterator(),
		[](const std::tr2::sys::directory_entry & d) {return !((std::string(d.path()).find("_features") != std::string::npos) || 
															  (std::string(d.path()).find("_faces") != std::string::npos) ||
															  std::tr2::sys::is_directory(d.path())); });
	int seq = 0;
	int begin = time(NULL);
	if (verbose) std::cout << "Loading data sequences... 0.00%";

	for (it = std::tr2::sys::directory_iterator(path); it != std::tr2::sys::directory_iterator(); ++it) {
		if ((std::string(it->path()).find("_features") != std::string::npos) || (std::string(it->path()).find("_faces") != std::string::npos) || std::tr2::sys::is_directory(it->path())) continue;
		std::string seqName = std::string(it->path());
		seqName = seqName.substr(seqName.find_last_of("\\") + 1, seqName.size());
		seqName = seqName.substr(0, seqName.find(".csv"));
		std::vector<GroundTruth> _gt = GRValidation::readGroundTruthFile(gt_path + "\\gt_" + seqName + ".csv", filterunknown);
		std::vector<Skeleton> _sk = Skeleton::gestureFromCSV(path + "\\" + seqName + ".csv");
		std::vector<Face> _faces = Face::faceGestureFromCSV(path + "\\" + seqName +"_faces.csv");
		//std::vector<std::vector<float>> _feats = Skeleton::gestureFeaturesFromCSV(path + "\\" + seqName + "_features.csv");
		if (_gt.size() > 0) {
			int uid = _gt[0].userId;
			if (uid >= gt.size()) { // Resize... 
				gt.resize(uid + 1, std::vector<std::vector<GroundTruth>>());
				sk_seqs.resize(uid + 1, std::vector<std::vector<Skeleton>>());
				face_seqs.resize(uid + 1, std::vector<std::vector<Face>>());
				feat.resize(uid + 1, std::vector<std::vector<std::vector<std::vector<float>>>>());
			}
			gt[uid].push_back(_gt);
			sk_seqs[uid].push_back(_sk);
			face_seqs[uid].push_back(_faces);
			feat[uid].push_back(std::vector<std::vector<std::vector<float>>>(N_GESTURES));
			//feat[uid][feat[uid].size() - 1][WAVE] = _feats;

			// Compute sequence features
			for (int k = 0; k < _sk.size(); ++k) {
				feat[uid][feat[uid].size() - 1][WAVE].push_back(_sk[k].getDynamicGestureRecognitionFeatures(rightBody));
				feat[uid][feat[uid].size() - 1][POINT_AT].push_back(_sk[k].getStaticGestureRecognitionFeatures(rightBody, true));
			}
		}
		if (verbose) Utils::printPercentage(++seq, nSeqs);
	}
	use_user = std::vector<bool>(gt.size(), true);
	if (verbose) {
		std::cout << ". It took " << float(time(NULL)-begin) / 60.0 << " minutes. " << std::endl;
	}
}

float GRValidation::binaryOverlap(const std::vector<int>& gt, const std::vector<int>& det, const std::vector<int>& dncmask) {
	assert(gt.size() == det.size() && dncmask.size() == det.size());
	float _intersect = 0;
	float _union = 0;
	for (int i = 0; i < gt.size(); ++i) {
		_intersect += ((gt[i] * dncmask[i]) == 1) && ((det[i] * dncmask[i]) == 1);
		_union += ((gt[i] * dncmask[i]) == 1) || ((det[i] * dncmask[i]) == 1);
	}
	if (_intersect == 0) return 0;
	return _intersect / _union;
}

void GRValidation::testovlp() {
	std::vector<int> m = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };// , 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	std::vector<int> g = { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
	std::vector<int> d = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };

	std::set<int> g1;
	std::set<int> d1;

	for (int i = 0; i < g.size(); ++i) {
		if (g[i] * m[i] == 1) g1.insert(i);
		if (d[i] * m[i] == 1) d1.insert(i);
	}
	std::cout << "Normal ovlp: " << Utils::overlap(d1, g1) << " Binary ovlp: " << binaryOverlap(g, d, m) << std::endl;
	int fgt, lgt, fd, ld;
	fgt = 3; lgt = 6;
	fd = 2; ld = 2;
	std::cout << "New ovlp: " << Utils::overlap(fgt, lgt, fd, ld) << std::endl;
}


float GRValidation::sequenceF1(int user, int seq, int gest, const std::vector<std::pair<int, int>>& detections, float ovlp_th) {
	int TP = 0;
	int FP = 0;
	int FN = 0;

	std::vector<bool> used(detections.size(), false);
	for (int i = 0; i < gt[user][seq].size(); ++i) {
		if (gt[user][seq][i].type == gest) {
			bool tpfound = false;
			for (int j = 0; j < detections.size(); ++j) {
				if (Utils::overlap(gt[user][seq][i].firstFrame, gt[user][seq][i].lastFrame, detections[j].first, detections[j].second) >= ovlp_th) {
					if (used[j]) {
						++FP;
						continue;
					}
					TP++;
					used[j] = true;
					tpfound = true;
					break;
				}
			}
			if (!tpfound) FN++;
		}
	}
	for (int i = 0; i < used.size(); ++i) FP += !used[i];
	if ((TP + FP + FN) == 0) return 1; // As there was no gesture of this type (there'd be FP in such case)
	return Utils::F1measure(TP, FP, FN);
}


// Returns a vector of sequences of ground truth
std::vector<GroundTruth> GRValidation::readGroundTruthFile(std::string path, bool filterunknown) {
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
		int uid, first, last, gest;
		iss >> uid >> delim >> gest >> delim >> first >> delim >> last;
		if (filterunknown && gest == -1) continue;
		gt.push_back(GroundTruth(uid, first - 1, last - 1, static_cast<Gesture>(gest))); // -1 as the gt frames go from 1 to N.
	}
	return gt;
}


void GRValidation::syncFaceFrames(int fbs, int user, int seq, int gest) {
	int lastFrame = 0;
	std::vector<int> _new;
	for (int i = 0; i < gt_seqs[user][seq][gest].size(); ++i) {
		if (++lastFrame%fbs == 0){
			_new.push_back(gt_seqs[user][seq][gest][i]);
		}
	}

	std::vector<std::pair<int, int>> new_gt;
	int first = -1;
	for (int i = 1; i < _new.size(); ++i) {
		if ((_new[i - 1] == 0) && (_new[i] == 1))  first = i;
		if ((_new[i - 1] == 1) && (_new[i] == 0)) {
			new_gt.push_back(std::make_pair(first, i-1));
		}
	}

	// Update values
	int j = 0;
	for (int i = 0; i < gt[user][seq].size(); ++i) {
		if (gt[user][seq][i].type == gest) {
			gt[user][seq][i].firstFrame = new_gt[j].first;
			gt[user][seq][i].lastFrame = new_gt[j].second;
			++j;
		}
	}
	gt_seqs[user][seq][gest] = _new;

	// Recompute donotcare
	dc_masks[user][seq][gest] = std::vector<int>(_new.size(), 1);
	if (dc_frames > 0) {
		for (int _g = 0; _g < gt[user][seq].size(); ++_g) { // For each gesture of the sequence
			int gid = gt[user][seq][_g].type;
			if (gid != gest) continue;
			for (int j = max(0, gt[user][seq][_g].firstFrame - int(dc_frames / 2.0)); j <= min((gt[user][seq][_g].firstFrame + int(dc_frames / 2.0)), dc_masks[user][seq][gid].size()-1); ++j) {
				dc_masks[user][seq][gid][j] = 0; // Set the surroundings to 0
			}
			for (int j = max(0, gt[user][seq][_g].lastFrame - int(dc_frames / 2.0)); j <= min((gt[user][seq][_g].lastFrame + int(dc_frames / 2.0)), dc_masks[user][seq][gid].size()-1); ++j) {
				dc_masks[user][seq][gid][j] = 0; // Set the surroundings to 0
			}
		}
	}
}


// If usef1 is false, overlap is used
float GRValidation::getDynamicSequenceScore(bool usef1, int gestureId, int user, int seq, const std::vector<std::vector<float>>& model, const std::vector<std::vector<float>>& sequence, float ALPHA, float MU, float ovlp_th) {
	float score;
	std::vector<int> bin_detections(sequence.size(), 0);
	std::vector<std::pair<int, int>> detections;	
	// Get M
	std::vector<std::vector<float>> M = GestureRecognition::conventionalDTW(gestureId, model, sequence, ALPHA);
	// Iterate over the last row to find all paths < threshold
	int NM = M.size() - 1; // Num of rows
	for (int j = 0; j < M[0].size(); ++j) { // for each column...
		if (M[NM][j] < MU) {
			std::deque<int> W = GestureRecognition::getWPath(M, j); // Warping path
			// Add wpath to the set... it'll remove repetitions
			// change for fill vector or get pairs of start,end frames std::copy(W.begin(), W.end(), std::inserter(detectedFrames, detectedFrames.end()));
			if (usef1) {
				detections.push_back(std::make_pair(W[0], W[W.size() - 1]));
			}
			else { // overlap, fill vector
				std::fill(bin_detections.begin()+W[0], bin_detections.begin()+W[W.size()-1] + 1, 1); // +1 at the end because if fills [first, last)
			}
		}
	}

	if (usef1) { // f1
		detections = Utils::detectionsUnion(detections);
		return sequenceF1(user, seq, gestureId, detections, ovlp_th);
	}
	// ovlp
	return binaryOverlap(gt_seqs[user][seq][gestureId], bin_detections, dc_masks[user][seq][gestureId]);
}


float GRValidation::getAllDynamicSequencesScore(bool usef1, int gestureId, const std::vector<std::vector<float>>& model, float ALPHA, float MU, float ovlp_th) {
	float score = 0;
	int used = 0;
	for (int u = 0; u < gt.size(); ++u) {
		if (use_user[u]) {
			float user_score = 0;
			for (int s = 0; s < gt[u].size(); ++s) { // Process all the sequences of the user u
				user_score += getDynamicSequenceScore(usef1, gestureId, u, s, model, feat[u][s][gestureId], ALPHA, MU, ovlp_th);
			}
			user_score /= gt[u].size();
			score += user_score;
			++used;
		}
	}
	return score/used;
}

#define DIST_TH	0.45
float GRValidation::getStaticSequenceScore(bool usef1, int user, int seq, const std::vector<std::vector<float>>& sequence, std::vector<float>& pointAtTh, float ovlp_th) {
	int consFrames = 0;
	float dist = 0;
	std::vector<float> lastHandPose;

	float score;
	std::vector<int> bin_detections(sequence.size(), 0);
	std::vector<std::pair<int, int>> detections;

	for (int i = 0; i < sequence.size(); ++i)  {
		if (sequence[i][0] > pointAtTh[0] && sequence[i][1] > pointAtTh[1]) {
			++consFrames;
			std::vector<float> handPose(sequence[i].begin() + 2, sequence[i].begin() + 5);
			if (consFrames > 1) dist += Utils::euclideanDistance(lastHandPose, handPose);
			lastHandPose = handPose;
			if (consFrames >= pointAtTh[2]) {
				if (dist < DIST_TH) { // Recognized!
					if (usef1) {
						detections.push_back(std::make_pair(i - consFrames + 1, i));
					}
					else { // overlap, fill vector
						std::fill(bin_detections.begin() + (i - consFrames + 1), bin_detections.begin() + i + 1, 1); // +1 at the end because if fills [first, last)
					}

				}
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
	}

	int gestureId = POINT_AT;
	if (usef1) { // f1
		detections = Utils::detectionsUnion(detections);
		return sequenceF1(user, seq, gestureId, detections, ovlp_th);
	}
	// ovlp
	return binaryOverlap(gt_seqs[user][seq][gestureId], bin_detections, dc_masks[user][seq][gestureId]);
}

float GRValidation::getAllStaticSequencesScore(bool usef1, int gestureId, std::vector<float>& pointAtTh, float ovlp_th) {
	float score = 0;
	int used = 0;
	for (int u = 0; u < gt.size(); ++u) {
		if (use_user[u]) {
			float user_score = 0;
			for (int s = 0; s < gt[u].size(); ++s) { // Process all the sequences of the user u
				user_score += getStaticSequenceScore(usef1, u, s, feat[u][s][gestureId], pointAtTh, ovlp_th);
			}
			user_score /= gt[u].size();
			score += user_score;
			++used;
		}
	}
	return score / used;
}

// Dyn params are a vector of params for each gesture that have a vector for each parameter and a vector of params at the innest level
GRParameters GRValidation::bestParametersSelection(bool usef1, float ovlp_th, const std::vector<std::vector<float>>& wave_model, std::vector<std::vector<Face>>& face_models,
												  const std::vector<std::vector<std::vector<float>>>& dyn_params, const std::vector<std::vector<float>>& dyn_mu,
												  const std::vector<std::vector<float>>& st_params, bool verbose, bool printPercent) {
	time_t begin = time(NULL);
	int totaliterations = 0;
	if (verbose || printPercent) {
		std::cout << "\tParameter selection method... 0.00%";
		for (int g = 0; g < N_GESTURES; ++g) {
			if (g == WAVE) totaliterations += dyn_params[g][0].size()*dyn_mu[g].size();
			else if (g == NOD || g == NEGATE) totaliterations += dyn_params[g][0].size()*dyn_params[g][1].size()*dyn_mu[g].size();
			else if (g == POINT_AT) totaliterations += st_params[0].size()*st_params[1].size()*st_params[2].size();
		}
	}
	int curriteration = 0;

	GRParameters bests; // to fill in
	bests.DynParams[WAVE] = std::vector<float>(1);
	bests.DynParams[NEGATE] = std::vector<float>(2);
	bests.DynParams[NOD] = std::vector<float>(2);
	bests.bestScore = -1;
	bests.scores[0] = bests.scores[1] = bests.scores[2] = bests.scores[3] = -1;

	for (int g = 0; g < N_GESTURES; ++g) {
		if (g == WAVE) {
			
			#pragma omp parallel for
			for (int a = 0; a < dyn_params[g][0].size(); ++a) {// alpha for distance
				for (int mu = 0; mu < dyn_mu[g].size(); ++mu) {
					// get ovlp/f1 of wave with this parameters
					float score = getAllDynamicSequencesScore(usef1, g, wave_model, dyn_params[g][0][a], dyn_mu[g][mu], ovlp_th);
					#pragma omp critical (paramsUpdate)
					{
						if (score > bests.scores[g]) { // This params improved the gesture recognition
							bests.gestMU[g] = dyn_mu[g][mu];
							bests.DynParams[g][0] = dyn_params[g][0][a];
							bests.scores[g] = score;
							#pragma omp flush
						}
					}
					if (verbose || printPercent) {
						#pragma omp critical (print)
						Utils::printPercentage(++curriteration, totaliterations);
					}
				}
			}
		}
		else if (g == NOD || g == NEGATE) {
			for (int fbs = 0; fbs < dyn_params[g][1].size(); ++fbs) { // frames before sampling i.e. picking new frame for feature computation
				// Backup pre syncronization
				std::vector<std::vector<std::vector<GroundTruth>>> gt_bak = gt;
				binaryGTseqs gt_seqs_bak = gt_seqs;
				dontcaremasks dc_masks_bak = dc_masks;
				//Sync frames
				for (int _u = 0; _u < feat.size(); ++_u) {
					for (int _s = 0; _s < feat[_u].size(); ++_s) {
						syncFaceFrames(dyn_params[g][1][fbs], _u, _s, g);
					}
				}

				for (int dbf = 0; dbf < dyn_params[g][0].size(); ++dbf) { // Difference between increments in successive frames
					// Compute new model and feature sequences
					std::vector<std::vector<float>> _model = Face::getFeatures(dyn_params[g][0][dbf], dyn_params[g][1][fbs], face_models[g]);
					for (int _u = 0; _u < feat.size(); ++_u) {
						if (!use_user[_u]) continue;
						for (int _s = 0; _s < feat[_u].size(); ++_s) {
							feat[_u][_s][g] = Face::getFeatures(dyn_params[g][0][dbf], dyn_params[g][1][fbs], face_seqs[_u][_s]);
						}
					}


					#pragma omp parallel for // Here to avoid syncronization of the feat variable and still have a minimum parallelism
					for (int mu = 0; mu < dyn_mu[g].size(); ++mu) {
						// get ovlp/f1 of facial gestures
						float score = getAllDynamicSequencesScore(usef1, g, _model, 0.0, dyn_mu[g][mu], ovlp_th); // ALPHA is not used in this case so we put 0.0
						#pragma omp critical (paramsUpdate)
						{
							if (score > bests.scores[g]) { // This params improved the gesture recognition
								bests.gestMU[g] = dyn_mu[g][mu];
								bests.DynParams[g][0] = dyn_params[g][0][dbf];
								bests.DynParams[g][1] = dyn_params[g][1][fbs];
								bests.scores[g] = score;
								#pragma omp flush
							}
						}
						if (verbose || printPercent) {
							#pragma omp critical (print)
							Utils::printPercentage(++curriteration, totaliterations);
						}
					}
				}
				// Restore baks
				gt = gt_bak;
				gt_seqs = gt_seqs_bak;
				dc_masks = dc_masks_bak;
			}
		}
		else if (g == POINT_AT) {
			#pragma omp parallel for
			for (int hhd = 0; hhd < st_params[0].size(); ++hhd) { // hand-hip distance
				for (int ea = 0; ea < st_params[1].size(); ++ea) { // elbow angle
					for (int cf = 0; cf < st_params[2].size(); ++cf) { // consecutive frames
						// get sequences overlap/f1 with this parameters
						std::vector<float> th = { st_params[0][hhd], st_params[1][ea], st_params[2][cf] };
						float score = getAllStaticSequencesScore(usef1, g, th, ovlp_th);
						#pragma omp critical (paramsUpdate)
						{
							if (score > bests.scores[g]) { // This params improved the gesture recognition
								bests.pointAtTh[0] = th[0];
								bests.pointAtTh[1] = th[1];
								bests.pointAtTh[2] = th[2];
								bests.scores[g] = score;
								#pragma omp flush
							}
						}
						if (verbose || printPercent) {
							#pragma omp critical (print)
							Utils::printPercentage(++curriteration, totaliterations);
						}
					}
				}
			}
		}
	}
	bests.bestScore = std::accumulate(bests.scores, bests.scores + N_GESTURES, 0.0f) / N_GESTURES;
	if (verbose) {
		std::cout << "\nDone! Parameters are:" << std::endl;
		for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) {
			std::cout << "\tDynamic parameters for gesture " << i << ": ";
			for (int j = 0; j < bests.DynParams[i].size(); ++j) std::cout << bests.DynParams[i][j] << " ";
			std::cout << std::endl;
		}
		for (int i = 0; i < N_DYNAMIC_GESTURES; ++i) std::cout << "\tMU for gesture " << i << ": " << bests.gestMU[i] << std::endl;
		std::cout << "\tStatic params: " << bests.pointAtTh[0] << " " << bests.pointAtTh[1] << " " << bests.pointAtTh[2] << std::endl;
		for (int i = 0; i < N_GESTURES; ++i) std::cout << "\t" << (usef1 ? "F1" : "Overlap") << " score for gesture " << i << ": " << bests.scores[i] << std::endl;
		std::cout << "\tMean " << (usef1? "F1" : "overlap") << " score: " << bests.bestScore << std::endl;
		if (usef1) std::cout << "\tF1's overlap threshold: " << ovlp_th << std::endl;
		
	}
	if (verbose || printPercent) std::cout << "\tIt took " << float(time(NULL) - begin) << " seconds (" << float(time(NULL) - begin) / 60.0 << " minutes)." << std::endl;
	bests.f1score = usef1;
	return bests;
}

// Leave One Subject Out Cross Validation
void GRValidation::LOSOCV(bool usef1, float ovlp_th, const std::vector<std::vector<float>>& wave_model, std::vector<std::vector<Face>>& face_models,
						  const std::vector<std::vector<std::vector<float>>>& dyn_params, const std::vector<std::vector<float>>& dyn_mu,
					      const std::vector<std::vector<float>>& st_params, bool verbose) {
	time_t begin = time(NULL);
	if (verbose) std::cout << "LOSOCV method..." << std::endl;

	std::vector<std::vector<std::vector<float>>> scores(gt.size()); // Each user, each sequence, each gesture a score

	for (int u = 0; u < gt.size(); ++u) { // u is the left-out user
		if (verbose) std::cout << "User fold " << u+1 << "/" << gt.size() << "..." << std::endl;
		use_user[u] = false;
		scores[u] = std::vector<std::vector<float>>(gt[u].size(), std::vector<float>(N_GESTURES));

		//train!
		GRParameters pars = bestParametersSelection(usef1, ovlp_th, wave_model, face_models, dyn_params, dyn_mu, st_params, false);
		
		// Test
		for (int g = 0; g < N_GESTURES; ++g) {
			if (g == WAVE) {
				for (int s = 0; s < gt[u].size(); ++s) { // Process all the sequences of the user u
					scores[u][s][g] += getDynamicSequenceScore(usef1, g, u, s, wave_model, feat[u][s][g], pars.DynParams[g][0], pars.gestMU[g], ovlp_th);
				}
			}
			else if (g == NOD || g == NEGATE) {
				// Backup pre syncronization
				std::vector<std::vector<std::vector<GroundTruth>>> gt_bak = gt;
				binaryGTseqs gt_seqs_bak = gt_seqs;
				dontcaremasks dc_masks_bak = dc_masks;

				// Compute new model and feature sequences
				std::vector<std::vector<float>> _model = Face::getFeatures(pars.DynParams[g][0], pars.DynParams[g][1], face_models[g]);
				for (int s = 0; s < gt[u].size(); ++s) {
					syncFaceFrames(pars.DynParams[g][1], u, s, g);
					feat[u][s][g] = Face::getFeatures(pars.DynParams[g][0], pars.DynParams[g][1], face_seqs[u][s]);
					scores[u][s][g] = getDynamicSequenceScore(usef1, g, u, s, _model, feat[u][s][g], 0.0, pars.gestMU[g], ovlp_th); // Alpha is not used
				}
				// Restore baks
				gt = gt_bak;
				gt_seqs = gt_seqs_bak;
				dc_masks = dc_masks_bak;
			}
			else if (g == POINT_AT) {
				std::vector<float> pa_th = { pars.pointAtTh[0], pars.pointAtTh[1], pars.pointAtTh[2] };
				for (int s = 0; s < gt[u].size(); ++s) { // Process all the sequences of the user u
					scores[u][s][g] = getStaticSequenceScore(usef1, u, s, feat[u][s][g], pa_th, ovlp_th);
				}
			}
		}
		use_user[u] = true;
	}

	// Write results
	std::string scoretype = (usef1 ? "f1" : "ovlp");
	std::string fname = "Parameters/LOSO_results_" + scoretype + ".csv";
	bool write_header = !std::tr2::sys::exists(std::tr2::sys::path(fname));
	std::ofstream f(fname, std::ios_base::app);
	if (write_header) {
		f << "sep=, " << std::endl;
		if (usef1) f << "Ovlp_th, ";
		else f << "DontCare_frames, ";
		for (int u = 0; u < scores.size(); ++u) {
			for (int s = 0; s < scores[u].size(); ++s)
				for (int g = 0; g < scores[u][s].size(); ++g)
					f << "User" << u << "-Seq" << s << "Gest" << g << ", ";
			for (int g = 0; g < N_GESTURES; ++g) f << "MeanUser" << u << "-Gest" << g << ", ";
			f << "GlobalMeanUser" << u << ", ";
		}
		for (int g = 0; g < N_GESTURES; ++g) f << "GlobalMeanGesture" << g << ", ";
		f << "GlobalScore" << std::endl;
	}

	if (usef1) f << ovlp_th << ", ";
	else f << dc_frames << ", ";
	float gmeans[N_GESTURES] = { 0 }; // Mean of each user's sequences for each gesture
	for (int u = 0; u < scores.size(); ++u) {
		float gscore[N_GESTURES] = { 0 }; // Score per gesture of user u
		for (int s = 0; s < scores[u].size(); ++s) {
			for (int g = 0; g < scores[u][s].size(); ++g) {
				f << scores[u][s][g] << ", ";
				gscore[g] += scores[u][s][g];
			}
		}
		float meanuser = 0;
		for (int g = 0; g < N_GESTURES; ++g) {
			f << gscore[g] / scores[u].size() << ", ";
			gmeans[g] += gscore[g] / scores[u].size();
			meanuser += gscore[g] / scores[u].size();
		}
		f << meanuser / N_GESTURES << ", ";
	}
	float globscore = 0;
	for (int g = 0; g < N_GESTURES; ++g) {
		f << gmeans[g] / scores.size() << ", ";
		globscore += gmeans[g] / scores.size();
	}
	f << globscore / N_GESTURES << std::endl;
	if (verbose) std::cout << "\tDone! Global score = " << globscore / N_GESTURES << std::endl;
	f.close();
}



// Static
void GRValidation::getTestParams(std::vector<std::vector<std::vector<float>>>& dyn_params, std::vector<std::vector<float>>& dyn_mu, std::vector<std::vector<float>>& st_params) {
	dyn_params[WAVE] = std::vector<std::vector<float>>(1);
	dyn_params[WAVE][0] = { 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f };
	dyn_params[NOD] = std::vector<std::vector<float>>(2);
	dyn_params[NOD][0] = { 5, 10, 15, 20, 25, 30 };
	dyn_params[NOD][1] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
	dyn_params[NEGATE] = std::vector<std::vector<float>>(2);
	dyn_params[NEGATE][0] = { 5, 10, 15, 20, 25, 30 };
	dyn_params[NEGATE][1] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
	
	dyn_mu[WAVE] = { 6.75f, 7.0f, 7.25f, 7.5f, 8.0f, 8.5f, 8.75f, 9.0f, 9.25f, 9.5f };
	dyn_mu[NOD] = { 2.75f, 3, 3.5f, 3.75f, 4, 4.25f, 4.5f, 4.75f, 5, 5.5f, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 20 };
	dyn_mu[NEGATE] = { 2.75f, 3, 3.5f, 3.75f, 4, 4.25f, 4.5f, 4.75f, 5, 5.5f, 7, 8, 9, 10, 11, 12,  13, 14, 15, 17, 20 };

	st_params[0] = { 0.1f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f }; // handhip dist
	st_params[1] = { 2.0f, 2.15f, 2.25f, 2.3f, 2.35f, 2.4f, 2.45f, 2.55f }; // elbow angles
	st_params[2] = { 3, 5, 10, 15, 20, 25, 30, 35 }; // consecutive frames
}

void GRValidation::computeAndWriteBestParams(bool usef1, std::string seqpath, std::string gtpath, std::string modelspath, float ovlp_th) {
	//time_t begin = time(NULL);
	GRValidation grv;
	int dnc = (usef1)? 0 : int(ovlp_th);
	grv.loadData(seqpath, gtpath, true);
	grv.constructBinaryGTsequences(dnc);
	
	// Models
	std::vector<std::vector<float>> wave_model = Skeleton::gestureFeaturesFromCSV(modelspath + "/HelloModel/HelloModel_features.csv");
	std::vector<std::vector<Face>> face_models(N_GESTURES);
	face_models[NOD] = Face::faceGestureFromCSV(modelspath + "/YesFacialModel/YesFacialModel_faces.csv");
	face_models[NEGATE] = Face::faceGestureFromCSV(modelspath + "/NoFacialModel/NoFacialModel_faces.csv");

	// Params
	std::vector<std::vector<std::vector<float>>> dyn_params(N_DYNAMIC_GESTURES);
	std::vector<std::vector<float>> dyn_mu(N_DYNAMIC_GESTURES);
	std::vector<std::vector<float>> st_params(3);
	GRValidation::getTestParams(dyn_params, dyn_mu, st_params);

	GRParameters params = grv.bestParametersSelection(usef1, ovlp_th, wave_model, face_models, dyn_params, dyn_mu, st_params, true);
	GestureRecognition::writeParameters(params, "Parameters/GestureRecognitionParameters.txt");
	//std::cout << "It took " << float(time(NULL) - begin) / 60.0 << " minutes." << std::endl;
}

void GRValidation::exhaustiveLOSOCV(std::string seqpath, std::string gtpath, std::string modelspath) {
	// Remove old files
	/*std::ofstream f("Parameters/LOSO_results_f1.csv", std::ofstream::out | std::ofstream::trunc);
	f.close();
	f.open("Parameters/LOSO_results_ovlp.csv", std::ofstream::out | std::ofstream::trunc);
	f.close();*/

	time_t begin = time(NULL);
	GRValidation grv;
	grv.loadData(seqpath, gtpath, true);
	grv.constructBinaryGTsequences(0);

	// Models
	std::vector<std::vector<float>> wave_model = Skeleton::gestureFeaturesFromCSV(modelspath + "/HelloModel/HelloModel_features.csv");
	std::vector<std::vector<Face>> face_models(N_GESTURES);
	face_models[NOD] = Face::faceGestureFromCSV(modelspath + "/YesFacialModel/YesFacialModel_faces.csv");
	face_models[NEGATE] = Face::faceGestureFromCSV(modelspath + "/NoFacialModel/NoFacialModel_faces.csv");

	// Params
	std::vector<std::vector<std::vector<float>>> dyn_params(N_DYNAMIC_GESTURES);
	std::vector<std::vector<float>> dyn_mu(N_DYNAMIC_GESTURES);
	std::vector<std::vector<float>> st_params(3);
	GRValidation::getTestParams(dyn_params, dyn_mu, st_params);

	std::cout << "F1 tests..." << std::endl;

	for (float th = 0.05; th <= 1.0; th += 0.05) {
		grv.LOSOCV(true, th, wave_model, face_models, dyn_params, dyn_mu, st_params, true);
		std::cout << std::endl;
	}

	std::cout << "Overlap tests..." << std::endl;

	std::vector<int> dc_frames = { 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21 };
	for (int i = 0; i < dc_frames.size(); ++i) {
		grv.constructBinaryGTsequences(dc_frames[i], false);
		grv.LOSOCV(false, 0.0, wave_model, face_models, dyn_params, dyn_mu, st_params, true); // th is not used in this case...
		std::cout << std::endl;
	}
	time_t end = time(NULL);
	std::cout << "Exhaustive LOSO CV took " << float(end - begin) / 60.0 << " minutes (" << float(end - begin) / 3600.0 << " hours)." << std::endl;
}