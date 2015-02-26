// Faces feature selection tests
#include "stdafx.h"
#include "GestureRecognition.h"
#include "Utils.h"
#include "Face.h"
#include <string>
#include <filesystem>
using namespace std;

vector<vector<float>> getFeatures(int dbf, int numoff, vector<Face>& seq) { // Distance between two used frames to be considered as diferences, number of frames to wait before using another frame (we pick a frame every numoff frames)
	int lastFrame = 0;
	vector<vector<float>> feat;
	int oldpitch = 0, oldyaw = 0, oldroll = 0;
	for (int i = 0; i < seq.size(); ++i) {
		if (++lastFrame == numoff){ // Every numoff frames
			double pitch, yaw, roll;
			Utils::ExtractFaceRotationInDegrees(&seq[i].getFaceRotation(), &pitch, &yaw, &roll);

			int difroll = roll - oldroll;
			int difyaw = yaw - oldyaw;
			int difpitch = pitch - oldpitch;

			if (abs(difroll) >= dbf) difroll = (0 < difroll) - (difroll < 0);
			else difroll = 0;
			if (abs(difyaw) >= dbf) difyaw = (0 < difyaw) - (difyaw < 0);
			else difyaw = 0;
			if (abs(difpitch) >= dbf) difpitch = (0 < difpitch) - (difpitch < 0);
			else difpitch = 0;

			feat.push_back({ (float)difpitch, (float)difyaw, (float)difroll });

			oldyaw = yaw; oldroll = roll; oldpitch = pitch;

			lastFrame = 0;
		}
	}
	return feat;
}

bool tryRecognize(int gestureId, vector<vector<float>> model, vector<vector<float>> seq, float MU) {
	GestureRecognition gr;
	std::vector<std::vector<float>> M = gr.conventionalDTW(gestureId, model, seq, 1);
	// Iterate over the last row to find all paths < threshold
	int NM = M.size() - 1; // Num of rows
	for (int j = 0; j < M[0].size(); ++j) { // for each column...
		if (M[NM][j] < MU) {
			return true; // RECOGNIZED
		}
	}
	return false;
}

void facestests() {
	string path = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\HR2I\\GestureRecorder\\GestureRecorder\\gestures\\";
	ofstream res_file("faces_tests_results.csv");
	TCHAR Buffer[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, Buffer); 
	std::wcout << "Results will be written in: " << Buffer << "\\faces_tests_results.csv" << std::endl;

	//Get Data
	vector<Face> yes_model = Face::faceGestureFromCSV(path + "YesFacialModel\\YesFacialModel0_faces.csv");
	vector<Face> no_model = Face::faceGestureFromCSV(path + "NoFacialModel\\NoFacialModel0_faces.csv");

	vector<vector<Face>> yes_seqs;
	tr2::sys::path p(path + "YesFacialExamples\\");
	for (tr2::sys::directory_iterator it(p); it != tr2::sys::directory_iterator(); ++it) {
		string file = string(it->path());
		if (file.find("_faces") == string::npos) continue; // We only want faces files
		yes_seqs.push_back(Face::faceGestureFromCSV(file));
	}

	vector<vector<Face>> no_seqs;
	p = tr2::sys::path(path + "NoFacialExamples\\");
	for (tr2::sys::directory_iterator it(p); it != tr2::sys::directory_iterator(); ++it) {
		string file = string(it->path());
		if (file.find("_faces") == string::npos) continue; // We only want faces files
		no_seqs.push_back(Face::faceGestureFromCSV(file));
	}

	vector<vector<Face>> unknown_seqs;
	p = tr2::sys::path(path + "UnknownFacialExamples\\");
	for (tr2::sys::directory_iterator it(p); it != tr2::sys::directory_iterator(); ++it) {
		string file = string(it->path());
		if (file.find("_faces") == string::npos) continue; // We only want faces files
		unknown_seqs.push_back(Face::faceGestureFromCSV(file));
	}

	// Best results
	float bestF1 = -1, bestF1yes, bestF1no;
	float bestyesF1 = -1, bestyesF1global, bestyesF1no;
	float bestnoF1 = -1, bestnoF1global, bestnoF1yes;
	int bestyesTP, bestyesFP, bestyesTN, bestyesFN, bestnoTP, bestnoFP, bestnoTN, bestnoFN;
	int byyesTP, byyesFP, byyesTN, byyesFN, bynoTP, bynoFP, bynoTN, bynoFN;
	int bnyesTP, bnyesFP, bnyesTN, bnyesFN, bnnoTP, bnnoFP, bnnoTN, bnnoFN;
	int bestdbf, bestnof, bestmu;
	int bydbf, bynof, bymu;
	int bndbf, bnnof, bnmu;


	// Parameters to test
	vector<int> difsBetweenFrames = { 5, 10, 15, 20 };
	vector<int> numberOfFrames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20}; // between data each data used
	vector<int> DTWthresholds = { 5, 7, 9, 11, 13, 15, 17, 19, 21 };

	int tot_iter = difsBetweenFrames.size() * numberOfFrames.size() * DTWthresholds.size() * (yes_seqs.size()*2 + no_seqs.size()*2 + unknown_seqs.size()*2);
	int curr_iter = 0;
	cout << "The number to iterations to perform is " << tot_iter << std::endl << "\tWorking... 0.00%";
	
	res_file << "sep=, \nDBF, NumOfFrames, MU, YTP, YFP, YTN, YFN, NTP, NFP, NTN, NFN, globalF1, yesF1, noF1" << endl;

	for (int i = 0; i < difsBetweenFrames.size(); ++i)	{
		int dbf = difsBetweenFrames[i];
		for (int j = 0; j < numberOfFrames.size(); ++j) {
			int nof = numberOfFrames[j];
			// Get Models' features
			vector<vector<float>> yes_features = getFeatures(dbf, nof, yes_model);
			vector<vector<float>> no_features = getFeatures(dbf, nof, no_model);
			
			// Get sequences' features
			vector<vector<vector<float>>> yes_seq_feat(yes_seqs.size());
			for (int si = 0; si < yes_seqs.size(); si++) {
				yes_seq_feat[si] = getFeatures(dbf, nof, yes_seqs[si]);
			}
			vector<vector<vector<float>>> no_seq_feat(no_seqs.size());
			for (int si = 0; si < no_seqs.size(); si++) {
				no_seq_feat[si] = getFeatures(dbf, nof, no_seqs[si]);
			}
			vector<vector<vector<float>>> unknown_seq_feat(unknown_seqs.size());
			for (int si = 0; si < unknown_seqs.size(); si++) {
				unknown_seq_feat[si] = getFeatures(dbf, nof, unknown_seqs[si]);
			}

			for (int k = 0; k < DTWthresholds.size(); ++k) {
				int dtwth = DTWthresholds[k];

				int yesTP, yesFP, yesTN, yesFN;
				int noTP, noFP, noTN, noFN;
				yesTP = yesFP = yesTN = yesFN = noTP = noFP = noTN = noFN = 0;

				// YES DTW -> all sequences
				for (int l = 0; l < yes_seq_feat.size(); ++l) { // YES sequences
					bool detected = tryRecognize(NOD, yes_features, yes_seq_feat[l], dtwth);
					if (detected) ++yesTP;
					else ++yesFN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}
				for (int l = 0; l < no_seq_feat.size(); ++l) { // NO sequences
					bool detected = tryRecognize(NOD, yes_features, no_seq_feat[l], dtwth);
					if (detected) ++yesFP;
					else ++yesTN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}
				for (int l = 0; l < unknown_seq_feat.size(); ++l) { // UNKNOWN sequences
					bool detected = tryRecognize(NOD, yes_features, unknown_seq_feat[l], dtwth);
					if (detected) ++yesFP;
					else ++yesTN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}

				// NO DTW -> all sequences
				for (int l = 0; l < yes_seq_feat.size(); ++l) { // YES sequences
					bool detected = tryRecognize(NEGATE, no_features, yes_seq_feat[l], dtwth);
					if (detected) ++noFP;
					else ++noTN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}
				for (int l = 0; l < no_seq_feat.size(); ++l) { // NO sequences
					bool detected = tryRecognize(NEGATE, no_features, no_seq_feat[l], dtwth);
					if (detected) ++noTP;
					else ++noFN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}
				for (int l = 0; l < unknown_seq_feat.size(); ++l) { // UNKNOWN sequences
					bool detected = tryRecognize(NEGATE, no_features, unknown_seq_feat[l], dtwth);
					if (detected) ++noFP;
					else ++noTN;
					Utils::printPercentage(++curr_iter, tot_iter);
				}

				// Get F1
				float yesF1 = Utils::F1measure(yesTP, yesFP, yesFN);
				float noF1 = Utils::F1measure(noTP, noFP, noFN);
				float globalF1 = Utils::F1measure(yesTP + noTP, yesFP + noFP, yesFN + noFN);

				if (globalF1 > bestF1) {
					bestyesTP = yesTP; bestyesFP = yesFP; bestyesTN = yesTN; bestyesFN = yesFN; bestnoTP = noTP; bestnoFP = noFP; bestnoTN = noTN; bestnoFN = noFN;
					bestdbf = dbf; bestnof = nof; bestmu = dtwth;
					bestF1 = globalF1;
					bestF1yes = yesF1; bestF1no = noF1;
				}
				if (yesF1 > bestyesF1) {
					byyesTP = yesTP; byyesFP = yesFP; byyesTN = yesTN; byyesFN = yesFN;	bynoTP = noTP; bynoFP = noFP; bynoTN = noTN; bynoFN = noFN;
					bydbf = dbf; bynof = nof; bymu = dtwth;
					bestyesF1 = yesF1;
					bestyesF1global = globalF1; bestyesF1no = noF1;
				}
				if (noF1 > bestnoF1) {
					bnyesTP = yesTP; bnyesFP = yesFP; bnyesTN = yesTN; bnyesFN = yesFN; bnnoTP = noTP; bnnoFP = noFP; bnnoTN = noTN; bnnoFN = noFN;
					bndbf = dbf; bnnof = nof; bnmu = dtwth;
					bestnoF1 = noF1;
					bestnoF1global = globalF1; bestnoF1yes = yesF1;
				}


				// Print results
				res_file << dbf << ", " << nof << ", " << dtwth << ", " << yesTP << ", " << yesFP << ", " << yesTN << ", " << yesFN << ", " << noTP << ", "
					<< noFP << ", " << noTN << ", " << noFN << ", " << globalF1 << ", " << yesF1 << ", " << noF1 <<endl;
			}
		}
	}
	res_file << ", , , , , , , , , , , , , " << endl;
	res_file << ", , , , , , , , , , , , , " << endl;


	res_file << bestdbf << ", " << bestnof << ", " << bestmu << ", " << bestyesTP << ", " << bestyesFP << ", " << bestyesTN << ", " << bestyesFN << ", " << bestnoTP << ", "
		<< bestnoFP << ", " << bestnoTN << ", " << bestnoFN << ", " << bestF1 << ", " << bestF1yes << ", " << bestF1no << " <- BEST GLOBAL" << endl;
	
	res_file << bydbf << ", " << bynof << ", " << bymu << ", " << byyesTP << ", " << byyesFP << ", " << byyesTN << ", " << byyesFN << ", " << bynoTP << ", "
		<< bynoFP << ", " << bynoTN << ", " << bynoFN << ", " << bestyesF1global << ", " << bestyesF1 << ", " << bestyesF1no << "<- BEST YES" << endl;
	
	res_file << bndbf << ", " << bnnof << ", " << bnmu << ", " << bnyesTP << ", " << bnyesFP << ", " << bnyesTN << ", " << bnyesFN << ", " << bnnoTP << ", "
		<< bnnoFP << ", " << bnnoTN << ", " << bnnoFN << ", " << bestnoF1global << ", " << bestnoF1yes << ", " << bestnoF1 << "<- BEST NO" << endl;

	res_file << "DBF, NumOfFrames, MU, YTP, YFP, YTN, YFN, NTP, NFP, NTN, NFN, globalF1, yesF1, noF1" << endl;

	cout << endl << "Done!" << endl;
	res_file.close();
}