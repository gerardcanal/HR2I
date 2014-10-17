#include "GestureRecognition.h"
#include <fstream>
#include "Kinect2Utils.h"
#include <filesystem>
#include <assert.h>
using namespace std;

void trainDTWParameters() {
	string resPath = "Results\\GestureRecognitionParameters.txt";
	string gestPath = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\";

	// Get the models...
	cout << "Loading the models..." << endl;
	std::vector<std::vector<std::vector<float>>> models(N_GESTURES);
	models[SALUTE] = Skeleton::gestureFeaturesFromCSV(gestPath + "HelloModel/HelloModel_features.csv");
	models[POINT_AT] = Skeleton::gestureFeaturesFromCSV(gestPath + "PointAtModel/PointAtModelShort_features.csv");

	// Load sequences -> I assume nobody changed the sequences names from TestSequenceX.csv!!!
	cout << "Loading the test sequences... 0.00%";
	tr2::sys::directory_iterator fit(gestPath + "TestSequence");
	int nSequences = count_if(fit, tr2::sys::directory_iterator(), [](const tr2::sys::directory_entry & d) {return !tr2::sys::is_directory(d.path()); }) / 2;
	std::vector<std::vector<std::vector<float>>> feat_sequences(nSequences);
	std::vector<std::vector<Skeleton>> skeletons_seq(nSequences);
	for (int i = 0; i < nSequences; ++i) {
		string basepath = gestPath + "TestSequence\\TestSequence" + to_string(i);
		skeletons_seq[i] = Skeleton::gestureFromCSV(basepath + ".csv");
		feat_sequences[i] = Skeleton::gestureFeaturesFromCSV(basepath + "_features.csv");
		assert(skeletons_seq[i].size() == feat_sequences[i].size());
		Utils::printPercentage(i+1, nSequences);
	}
	cout << endl;

	// Load ground truth
	cout << "Loading the ground truth..." << endl;
	std::vector<std::vector<GroundTruth>> gt(nSequences);
	for (int i = 0; i < nSequences; ++i) gt[i] = GestureRecognition::readGrountTruth(gestPath + "gt_testseq" + to_string(i) + ".csv");

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Set parameters.../////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/* Originals (first try ever)
	std::vector<float> restTh = { 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f };
	std::vector<std::vector<float>> gestTh(N_GESTURES);
	gestTh[SALUTE] = {5.0f, 7.0f, 9.0f, 11.0f, 13.0f, 15.0f, 17.0f, 20.0f};
	gestTh[POINT_AT] = {3.5f, 4.0f, 4.5f, 5.0f, 6.0f, 6.5f, 7.0f, 8.0f};
	std::vector<float> alphas = { 0.25f, 0.5f, 0.75f, 1.0f, 1.25f, 1.5f, 2.0f, 4.0f };*/
	
	/*Extended with info from LONG PA model -> iteration 1
	std::vector<float> restTh = { 0.2f, 0.25f, 0.3f, 0.32f, 0.35f, 0.4f, 0.45f };
	std::vector<std::vector<float>> gestTh(N_GESTURES);
	gestTh[SALUTE] = {6.0f, 6.5f, 6.75f, 7.0f, 7.25f, 7.5f, 8.0f, 9.0f };
	gestTh[POINT_AT] = {6.5f, 7.0f, 7.5f, 7.75f, 8.0f, 8.25f, 8.5f, 8.75f };
	std::vector<float> alphas = { 0.25f, 0.5f, 0.55f, 0.60f, 0.65f, 0.75f, 0.8f, 0.85f, 0.9f, 1.0f };*/
	
	/*Extended with info from LONG PA model -> iteration 2
	std::vector<float> restTh = { 0.2f, 0.25f, 0.27f, 0.3f, 0.32f, 0.35f, 0.4f };
	std::vector<std::vector<float>> gestTh(N_GESTURES);
	gestTh[SALUTE] = {6.75f, 7.0f, 7.25f, 7.5f, 7.75f, 8.0f, 8.25f, 8.5f, 9.0f };
	gestTh[POINT_AT] = {6.5f, 6.75f, 7.0f, 7.25f, 7.5f, 7.75f, 8.0f, 8.25f, 8.5f, 8.75f };
	std::vector<float> alphas = {0.0f, 0.10f, 0.15f, 0.20f, 0.25f, 0.3f, 0.5f, 0.60f, 0.65f, 0.75f, 0.8f };*/

	/*Extended with info from SHORT PA model -> iteration 1*/
	std::vector<float> restTh = { 0.2f, 0.23f, 0.25f, 0.27f, 0.3f, 0.32f, 0.35f, 0.4f };
	std::vector<std::vector<float>> gestTh(N_GESTURES);
	gestTh[SALUTE] = {6.75f, 7.0f, 7.25f, 7.5f, 8.0f, 8.5f, 8.75f, 9.0f, 9.25f };
	gestTh[POINT_AT] = {6.5f, 7.0f, 7.5f, 7.75f, 8.0f, 8.25f, 8.5f, 8.75f };
	std::vector<float> alphas = { 0.0f, 0.1f, 0.15f, 0.2f, 0.25f, 0.5f, 0.55f, 0.60f, 0.65f, 0.75f };

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Train
	cout << "Beginning the parameter selection..." << endl;
	GestureRecognition gr;
	GRParameters params = gr.trainThresholds(models, feat_sequences, skeletons_seq, gt, alphas, gestTh, restTh, true);
	GestureRecognition::writeParameters(params, resPath);
	cout << "Parameters were stored in " << resPath;
}

void showValuesGestTh() {
	float ALPH[] = { 0.25f, 0.5f, 0.75f, 1.0f, 1.25f, 1.5f };
	// Hello
	cout << "HELLO-----------------------------" << endl;
	for (int i = 0; i < _countof(ALPH); ++i) {
		cout << "For ALPHA = " << ALPH[i] << endl << "\t";
		string path = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\PointAtModel\\PointAtModel_features.csv";
		string path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\DiscardedModels\\PointAtModel0_features.csv";
		std::vector<std::vector<float>> modelf = Skeleton::gestureFeaturesFromCSV(path);
		std::vector<std::vector<float>> inputf = Skeleton::gestureFeaturesFromCSV(path2);

		// Conventional version
		GestureRecognition gr;
		vector<vector<float>> M = gr.conventionalDTW(modelf, inputf, ALPH[i]);
		int LR = M.size() - 1;
		float _min = M[M.size() - 1][0];
		for (int j = 0; j < M[0].size(); ++j) _min = min(_min, M[LR][j]);
		cout << _min << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence0_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][235] << " " << M[LR][429] << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence3_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][314] << " " << M[LR][927] << " " << M[LR][1204] << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence4_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][249] << " " << M[LR][331] << " " << M[LR][625] << " " << endl << endl;
	}

	// Point At
	cout << "POINT AT--------------------------" << endl;
	for (int i = 0; i < _countof(ALPH); ++i) {
		cout << "For ALPHA = " << ALPH[i] << endl << "\t";
		string path = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\HelloModel\\HelloModel_features.csv";
		string path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\DiscardedModels\\HelloModel2_features.csv";
		std::vector<std::vector<float>> modelf = Skeleton::gestureFeaturesFromCSV(path);
		std::vector<std::vector<float>> inputf = Skeleton::gestureFeaturesFromCSV(path2);

		// Conventional version
		GestureRecognition gr;
		vector<vector<float>> M = gr.conventionalDTW(modelf, inputf, ALPH[i]);
		int LR = M.size() - 1;
		float _min = M[M.size() - 1][0];
		for (int j = 0; j < M[0].size(); ++j) _min = min(_min, M[LR][j]);
		cout << _min << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence0_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][114] << " " << M[LR][323] << " " << M[LR][543] << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence2_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][104] << " " << M[LR][164] << " " << M[LR][855] << " ";

		path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\TestSequence4_features.csv";
		inputf = Skeleton::gestureFeaturesFromCSV(path2);
		M = gr.conventionalDTW(modelf, inputf, ALPH[i]); LR = M.size() - 1;
		cout << M[LR][108] << " " << M[LR][144] << " " << M[LR][509] << " " << endl;
	}

	/*Output:
	HELLO-----------------------------
	For ALPHA = 0.25
			0.470276 7.90151 7.45914 9.502 10.4053 9.69442 5.85882 2.26679 2.94565

	For ALPHA = 0.5
			0.631089 7.95527 9.24002 9.86956 11.8969 10.4103 7.36095 2.32653 3.00567


	For ALPHA = 0.75
			0.791901 7.98765 11.0209 10.2371 13.3884 11.1262 8.86309 2.38628 3.06568


	For ALPHA = 1
			0.952714 8.00498 12.8018 10.6047 14.88 11.8421 10.3652 2.41353 3.1257

	For ALPHA = 1.25
			1.11353 8.02233 14.5827 10.9722 16.3715 12.558 11.8674 2.42853 3.18571

	For ALPHA = 1.5
			1.27434 8.03967 16.3635 11.3398 17.8631 13.2738 13.3695 2.4391 3.24573

	POINT AT--------------------------
	For ALPHA = 0.25
			2.75977 2.59204 2.06704 2.81715 3.35906 2.60507 4.50742 4.66045 4.69215 1.27075
	For ALPHA = 0.5
			2.92612 2.87431 2.47757 3.18515 3.91555 3.14205 5.1262 5.05292 5.08421 1.4429
	For ALPHA = 0.75
			3.09064 3.12989 2.87489 3.55316 4.45097 3.67271 5.73522 5.44293 5.45555 1.60345
	For ALPHA = 1
			3.25482 3.38389 3.26648 3.92116 4.96397 4.1976 6.33457 5.82894 5.82171 1.74165
	For ALPHA = 1.25
			3.41748 3.62634 3.63543 4.28916 5.47063 4.71985 6.91002 6.20259 6.18423 1.87103
	For ALPHA = 1.5
			3.57707 3.8688 3.98691 4.65716 5.96599 5.2421 7.47383 6.54381 6.54292 2.00042 */
}

void showValuesRestTh() { // To decide the range of the thresholds. Used frames were selected manually
	string gestPath = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\TestSequence\\";
	std::vector<float> rest, point;

	std::vector<Skeleton> sk = Skeleton::gestureFromCSV(gestPath+"TestSequence0.csv");
	rest.push_back(sk[0].getElbowSpineDistance(true));
	rest.push_back(sk[175].getElbowSpineDistance(true));
	point.push_back(sk[232].getElbowSpineDistance(true));
	point.push_back(sk[431].getElbowSpineDistance(true));

	sk = Skeleton::gestureFromCSV(gestPath + "TestSequence2.csv");
	rest.push_back(sk[0].getElbowSpineDistance(true));
	point.push_back(sk[236].getElbowSpineDistance(true));

	sk = Skeleton::gestureFromCSV(gestPath + "TestSequence4.csv");
	rest.push_back(sk[0].getElbowSpineDistance(true));
	rest.push_back(sk[163].getElbowSpineDistance(true));
	point.push_back(sk[242].getElbowSpineDistance(true));
	point.push_back(sk[307].getElbowSpineDistance(true));

	cout << "Point distances:\n\t";
	for (int i = 0; i < point.size(); ++i) cout << point[i] << " ";
	cout << endl << "Rest distances:\n\t";
	for (int i = 0; i < rest.size(); ++i) cout << rest[i] << " ";
	cout << endl << endl;

	/*Output:
	Point distances:
        0.392832 0.251208 0.328018 0.303281 0.391357
	Rest distances:
        0.276977 0.267687 0.244142 0.22173 0.251135	*/
}

void testEqualDTWMethods() { // Some changes must be done in GestRec.h and in the RealTimeDTW to make it work
	string path = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\HelloModel\\HelloModel0_features.csv";
	string path2 = "C:\\Users\\Gerard\\Dropbox\\MAI\\3dSemester\\TFM\\src\\GestureRecorder\\GestureRecorder\\gestures\\HelloModel\\HelloModel1_features.csv";
	std::vector<std::vector<float>> modelf = Skeleton::gestureFeaturesFromCSV(path2);
	std::vector<std::vector<float>> inputf = Skeleton::gestureFeaturesFromCSV(path);

	float ALPH = 1.0;
	float MU = 0.000001;

	// Conventional version
	GestureRecognition gr;
	vector<vector<float>> M = gr.conventionalDTW(modelf, modelf, ALPH);
	ofstream of("DTWconventional2.txt");
	for (int i = 0; i < M.size(); ++i) {
		for (int j = 0; j < M[i].size(); ++j) of << M[i][j] << ((j + 1 == M[i].size()) ? "" : " ");
		of << endl;
	}
	of << endl;
	int NM = M.size(); // N of rows
	for (int j = 0; j < M[0].size(); ++j) { // for each column...
		if (M[NM - 1][j] < MU) {
			std::deque<int> W = gr.getWPath(M, j); // Warping path
			for (int i = 0; i < W.size(); ++i) of << W[i] << " ";
			of << endl;
		}
	}
	of.close();
	/*
	// RealTime one...
	GestureRecognition gr1; // Just in case, begin from 0
	// Set finframe for each input sequence to avoid an infinite loop waiting for more frames in the DTW
	//for (int i = 0; i < Inputsequences.size(); ++i) Inputsequences[i].push_back(std::vector<float>({ -1.0, -1.0 }));
	inputf.push_back(std::vector<float>({ -1.0, -1.0 }));
	gr1.addFrames(inputf);
	// At this point, the RealTimeDTW was modified to write the same information on the file and not return until end of the sequence...
	gr1.RealTimeDTW(0, modelf, inputf.size(), ALPH, MU);*/
}