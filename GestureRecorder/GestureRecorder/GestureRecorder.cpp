// GestureRecorder.cpp: define el punto de entrada de la aplicación de consola.
//

#include "stdafx.h"
#include <filesystem>
#include "Kinect2Utils.h"
#include "BodyRGBViewer.h"
#include <iostream>
#include <vector>
#include <thread>
#include <future>
#include <mutex>
#include <algorithm>
#include <string>
using namespace std;

bool recording = false;
mutex mtx;
const string BASEPATH = "gestures/";
const int GESTUREDELAY = 500; //mseconds
const int PRE_RECORD_SLEEP = 1500; //mseconds

static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInInfraredSpace 
| FaceFrameFeatures::FaceFrameFeatures_PointsInInfraredSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

pair<vector<Skeleton>, vector<Face>> recordGesture(Kinect2Utils* ku, BodyRGBViewer* view) {
	vector<Skeleton> gesture;
	vector<Face> facial_gesture;
	UINT64 id = 0;
	bool first = true;
	mtx.lock();
	bool _rec = recording;
	mtx.unlock();
	//ofstream ofs = ofstream("test.txt");
	while (_rec) {
		IBodyFrame* bodyFrame = NULL;
		IFaceFrame* faceFrame = ku->getLastFaceFrameFromDefault();

		Face face;
		if (faceFrame) {
			face = Kinect2Utils::getFaceFromFaceFrame(faceFrame, true); // true - infrared, false color
			if (view != NULL && !face.getIsEmpty()) 
				view->setFaceFrameToDraw(face);
			//facial_gesture.push_back(face);
			if (bodyFrame == NULL) {
				IBodyFrameReference* bfr = NULL;
				faceFrame->get_BodyFrameReference(&bfr);
				bfr->AcquireFrame(&bodyFrame);
				SafeRelease(bfr);
			}
		}
		/*else {
			bodyFrame = ku->getLastBodyFrameFromDefault();
		}*/

		if (bodyFrame) {
			view->setBodyFrameToDraw(bodyFrame);
			Skeleton sk = Kinect2Utils::getTrackedSkeleton(bodyFrame, id, first);
			ku->setFaceTrackingId(sk.getTrackingID());
			if (!first && id == sk.getTrackingID()) {
				gesture.push_back(sk);
				facial_gesture.push_back(face);
				//Joint* j = sk.getJoints();
				//ofs << Utils::euclideanDistance(j[JointType_ShoulderLeft], j[JointType_ShoulderRight]) << endl;
			}
			else if (first && id != sk.getTrackingID()) {
				id = sk.getTrackingID(); // Even though the skeleton is empty -i.e. id == -1- this doesn't change nything
				gesture.push_back(sk);
				facial_gesture.push_back(face);
				first = false;
			}
		}
		SafeRelease(bodyFrame); // If not the bodyFrame is not got again
		SafeRelease(faceFrame);
		mtx.lock();
		_rec = recording;
		mtx.unlock();
	}
	//ofs.close();
	return make_pair(gesture, facial_gesture);
}

void printHelp() {
	cout << "Possible options: " << endl;
	cout << "\t1 - Start recording a gesture." << endl;
	cout << "\t2 - Stop recording a gesture. (Must be done after option 1)." << endl;
	cout << "\t3 - Change the name of the gestures to record." << endl;
	cout << "\t4 - Finish the program and play the recorded gestures." << endl;
	cout << "\t5 - Finish the program and play ALL the stored gestures." << endl;
	cout << "\t6 - Finish the program and play ALL the stored gestures of ONE type." << endl;
	cout << "\t7 - Enable/Disable the manual frame control of the player of gestures." << endl;
	cout << "\t8 - Print this help." << endl << endl;
}

int getNextGestureIdFromFile(string gestName) {
	tr2::sys::path p(BASEPATH + gestName);
	vector<tr2::sys::path> files;
	for (tr2::sys::directory_iterator it(p); it != tr2::sys::directory_iterator(); ++it) {
		if ((string(it->path()).find("_features") != string::npos) || (string(it->path()).find("_faces") != string::npos)) continue;
		files.push_back(it->path());
	}
	if (files.size() == 0) return 0;

	sort(files.begin(), files.end());
	string file = files[files.size() - 1];
	file = file.substr(file.find_last_of("/") + 1, file.find_last_of(".")); // From the dir separator to the point of the extension
	return atoi(&file[file.size()-1]) + 1;
}

void createFolders(string foldpath) {
	tr2::sys::path bp(foldpath);
	tr2::sys::create_directories(bp);
}

void playStoredGestures(BodyRGBViewer& gestview, string gestToPlay, bool enableControl) { // If gestToPlay is "", ALL gestures are played
	cout << endl << "All the stored gestures are going to be played: " << endl;
	tr2::sys::path p(BASEPATH);
	for (tr2::sys::directory_iterator it(p); it != tr2::sys::directory_iterator(); ++it) {
		tr2::sys::path folder = it->path();
		if (tr2::sys::is_directory(folder)) {
			// Get gesture name
			tr2::sys::directory_iterator fit(folder);
			int nGestures = count_if(fit, tr2::sys::directory_iterator(),
									 [](const tr2::sys::directory_entry & d) {return !tr2::sys::is_directory(d.path()); });
			string sFolder = folder;
			string gestName = sFolder.substr(sFolder.find_last_of("\\") + 1, sFolder.size()); // Damn it windows and his backslashes..
			if (gestToPlay != "" && gestName != gestToPlay) continue;

			cout << "\tPlaying the " << nGestures/3 << " recorded gestures of type \"" << gestName << "\"." << endl;

			// Play the gestures
			for (fit = tr2::sys::directory_iterator(folder); fit != tr2::sys::directory_iterator(); ++fit) {
				if ((string(fit->path()).find("_features") != string::npos) || (string(fit->path()).find("_faces") != string::npos) || tr2::sys::is_directory(fit->path())) continue;
				cout << "\tPlaying \"" << fit->path() << "\"..." << endl;
				vector<Skeleton> gesture = Skeleton::gestureFromCSV(fit->path());
				vector<Face> face_gest;
				try { 
					string pth = string(fit->path());
					pth = pth.substr(0, pth.find(".csv"));
					pth += "_faces.csv";
					face_gest = Face::faceGestureFromCSV(pth); 
				}
				catch (std::exception ex) {}
				gestview.playGesture(gesture, face_gest, enableControl, false); // Will return false in the last gesture
				if (!enableControl) Sleep(GESTUREDELAY); // To make a little space
			}
		}
	}
	gestview.closeWindow();
}

void setConsolePosition(BodyRGBViewer& body_view) {
	if (!body_view.isRunning()) throw std::exception("Cannot compute size if BodyViewer is not running!");
	std::array<int, 2> viewR;
	bool wpos = body_view.getWindowSize(viewR[0], viewR[1]);
	if (!wpos) viewR = { { 976, 647 } };
	HWND console = GetConsoleWindow();
	RECT r;
	GetWindowRect(console, &r); //stores the console's current dimensions
	SetWindowPos(console, HWND_NOTOPMOST, 0, viewR[1], r.right - r.left + 150, r.bottom - r.top - 95, /*SWP_NOSIZE | */SWP_NOZORDER);
}

int main(int argc, _TCHAR* argv[])
{
	// Prepare environment
	Kinect2Utils ku;
	HRESULT hr = ku.initDefaultKinectSensor(true);
	if (!SUCCEEDED(hr)) return -1;

	hr = ku.openBodyFrameReader();
	hr = ku.openFaceFrameReader(c_FaceFrameFeatures);
	BodyRGBViewer view(&ku);
	thread iface = view.RunThreaded(0, true, false);
	setConsolePosition(view);

	vector<vector<vector<Skeleton>>> gestures; // Recorded gestures for each gesture type
	vector<vector<vector<Face>>> facial_gestures; // Recorded facial gesture for each gesture type
	vector<string> gestureNames;
	future<pair<vector<Skeleton>, vector<Face>>> recorder;

	// Main loop
	bool playStored = false; // Play ALL stored gestures...
	bool controlPlaying = false;//Control the player
	string gestName;
	cout << "Write the first gesture name: ";
	cin >> gestName;
	gestureNames.push_back(gestName);
	gestures.push_back(vector<vector<Skeleton>>());
	facial_gestures.push_back(vector<vector<Face>>());
	createFolders(BASEPATH + gestName);
	int id = getNextGestureIdFromFile(gestName);
	printHelp();
	int option;
	do {
		cout << "Write an option (8 for help): ";
		cin >> option;

		if (option == 1) { //Start recording
			Sleep(PRE_RECORD_SLEEP); // Wait N seconds to record
			cout << "\tRecording gesture of type \"" << gestName << "\"..." << endl << endl;
			mtx.lock();
			recording = true;
			mtx.unlock();

			// Async call
			recorder = async(recordGesture, &ku, &view);
		}
		else if (option == 2) { // Stop recording
			if (!recorder.valid()) {
				cout << "\tNo gesture was being recorded!" << endl << endl;
			}
			else {
				mtx.lock();
				recording = false;
				mtx.unlock();
				pair<vector<Skeleton>, vector<Face>> gesture = recorder.get();
				string gesturePath = BASEPATH + gestName + "/" + gestName + to_string(id) + ".csv";
				Skeleton::gestureToCSV(gesture.first, gesturePath);
				Face::faceGestureToCSV(gesture.second, BASEPATH + gestName + "/" + gestName + to_string(id) + "_faces.csv");
				string featurePath = BASEPATH + gestName + "/" + gestName + to_string(id) + "_features.csv";
				Skeleton::gestureFeaturesToCSV(gesture.first, featurePath);
				gestures[gestures.size() - 1].push_back(gesture.first);
				facial_gestures[facial_gestures.size() - 1].push_back(gesture.second);
				cout << "\tEnd of record. File written in " << gesturePath << ". "<< endl << endl;
				++id;
			}
		}
		else if (option == 3) {
			if (recording) {
				cout << "\tYou have to finish the recording of the current gesture! (Option 2)" << endl << endl;
				continue;
			}
			cout << "\tWrite the new gesture name: ";
			cin >> gestName;
			cout << endl;
			gestureNames.push_back(gestName);
			gestures.push_back(vector<vector<Skeleton>>());
			facial_gestures.push_back(vector<vector<Face>>());
			createFolders(BASEPATH + gestName);
			id = getNextGestureIdFromFile(gestName);
		}
		else if (option == 4) {
			if (recording) {
				cout << "\tYou have to finish the recording of the current gesture! (Option 2)" << endl << endl;
				continue;
			}
			gestName = "";
			break;
		}
		else if (option == 5) {
			if (recording) {
				cout << "\tYou have to finish the recording of the gesture! (Option 2)" << endl << endl;
				continue;
			}
			gestName = "";
			playStored = true;
			break;
		}
		else if (option == 6) {
			if (recording) {
				cout << "\tYou have to finish the recording of the gesture! (Option 2)" << endl << endl;
				continue;
			}
			cout << "\tWrite the name of gesture you want to play: ";
			cin >> gestName;
			cout << endl;
			playStored = true;
			break;
		}
		else if (option == 7) {
			controlPlaying = !controlPlaying;
			cout << "\tNow the player controls are " << (controlPlaying? "enabled." : "disabled.") << endl << endl;
		}
		else printHelp();
	} while (true);

	view.closeWindow();
	iface.join();

	// Play gestures
	BodyRGBViewer gestview = BodyRGBViewer(&ku);
	if (!playStored) {
		cout << endl << "Recorded gestures are going to be played: " << endl;
		for (int i = 0; i < gestureNames.size(); ++i) {
			cout << "\tPlaying the " << gestures[i].size() << " recorded gestures of type \"" << gestureNames[i] << "\"." << endl;
			for (int j = 0; j < gestures[i].size(); ++j) {
				gestview.playGesture(gestures[i][j], facial_gestures[i][j], controlPlaying, i + 1 == gestureNames.size() && j + 1 == gestures[i].size()); // Will return false in the last gesture
				Sleep(GESTUREDELAY); // To make a little space
			}
		}
	}
	else playStoredGestures(gestview, gestName, controlPlaying);
	cout << "The recording program will now close!" << endl;

	Sleep(1000);
	return 0;
}