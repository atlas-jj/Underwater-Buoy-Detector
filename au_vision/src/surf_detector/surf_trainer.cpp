#include <ros/ros.h>

#include <stdio.h>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <au_vision/surf_detector.h>

using namespace std;
using namespace cv;
using namespace au_vision;

const cv::Scalar GREEN(0, 255, 0);

SurfDetector* surfDetector;
string destinationFolder;

//Prototypes
bool confirm(string message, bool dflt);
void save(vector<float>& d, vector<float>& r);
void shutdown();

int main(int argc, char** argv) {
	ros::init(argc, argv, "surf_trainer");

	if (argc != 3) {
		cout << "Usage: rosrun au_vision video_surf_trainer [video/file/path] [destination/folder]" << endl;
		return -1;
	}

	VideoCapture vstream(argv[1]);
    destinationFolder = argv[2];

	if(vstream.isOpened())
		cout << "Reading video..." << endl;
	else {
		cout << "Error: video not found" << endl;
		return -1;
	}

	cv::namedWindow("Training", cv::WINDOW_NORMAL);

    ros::NodeHandle nh, private_nh;
	vector<float> descriptors, responses;

    surfDetector = new SurfDetector(nh, private_nh, "");

	while(vstream.grab()) {
		vector<float> frame_descriptors;
		vector<KeyPoint> frame_keyPoints;
		Mat frame, gray_frame;

		vstream.retrieve(frame);

		//Get the frame descriptors
		cv::cvtColor(frame, gray_frame, CV_BGR2GRAY);
		surfDetector->surf(gray_frame, Mat(), frame_keyPoints, frame_descriptors);

        cout << "Ready for input..." << endl;
        
		for(vector<KeyPoint>::iterator kp = frame_keyPoints.begin(); kp != frame_keyPoints.end(); kp++) {
			Mat kp_frame = frame.clone();

			cv::circle(kp_frame, kp->pt, 12, GREEN, 3);

			cv::imshow("Training", kp_frame);

			while(true) {
				char key = cv::waitKey(10);

				if(key == 'y') {
					responses.push_back(1);
					cout << responses.size() << " - Marked yes" << endl;
					break;
				}

				if(key == 'n') {
					responses.push_back(-1);
					cout << responses.size() << " - Marked no" << endl;
					break;
				}

				if(key == 'b' && kp != frame_keyPoints.begin()) {
					//Works as long as you haven't just changed the frame
					responses.pop_back();
					kp = kp - 2;
					break;
				}

				if(key == 's' && kp == frame_keyPoints.begin()) {
                    bool yes = confirm("You are exiting early (saving)... are you sure you want to continue?", false);

                    if(yes) {
                        save(descriptors, responses);
                        shutdown();
                        return 0;
                    }
				}

				if(key == 'q') {
                    bool yes = confirm("You are exiting without saving... are you sure you want to continue?", false);
                    
                    if(yes) {
                        shutdown();
                        return 0;
                    }
				}
			}

		} //for each keypoint

		//Append frame_descriptors onto descriptors
		descriptors.insert(descriptors.end(), frame_descriptors.begin(), frame_descriptors.end());

	} // for each frame
	
	save(descriptors, responses);
	shutdown();
	return 0;
}

bool confirm(string message, bool dflt) {
    string confirmation;
    string dflt_hint = dflt ? "[(y) or n]" : "[y or (n)]";

    cout << message << " " << dflt_hint << endl << "Response: ";
    cin >> confirmation;
    cout << endl;

    if((dflt && confirmation == "n") || (!dflt && confirmation == "y")) {
        return !dflt;
    }
    else {
        return dflt;
    }
}

void save(vector<float>& d, vector<float>& responses) {
	Mat t = surfDetector->descriptorsToMat(d);
	Mat r(responses.size(), 1, CV_32FC1, &responses[0]);
    string destination, filename, type;
	
    cout << "Saving data..." << endl;

	cout << "Please enter the type of object you have identified (ex. \'red buoy\'): ";
	cin >> type;

    cout << endl << "Please enter file name (no extension): ";
    cin >> filename;

    destination = destinationFolder + "/" + filename + ".yml";

    cout << endl << "Destination registered: " << destination << endl;
    
    bool confirmed = confirm("Confirm?", true);

    if (!confirmed) {
        cout << endl << "Enter full destination path: ";
        destination = "";
        cin >> destination;
    }

	//Save training data
	FileStorage fs(destination, FileStorage::WRITE);
	fs << "type" << type << "data" << t << "responses" << r;
}

void shutdown() {
	cout << "Shutting Down..." << endl;

    delete surfDetector;

	cv::destroyWindow("Training");
	ros::shutdown();
}