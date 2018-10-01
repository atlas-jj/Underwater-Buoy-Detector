
#include <au_vision/surf_detector.h>

using namespace cv;

namespace au_vision {

/*
*	Public Methods
*/

SurfDetector::SurfDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh, std::string train_path = "") :
	Detector(nh, private_nh),
	nh_(nh), private_nh_(private_nh),
	K(40), is_trained_(false), minOrientation(0), maxOrientation(360), featureType("unknown")
{
	//Enable non-free module
	ROS_INFO("Initializing opencv-nonfree...");
	initModule_nonfree();

	//Customize the SURF Detector
	surf.extended = 0; //basic 64-floating point descriptors
	surf.upright = 1; //enable U_SURF
	surf.hessianThreshold = 400; //TODO: add tracker bar for this value (300-500)

	//Initialize the SVM parameters
	svm_params_.svm_type = CvSVM::C_SVC;
	svm_params_.kernel_type = CvSVM::POLY;
	svm_params_.degree = 2;
	svm_params_.gamma = 1;
	svm_params_.coef0 = 0;
	svm_params_.term_crit = TermCriteria(TermCriteria::COUNT, 100, 1e-6);

	//Setup the Publisher
	pub_ = VisionBridge::createPublisher(private_nh_, "point_overlay", 1);

	//Use conventional video topic by default
	setupSubscriber("/front/camera/image_raw");

	//Train the svm
	if (train_path != "") {
		train(train_path);
	}
	else {
		ROS_WARN("No training data provided, SurfDetector classification will not work!");
	}
}

SurfDetector::~SurfDetector()
{
	sub_.shutdown();
	pub_.shutdown();
}

void SurfDetector::setupSubscriber(std::string topic) 
{
	sub_ = private_nh_.subscribe(topic, 1, &SurfDetector::frameCallback, this);
}

std::vector<float> SurfDetector::getResponses(const std::vector<float>& descriptors)
{
	std::vector<float> responses, center_responses;
	Mat data, centers, labels;
	data = descriptorsToMat(descriptors);

	cluster(data, centers, labels);

	//Flatten clusters to 1D array
	float* array = (float *) labels.data;
	std::vector<float> clusters(array, array + sizeof array / sizeof array[0]);

	for (int i = 0; i < centers.rows; i++) {
		float resp = classify(centers.row(i));
		center_responses.push_back(resp);
	}

	//Map responses
	for (std::vector<float>::iterator it = clusters.begin(); it != clusters.end(); it++) {
		responses.push_back(center_responses[*it]);
	}

	return responses;
}

void SurfDetector::cluster(const Mat& descriptors, Mat& centers, Mat& clusters)
{
	TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1);
	double compactness = 0;
	int clusterCount;

	clusterCount = std::min(descriptors.rows, K);

	compactness = kmeans(descriptors, clusterCount, clusters, criteria, 5, KMEANS_PP_CENTERS, centers);
}

float SurfDetector::classify(const Mat& descriptor)
{
	if (is_trained_) {
		return svm.predict(descriptor);
	}
	else {
		ROS_ERROR("You cannot classify features when the SVM is not trained!");
		return -1;
	}
}

Mat SurfDetector::descriptorsToMat(const std::vector<float>& descriptors)
{
	//Convert basic decriptors into an n by 64 matrix for kmeans
	int n = descriptors.size() / 64;
	Mat output(n, 64, CV_32FC1);

	for (size_t i = 0; i < n; i++) {
	    for (size_t j = 0; j < 64; j++) {
	        output.at<float>(i, j) = descriptors[(i * 64) + j];
	    }
	}

	return output;
}

void SurfDetector::train(const std::string train_path) 
{
	Mat trainingData, responses;
	FileStorage fs(train_path, FileStorage::READ);

	if( fs.isOpened() ) {
		
		if(!fs["type"].isNone()) {
			fs["type"] >> featureType;
		}

		if(!fs["data"].isNone() && !fs["responses"].isNone()) {
			fs["data"] >> trainingData;
			fs["responses"] >> responses;
			train(trainingData, responses);
		}
		else {
			ROS_ERROR("Training file does not have required \'data\' or \'responses\' section... not training");
		}
	}
	else {
		ROS_ERROR_STREAM("Could not open training file at " << train_path);
	}

	fs.release();
}

void SurfDetector::train(const Mat& training_data, const Mat& responses)
{
	ROS_INFO("Training SVM...");
	svm.train(training_data, responses, Mat(), Mat(), svm_params_);
	is_trained_ = true;
}

/*
*	Protected Methods
*/

std::vector<au_core::Roi> SurfDetector::detect(const cv::Mat &frame)
{
	if(!is_trained_) {
		ROS_ERROR("Cannot detect when SVM is untrained!");
	}

    std::vector<au_core::Roi> roiArray;
	std::vector<KeyPoint> keyPoints;
	std::vector<float> descriptors, responses;
	Mat gray_frame;

	//Convert to gray-scale
	cvtColor(frame, gray_frame, CV_RGB2GRAY);

	surf(gray_frame, Mat(), keyPoints, descriptors);

	if (descriptors.size() != 0) {
		responses = getResponses(descriptors);

		vector<float>::iterator r = responses.begin();
		for(vector<KeyPoint>::iterator kp = keyPoints.begin(); kp != keyPoints.end(); kp++) {

			if (*r == 1 && hasValidOrientation(kp->angle)) {
				au_core::Roi roi;

				roi.xOffset = kp->pt.x - (kp->size / 2);
				roi.yOffset = kp->pt.y - (kp->size / 2);
				roi.width = kp->size;
				roi.height = kp->size;
				roi.type = featureType;

				roiArray.push_back(roi);
			}

			r++;
		}
	}

	return roiArray;
}

/*
*	Private Methods
*/

void SurfDetector::frameCallback(const sensor_msgs::ImageConstPtr& msg) 
{
	cv::Mat bgr_frame, frame, mask;
	vector<cv::KeyPoint> keyPoints;
	vector<cv::Mat> hsv;
	vector<float> descriptors, responses;

	bgr_frame = VisionBridge::toCvMat(msg);

	int thickness = floor(sqrt(bgr_frame.size().area()) / 240);

	//Convert to desired color space
	cv::cvtColor(bgr_frame, frame, CV_BGR2HSV);
	cv::split(frame, hsv);
	frame = hsv[0];

	//Get the key points
	surf(frame, mask, keyPoints, descriptors);

	if (descriptors.size() != 0) {

		for(vector<cv::KeyPoint>::iterator kp = keyPoints.begin(); kp != keyPoints.end(); kp++) {
			cv::circle(bgr_frame, kp->pt, floor(kp->size / 2), cv::Scalar(81, 237, 28), thickness);
		}

	}

	pub_.publish(VisionBridge::toImgMsg(bgr_frame));
}

bool SurfDetector::hasValidOrientation(float o)
{
	if (o == -1) //not using U_SURF
		return true;
	else if (minOrientation <= maxOrientation)
		return o >= minOrientation && o <= maxOrientation;
	else
		return (o < 360 && o > minOrientation) || (o >= 0 && o < maxOrientation);
}

} //namespace au_vision
