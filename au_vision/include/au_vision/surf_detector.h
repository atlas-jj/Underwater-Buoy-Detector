/**
 * @file surf_detector.h
 * @author James Hryniw
 * @date 18 Feb 2016
 * @brief Header for the SurfDetector class
 *
 */

#ifndef SURF_DETECTOR_H
#define SURF_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <stdio.h>
#include <iostream>

#include <au_vision/detector.h>
#include <au_vision/vision_bridge.h>

using namespace cv;

namespace au_vision {

/**
* @class SurfDetector
* @ingroup au_vision
* @brief Manages specific detector, by handling all the inputs and outputs
*
* Implements the Detector class. If trained, detect(frame) returns a list of ROIs 
* containing the desired descriptors. 
* 
* 
* Calling setupSubscriber(topic_name) publishes frame with all SURF 
* features drawn to SurfDetector/point_overlay.
* 
*/

class SurfDetector : public Detector {

	public:
		SURF surf;
		CvSVM svm;
		int K;
		int minOrientation, maxOrientation;
		string featureType;
		
		//Constructors and destructors

		/**
		 * @breif Builds a new SurfDetector
		 * @param nh node handler
	 	 * @param private_nh node handler inside the private namespace of the node
		 * @param train_path path to yaml file containing training data
		 * 
		 * Initializes SURF library and creates the ROS publishers and subscribers. 
		 * Trains the SVM with yaml data contained at train_path. Training data must be of the format:
		 * 	 data: [array of descriptors]
		 * 	 responses: [array of correct responses]
		 *
		 */
		SurfDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh, std::string train_path);

		/**
		 * @breif Destroys the object and releases publishers and subscribers
		 */
		~SurfDetector();

		/**
		 * @breif Helper function to convert surf descriptors to a handleable OpenCV matrix
		 * @param descriptors one dimentional array of n length 64 descriptors
		 * @return nx64 Mat of descriptors
		 *
		 * Converts a one dimensional array of n length 64 descriptors to an nx64 OpenCV matrix
		 * TODO: handle length 128 descriptors
		 */
		static Mat descriptorsToMat(const std::vector<float>& descriptors);

		/**
		 * @breif Subscribes to frame topic
		 * @param topic video topic to subscribe from 
		 * 
		 * Publishes to /SurfDetector/point_overlay with keypoints drawn on received images
		 */
		void setupSubscriber(std::string topic);

		/**
		 * @breif Classifies descriptors
		 * @param one dimensional array of descriptors
		 * @return array of classifications for the input descriptors 
		 */
		std::vector<float> getResponses(const std::vector<float>& descriptors);

		/**
		 * @breif Clusters Descriptors into K bins
		 * @param[in] descriptors descriptors to cluster
		 * @param[out] centers array of bin labels for corresponding descriptors
		 * @param[out] clusters array of K descriptor bins
		 */
		void cluster(const Mat& descriptors, Mat& centers, Mat& clusters);

		/**
		 * @breif Classifies a descriptor
		 * @param descriptor descriptor to classify
		 * @return -1 for non match, 1 for match
		 */
		float classify(const Mat& descriptor);
		
		//SURF Detector Associator
		/**
		 * @breif Trains the Classifier (SVM)
		 * @param train_path path to yaml file containing training data
		 * 
		 * Trains the SVM with yaml data contained at train_path. Training data must be of the format:
		 * 	 data: [array of descriptors]
		 * 	 responses: [array of correct responses], -1 for non-match and 1 for match
		 */
		void train(const std::string train_path);

		/**
		 * @breif Trains the Classifier (SVM)
		 * @param training_data nx64 matrix of descriptors
		 * @param responses nx1 matrix of corresponding correct responses
		 */
		void train(const Mat& training_data, const Mat& responses);
		
		/**
		 * @breif Attempts to detect objects using SURF descriptors
		 * @param frame the frame to inspect
		 * @return array of ROIs containing the matched descriptors
		 */
		std::vector<au_core::Roi> detect( const cv::Mat& frame );

	private:
		CvSVMParams svm_params_;
		bool is_trained_;

		ros::NodeHandle nh_, private_nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		void frameCallback(const sensor_msgs::ImageConstPtr& msg);

		bool hasValidOrientation(float o);
};

} //namespace au_vision

#endif //SURF_DETECTOR_H
