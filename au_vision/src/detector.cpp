/**
 * @file detector.cpp
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Implementation for the Detector class
 *
 */

#include <au_vision/detector.h>

namespace au_vision
{
    Detector::Detector(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh), detectorType_("")
    {
        // create publishers and subscribers
        initializeIo();

        ROS_INFO_STREAM( "Waiting for input frames" );
    }

    Detector::~Detector()
    {
        // shutdown publishers and subscribers
        debugImagePub_.shutdown();
    }

    void Detector::initializeIo()
    {
        // input image subscriber
        image_transport::ImageTransport it(private_nh_);

        // output debug image publisher
        debugImagePub_ = it.advertise( "debug", 10 );
    }

	void Detector::debugDraw( cv::Mat &frame, std::vector<au_core::Roi> roiArray )
	{
		outputImage_ = frame.clone();

        if(roiArray.size() > 0)
        {
            for(int i = 0 ; i < roiArray.size() ; i++)
            {
                au_core::Roi roi = roiArray[i];
                cv::Rect rect(roi.xOffset,roi.yOffset,roi.width,roi.height);
                cv::rectangle( outputImage_, rect, cv::Scalar(0,0,255), 2, CV_AA );
                putText( outputImage_, roi.type, cv::Point(roi.xOffset,roi.yOffset), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, CV_AA);
            }
        }
		else
		{
			putText( outputImage_, "Not roi's detected", cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, CV_AA);
		}
	}

    bool Detector::convertToMat( const sensor_msgs::Image &frame, cv::Mat &output )
    {
        /*
         * convert to opencv image
         */
        try
        {
            cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy( frame, sensor_msgs::image_encodings::RGB8 );
            output = cvPtr->image;
        }
        catch( cv_bridge::Exception& e )
        {
            // if conversion didnt work, try a few image formats
            try
            {
                cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(frame);
                if( frame.encoding == "CV_8UC3" )
                { // assuming rgb
                    output = cvPtr->image;
                }
                else if( frame.encoding == "8UC1" )
                { // greyscale image
                    cv::cvtColor( cvPtr->image, output, CV_GRAY2RGB );
                }
                else
                {
                    ROS_ERROR( "Detector failed while trying to convert image from '%s' to 'rgb8' (%s)", frame.encoding.c_str(), e.what() );
                    return false;
                }
            }
            catch( cv_bridge::Exception& e )
            {
                ROS_ERROR( "Detector failed while trying to convert image from '%s' to 'rgb8' (%s)", frame.encoding.c_str(), e.what() );
                return false;
            }
        }
        return true;
    }

    bool Detector::inputImageDetectRequest(au_vision::DetectObjects::Request  &request,
             au_vision::DetectObjects::Response &response)
    {
        ROS_INFO("Received input array...");
        sensor_msgs::Image source = request.inputFrame;


        if(!convertToMat( source, inputImage_ ))
        {
            return false;
        }

        roiArray_ = detect( inputImage_ );

        //output debug image and ROI
	    debugDraw( inputImage_, roiArray_ );

        // publish output image
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "detector";
        cvBridgeImage_.header = header;
        cvBridgeImage_.encoding = sensor_msgs::image_encodings::RGB8;
        cvBridgeImage_.image = outputImage_;
        debugImagePub_.publish( cvBridgeImage_.toImageMsg() );

        ROS_INFO_STREAM( "Number of ROI's detected: " << roiArray_.size() );
        // publish ROI
	    if( roiArray_.size() > 0 )
	    {
            response.roiArray.regionsOfInterest = roiArray_;
		    return true;
	    }else{

            return false;
        }
    }

    std::vector<au_core::Roi> Detector::inputImageDetectAction( const sensor_msgs::Image& frame )
    {

        std::vector<au_core::Roi> roiArray;

        if(!convertToMat( frame, inputImage_ ))
        {
            return roiArray;
        }

        roiArray = detect( inputImage_ );

        //output debug image and ROI
	    debugDraw( inputImage_, roiArray );

        // publish output image
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "detector";
        cvBridgeImage_.header = header;
        cvBridgeImage_.encoding = sensor_msgs::image_encodings::RGB8;
        cvBridgeImage_.image = outputImage_;
        debugImagePub_.publish( cvBridgeImage_.toImageMsg() );

        ROS_INFO_STREAM( "Number of ROI's detected: " << roiArray.size() );
        return roiArray;
    }
} // end of namespace
