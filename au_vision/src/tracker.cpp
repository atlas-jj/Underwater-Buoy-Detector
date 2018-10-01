/**
 * @file tracker.cpp
 * @author Rumman Waqar
 * @date 2 Sep 2015
 * @brief Implementation for the Tracker class
 *
 */

#include <au_vision/tracker.h>

namespace au_vision
{

    Tracker::Tracker(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh), trackerType_(""), isTracking_(false)
    {
        // create publishers and subscribers
        initializeIo();

        ROS_INFO_STREAM( "Tracker has not been initialized yet. Waiting for ROI" );
    }

    Tracker::~Tracker()
    {
        // shutdown publishers and subscribers
        inputImageSub_.shutdown();
        roiSubscriber_.shutdown();
        roiPublisher_.shutdown();
        debugImagePub_.shutdown();
    }

    void Tracker::initializeIo()
    {
        // input image subscriber
        image_transport::ImageTransport it(private_nh_);
        inputImageSub_ = it.subscribe( "camera", 10, &Tracker::inputImageCallback, this );

        // input ROI subscriber
        roiSubscriber_ = private_nh_.subscribe( "inputRoi", 10, &Tracker::roiCallback, this );

        // output debug image publisher
        debugImagePub_ = it.advertise( "debug", 10 );

        // output ROI publisher
        roiPublisher_ = private_nh_.advertise<au_core::Roi>( "outputRoi", 10 );
    }

	void Tracker::debugDraw( cv::Mat &frame, cv::Rect roi )
	{
		outputImage_ = frame.clone();
		if( isTracking_ )
		{
			cv::rectangle( outputImage_, trackedWindow_, cv::Scalar(0,0,255), 2, CV_AA );
		}
		else
		{
			putText( outputImage_, "Not tracking", cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, CV_AA);
		}
	}

    void Tracker::inputImageCallback( const sensor_msgs::ImageConstPtr& msg )
    {
        /*
         * convert to opencv image
         */
        try
        {
            cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::RGB8 );
            inputImage_ = cvPtr->image;
        }
        catch( cv_bridge::Exception& e )
        {
            // if conversion didnt work, try a few image formats
            try
            {
                cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(msg);
                if( msg->encoding == "CV_8UC3" )
                { // assuming rgb
                    inputImage_ = cvPtr->image;
                }
                else if( msg->encoding == "8UC1" )
                { // greyscale image
                    cv::cvtColor( cvPtr->image, inputImage_, CV_GRAY2RGB );
                }
                else
                {
                    ROS_ERROR( "Tracker.inputImageCallback() while trying to convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what() );
                    return;
                }
            }
            catch( cv_bridge::Exception& e )
            {
                ROS_ERROR( "Tracker.inputImageCallback() while trying to convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what() );
                return;
            }
        }

        /*
         * process image
         */
        if( !isTracking_ )
        {
	        ROS_WARN_STREAM_THROTTLE( 2.0, "Received image but tracker is not initialized!" );
        }
	    else
        {
	        trackedWindow_ = track( inputImage_ );
	        if( not trackedWindow_.area() )
		        isTracking_ = false;
        }

        /*
         * output debug image and ROI
         */
	    debugDraw( inputImage_, trackedWindow_ );

        // publish output image
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "tracker";
        cvBridgeImage_.header = header;
        cvBridgeImage_.encoding = sensor_msgs::image_encodings::RGB8;
        cvBridgeImage_.image = outputImage_;
        debugImagePub_.publish( cvBridgeImage_.toImageMsg() );

        // publish ROI
	    if( isTracking_ )
	    {

		    au_core::Roi roi;
		    roi.xOffset = (uint)trackedWindow_.x;
		    roi.yOffset = (uint)trackedWindow_.y;
		    roi.width = (uint)trackedWindow_.width;
		    roi.height = (uint)trackedWindow_.height;
		    roiPublisher_.publish( roi );
	    }

    }

    void Tracker::roiCallback( const au_core::RoiConstPtr msg )
    {
	    trackedWindow_ = cv::Rect( msg->xOffset, msg->yOffset, msg->width, msg->height );
        isTracking_ = initializeTracker( inputImage_, trackedWindow_ );

	    if( initializeTracker( inputImage_, trackedWindow_ ) )
	    {
		    isTracking_ = true;
		    ROS_INFO( "Tracker initialized to new ROI." );
	    }
	    else
	    {
		    ROS_WARN( "Tracker failed to initialized." );
	    }
    }

} // end of namespace
