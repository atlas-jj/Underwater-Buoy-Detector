/**
 * @file test_action_client.cpp
 * @author Sean Scheideman
 * @date 3 Dec 2016
 * @brief Action client for testing detector_action_server
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <au_vision/DetectObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

class TestClient
{
public:
    TestClient() : ac("object_detection_server", true), it(nh)
    {
        sub = it.subscribe("/front/camera/image_raw", 1, &TestClient::imageCallback, this);
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, waiting for images.");
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        au_vision::DetectObjectsGoal goal;
        goal.inputFrame = *msg;
        ac.sendGoal(goal);
    }

private:
    actionlib::SimpleActionClient<au_vision::DetectObjectsAction> ac;
    image_transport::Subscriber sub;
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_action_client");
    TestClient testClient;
    ros::spin();
    return 0;
}
