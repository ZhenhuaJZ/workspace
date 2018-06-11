#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <deque>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>

class FrontierExploration
{
private:
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imgTrans_;
    image_transport::Subscriber imgTransSub_;
    ros::Subscriber odomSub_;
    cv_bridge::CvImagePtr cvImgPtr_;

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imgBuffer;
        std::mutex imgMtx;
    };
    ImageDataBuffer imgBuffer_;

    struct OgPose
    {
        double x;
        double y;
        double yaw;
    };

    OgPose ogMapGoalPose_;
    OgPose robotGoalPose_;

    struct PoseDataBuffer
    {
        std::deque<OgPose> buffer;
        std::mutex mtx;
    };
    PoseDataBuffer poseBuffer;
    cv::Mat ogMap_;


public:
    FrontierExploration(ros::NodeHandle node);

    ~FrontierExploration();

    void odomCallBack(const nav_msgs::OdometryConstPtr& msg);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void calculateFrontier();

    void calculateGoalPose();
};

#endif
