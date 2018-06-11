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

/*!
 * \brief The FrontierExploration class receives OgMap and odom informations and
 * calculate the frontier cell base on the OgMap, hence compute a goal pose which
 * the robot will head to.
 */

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

    std::deque<OgPose> frontierCells_;

public:
    /*!
     * \brief FrontierExploration takes a node handle and subscribe and advertises topics
     * \param node ros node handler
     */
    FrontierExploration(ros::NodeHandle node);

    /*!
     * \brief FrontierExploration destructor
     */
    ~FrontierExploration();

    /*!
     * \brief odomCallBack receive topic message from the subscribed topic.
     * This function subscribed to topic "odom"
     * \param msg odom message receive from topic
     */
    void odomCallBack(const nav_msgs::OdometryConstPtr& msg);

    /*!
     * \brief imageCallback receives topic message from the subscribed topic.
     * imageCallback is subscribed to topic "map_image/full"
     * \param msg image message receive from topic
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /*!
     * \brief processFrontier is a function to process all the frontier cells.
     * It compute and display the image of fontier cell
     */
    void processFrontier();

    /*!
     * \brief computeFontierCell receives the ogMap obtain from imageCallback and compute all the frontier cells.
     * All the frontier cells are stored inside a container.
     * \param ogMap Mono image containing information of the map seen by robot
     * \return Goal pose in reference to the ogMap
     */
    OgPose computeFontierCell(cv::Mat ogMap);
    /*!
     * \brief calculateGoalPose calculates the goal x y and yaw pose in reference the robot
     */
    void calculateGoalPose();
};

#endif
