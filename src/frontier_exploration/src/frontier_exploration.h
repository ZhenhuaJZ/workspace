#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <deque>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>

#define ROBOT_LOCATION_CELL_X 100
#define ROBOT_LOCATION_CELL_Y 100

/*!
 * \brief The FrontierExploration class receives OgMap and odom informations and
 * calculate the frontier cell base on the OgMap, hence compute a goal pose which
 * the robot will head to.
 */

class FrontierExploration
{
public:
    /*!
     * \brief The OgPose struct contains x y and yaw in double.
     */
    struct OgPose
    {
        double x;
        double y;
        double yaw;
    };
private:
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imgTrans_;
    image_transport::Subscriber imgTransSub_;
    ros::Subscriber odomSub_;
    ros::Subscriber pathSub_;
    cv_bridge::CvImagePtr cvImgPtr_;
    ros::ServiceClient client_;
    cv_bridge::CvImage cvImageFbe_;
    image_transport::Publisher imageFbePublisher_;


    cv::Mat frontierMap_;


    double resolution_;

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imgBuffer;
        std::mutex imgMtx;
    };
    ImageDataBuffer imgBuffer_;

    struct PoseDataBuffer
    {
        std::deque<OgPose> buffer;
        std::mutex mtx;
    };
    PoseDataBuffer poseBuffer;

    struct pathDataBuffer
    {
        std::deque<OgPose> buffer;
        std::mutex mtx;
    };

    std::deque<OgPose> frontierCells_;
    std::deque<OgPose> goalFrontierCells_;
    OgPose ogMapReferenceGoalPose_;
    OgPose robotReferenceGoalPose_;
    OgPose globalReferenceGoalPose_;

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
     * The frontier cell is calculated base on iterator through each individual columns and rows.
     * If the current cell is different from previous cell and both current cell and previous cell are not black, then the white cell is the frontier cell.
     * All the frontier cells are stored inside a container.
     * \param ogMap Mono image containing information of the map seen by robot
     * \return Goal pose in reference to the ogMap
     */
    OgPose computeFontierCell(cv::Mat ogMap);
    /*!
     * \brief calculateGoalPose calculates the goal x y and yaw pose in reference the robot.
     * The goal pose in reference to the robot is by calculate the ogMap coorindate and coordinate conversion, the calculated coordinate is in reference in the center of ogMap which is the
     * location of the robot. The yaw is calculated using atan2(y,x).
     */
    void calculateGoalPose();

    /*!
     * \brief getFrontierCells
     * \return
     */
    std::deque<OgPose> getFrontierCells();

    void computeFrontierAtGoal();

    void pathCallback(const geometry_msgs::PoseArrayConstPtr& msg);
};

#endif
