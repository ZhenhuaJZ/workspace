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
#include "../../devel/include/frontier_exploration/RequestGoal.h"

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

//    struct Response
//    {
//        bool ack;
//    };

//    struct Request
//    {
//        double x;
//        double y;
//        double yaw;
//    };

private:
    ros::NodeHandle nodeHandle_; //!< Node handle of frontier node
    image_transport::ImageTransport imgTrans_; //!< image tranport object
    image_transport::Subscriber imgTransSub_; //!<
    ros::Subscriber odomSub_; //!< Ros subscriber for odom topic
    ros::Subscriber pathSub_; //!< Ros subscriber for path topic
    cv_bridge::CvImagePtr cvImgPtr_; //!< cv image pointer to transport image with ros
    cv_bridge::CvImage cvImageFbe_; //!< CvImage to transport image with ros
    image_transport::Publisher imageFbePublisher_; //!< publisher to publish image_fbe topic
    ros::ServiceServer service_; //!< ros service server
    ros::ServiceClient client_;

    cv::Mat frontierMap_; //!< Stores map containing frontier, goal cell, and frontier cells seen by goal
    cv::Mat OgMap_; //!< OgMap receive from map_image/full topic
    double resolution_; //!< Resolution for each cell

    /*!
     * \brief The ImageDataBuffer struct contain deque container containing Mat image and mutex lock
     */
    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imgBuffer;
        std::mutex imgMtx;
    };
    ImageDataBuffer imgBuffer_; //!< ImageDataBuffer object

    /*!
     * \brief The PoseDataBuffer struct contain deque container containing Poses and mutex lock
     */
    struct PoseDataBuffer
    {
        std::deque<OgPose> buffer;
        std::mutex mtx;
    };
    PoseDataBuffer poseBuffer; //!< PoseDataBuffer object

    std::deque<OgPose> frontierCells_; //!< stores poses of frontier cells
    std::deque<OgPose> goalFrontierCells_; //!< stores poses of frontier cells seen by the goal pose
    OgPose ogMapReferenceGoalPose_; //!< Goal pose in reference of OgMap
    OgPose robotReferenceGoalPose_; //!< Goal pose in reference of the robot
    OgPose globalReferenceGoalPose_; //!< Goal pose in reference of global

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
     * \brief processFrontier is a function to process all the frontier cells. All the computation of the frontier cells , goal pose and frontier cells seen by goal pose are all inside this function.
     * The function also pulishes an image containing all the mentioned cells. color coding: Blue - Goal pose, Red - frontier cell seen by goal pose, Black - frontier cells
     */
    void processFrontier();

    /*!
     * \brief computeFontierCell receives the ogMap obtain from imageCallback and compute all the frontier cells.
     * The frontier cell is calculated base on iterator through each individual columns and rows.
     * If the current cell is different from previous cell and both current cell and previous cell are not black, then the white cell is the frontier cell.
     * All the frontier cells are stored inside a container.
     * \param ogMap Mono image containing information of the map seen by robot
     * \return Deque container of frontier cells
     */
    std::deque<OgPose> computeFontierCell(cv::Mat ogMap);

    /*!
     * \brief computeGoalPose compute the goal pose base on given frontier cells
     * \param frontierCells contains calculated frontier cells from OgMap
     * \return OgPose of goal pose
     */
    OgPose computeGoalPose(std::deque<OgPose> frontierCells);

    /*!
     * \brief calculateGoalPose calculates the goal x y and yaw pose in reference the robot.
     * The goal pose in reference to the robot is by calculate the ogMap coorindate and coordinate conversion, the calculated coordinate is in reference in the center of ogMap which is the
     * location of the robot. The yaw is calculated using atan2(y,x).
     */
    void calculateGoalPose();

    /*!
     * \brief getFrontierCells is a getter function to obtain the container containing frontier cells.
     * \return a deque container containing frontier cells
     */
    std::deque<OgPose> getFrontierCells();

    /*!
     * \brief getGoalFrontierCells is a getter function to obtain the container containing frontier cells seen by goal.
     * \return a deque container containin frontier cells seen by goal
     */
    std::deque<OgPose> getGoalFrontierCells();

    /*!
     * \brief computeFrontierAtGoal computes the frontier cells seen by goal base on compare if the cells in within range of field of view of the laser.
     * If the frontier cell is within range of field of view and is within laser distance then its an frontier cell seen by goal.
     * \param frontierCells containing calculated all the frontier cells of OgMap
     * \param goalPose is the goal pose
     * \return a deque container containing frontier cells seen by goal pose
     */
    std::deque<OgPose> computeFrontierAtGoal(std::deque<OgPose> frontierCells, OgPose goalPose);

    /*!
     * \brief pathCallback is the call back function when topic path is sending back array of pose informations.
     * \param msg send back by the path topic containing pose array.
     */
    void pathCallback(const geometry_msgs::PoseArrayConstPtr& msg);

    /*!
     * \brief requestGoal requests goal to other nodes that subscribed to the service.
     * \param req is a parameter sends the request to the service.
     * \param res is a parameter containing the response send by subscribed node.
     * \return bool variable indicating if the request is send and received a response.
     */
    bool requestGoal(frontier_exploration::RequestGoal::Request &req, frontier_exploration::RequestGoal::Response &res);
};
#endif
