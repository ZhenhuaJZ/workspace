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

using namespace std;

deque<cv::Mat> mat;

cv_bridge::CvImagePtr ptr;

struct goal_pose
{
    int x = 0;
    int y = 0;
    double yaw = 0;
    double dist = 999;
};

struct currentPos
{
    int x = 0;
    int y = 0;
    double yaw = 0;
};

currentPos currentPose;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // if the image is color then choose it as color, else mono color
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        else
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    mat.push_back(ptr->image);
    if(mat.size())
    {
        cv::imshow("window", mat.front());
        cv::waitKey(5);
    }

    // calculate row frontier cells
    cv::Mat map = mat.front();
    cv::Mat frontier_map = map.clone();

    frontier_map = 255;
    goal_pose ogGoalCoodi;
    int min_dist = 9999;
    // Find column frontier cell
    for(int i = 0; i < map.rows; i++)
    {
        int prev = 0;
        for(int j = 0; j < map.cols; j++)
        {
            int current = map.at<uchar>(i,j);
            if((prev != current) && current != 0 && prev != 0)
            {
                frontier_map.at<uchar>(i,j) = 0;
                double dist = sqrt(pow((100-i),2) + pow((100-j),2));
//                std::cout << "distance:" << dist << std::endl;
                if(dist < min_dist)
                {
                    ogGoalCoodi.x = j;
                    ogGoalCoodi.y = i;
                    ogGoalCoodi.dist = dist;
                    min_dist = dist;
                }
            }
            prev = current;
        }
    }

//     Find rows frontier cell
    for(int j = 0; j < map.cols; j++)
    {
        int prev = 0;
        for(int i = 0; i < map.rows; i++)
        {
            int current = map.at<uchar>(i,j);
            if((prev != current) && current != 0 && prev != 0)
            {
                frontier_map.at<uchar>(i,j) = 0;
                double dist = sqrt(pow((100-i),2) + pow((100-j),2));
                if(dist < min_dist)
                {
                    ogGoalCoodi.x = j;
                    ogGoalCoodi.y = i;
                    ogGoalCoodi.dist = dist;
                    min_dist = dist;
                }
            }
            prev = current;
        }
    }

//    double yaw = atan2(100-gPose.y,100-gPose.x);
    std::cout << "og map goal coordinate x:" << ogGoalCoodi.x << " y:" << ogGoalCoodi.y <<  std::endl;
    cv::Point pt;
    pt.y = ogGoalCoodi.y;
    pt.x = ogGoalCoodi.x;
    cv::Point robotPos;

    goal_pose goalPose;
    goalPose.x = ogGoalCoodi.x - 100;
    goalPose.y = 100 - ogGoalCoodi.y;
    goalPose.yaw = atan2(goalPose.y,goalPose.x);
    std::cout << "(robot reference) goal pose x:"<< goalPose.x << " y:" << goalPose.y << " yaw:" << goalPose.yaw << "," << goalPose.yaw * 180/3.1415 << std::endl;



    robotPos.x = 100;
    robotPos.y = 100;
    cv::circle(frontier_map, pt, 10, 0,1);
    cv::circle(frontier_map,robotPos,3,0,3);
    cv::imshow("frontier_map", frontier_map);
    cv::waitKey(5);
    // modify cells in image
    if(mat.size()>2)
    {
        mat.pop_front();
    }
}

void odomCallBack(const nav_msgs::OdometryConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    currentPose.x = pose.position.x;
    currentPose.y = pose.position.y;
    currentPose.yaw = tf::getYaw(pose.orientation);
    cout << "odom returned attributes: x = " << currentPose.x << ", y = " << currentPose.y << ", yaw = " << currentPose.yaw << endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "frontier_exploration");

    ros::NodeHandle nh;
    std::cout << "node created" << std::endl;
    image_transport::ImageTransport tsp(nh);
    std::cout << "ImageTransport channel opened" << std::endl;

    image_transport::Subscriber sub = tsp.subscribe("map_image/full", 1, imageCallback);
    std::cout << "map_image/full subscribed" << std::endl;

    ros::Subscriber sub2 = nh.subscribe("odom", 1000, odomCallBack);
    std::cout << "odom subscribed" << std::endl;

    ros::spin();

    ros::shutdown();

    cv::destroyAllWindows();

    std::cout << "quack" << std::endl;
}
