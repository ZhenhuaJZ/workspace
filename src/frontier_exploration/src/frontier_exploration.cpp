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
    int dist = 999;
};

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
    goal_pose gPose;
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
                int dist = sqrt(pow((100-i),2) + pow((100-j),2));
                std::cout << "distance:" << dist << std::endl;
                if(dist < min_dist)
                {
                    gPose.x = i;
                    gPose.y = j;
                    gPose.dist = dist;
                    min_dist = dist;
                }
            }
            prev = current;
        }
    }

//     Find column frontier cell
    for(int j = 0; j < map.rows; j++)
    {
        int prev = 0;
        for(int i = 0; i < map.cols; i++)
        {
            int current = map.at<uchar>(i,j);
            if((prev != current) && current != 0 && prev != 0)
            {
                frontier_map.at<uchar>(i,j) = 0;
                int dist = sqrt(pow((100-i),2) + pow((100-j),2));
                if(dist < min_dist)
                {
                    gPose.x = i;
                    gPose.y = j;
                    gPose.dist = dist;
                    min_dist = dist;
                }
            }
            prev = current;
        }
    }

    std::cout << "goal pose x:" << gPose.x << " y:" << gPose.y << std::endl;
    cv::Point pt;
    pt.y = gPose.x;
    pt.x = gPose.y;
    cv::circle(frontier_map, pt, 10, 0,2);
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
    double x = pose.position.x;
    double y = pose.position.y;
    double yaw = tf::getYaw(pose.orientation);
//    cout << "odom returned attributes: x = " << x << ", y = " << y << ", yaw = " << yaw << endl;
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
