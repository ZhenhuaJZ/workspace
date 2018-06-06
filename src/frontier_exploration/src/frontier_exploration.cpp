#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <deque>

using namespace std;

deque<cv::Mat> mat;

cv_bridge::CvImagePtr ptr;

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
        std::cout << "callback" << std::endl;
        cv::imshow("window", mat.front());
        cv::waitKey(30);
    }
    if(mat.size()>2)
    {
        mat.pop_front();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "frontier_exploration");

    ros::NodeHandle nh;
    std::cout << "node created" << std::endl;
    image_transport::ImageTransport tsp(nh);

    image_transport::Subscriber sub = tsp.subscribe("map_image/full", 1, imageCallback);
    std::cout << "subed" << std::endl;

    ros::spin();

    ros::shutdown();

    cv::destroyAllWindows();

    std::cout << "quack" << std::endl;
}
