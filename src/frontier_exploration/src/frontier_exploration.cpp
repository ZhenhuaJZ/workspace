#include "frontier_exploration.h"

using namespace std;

FrontierExploration::FrontierExploration(ros::NodeHandle node)
    : nodeHandle_(node), imgTrans_(node)
{
    imgTransSub_ = imgTrans_.subscribe("map_image/full", 1, &FrontierExploration::imageCallback,this);
    odomSub_ = nodeHandle_.subscribe("odom", 1000, &FrontierExploration::odomCallBack,this);
}


FrontierExploration::~FrontierExploration()
{
    cv::destroyAllWindows();
}

void FrontierExploration::calculateGoalPose()
{
    std::mutex mtx;
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lck(mtx);
        // Display goal coordinate in reference of OgMap
        std::cout << "og map goal coordinate x:" << ogMapGoalPose_.x << " y:" << ogMapGoalPose_.y <<  std::endl;
        robotGoalPose_.x = ogMapGoalPose_.x - 100;
        robotGoalPose_.y = 100 - ogMapGoalPose_.y;
        robotGoalPose_.yaw = atan2(robotGoalPose_.y,robotGoalPose_.x);
        std::cout << "(robot reference) goal pose x:"<< robotGoalPose_.x << " y:" << robotGoalPose_.y << " yaw:"
                  << robotGoalPose_.yaw << ", " << robotGoalPose_.yaw * 180/3.1415 << " Degree"<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

FrontierExploration::OgPose FrontierExploration::computeFontierCell(cv::Mat ogMap)
{
    int min_dist = 9999;
    OgPose minDistPose;
    OgPose frontierCell;
    // Clear previous frontier cells before storing new cells
    frontierCells_.clear();
    // Iterate each row pixels to find frontier cell
    for(int i = 0; i < ogMap.rows; i++)
    {
        double prev = 0;
        for(int j = 0; j < ogMap.cols; j++)
        {
            // obtain current cell color
            int current = ogMap.at<uchar>(i,j);
            // If the previous cell and current cell are both not the same and not dark cell, then it is a frontier cell
            if((prev != current) && current != 0 && prev != 0)
            {
                // Store the frontier cell into a cell container
                frontierCell.x = j;
                frontierCell.y = i;
                frontierCells_.push_back(frontierCell);

                // Calculate the frontier cell distance relative to the robot
                double dist = sqrt(pow((100-i),2) + pow((100-j),2));
                if(dist < min_dist)
                {
                    // Temporarily store the shortest distance ogMap coordinate
                    minDistPose.x = j;
                    minDistPose.y = i;
                    min_dist = dist;
                }
            }
            // Set previous cell as current cell
            prev = current;
        }
    }

    // Iterate each column pixels to find frontier cell
    for(int j = 0; j < ogMap.cols; j++)
    {
        double prev = 0;
        for(int i = 0; i < ogMap.rows; i++)
        {
            // obtain current cell color
            int current = ogMap.at<uchar>(i,j);
            // If the previous cell and current cell are both not the same and not dark cell, then it is a frontier cell
            if((prev != current) && current != 0 && prev != 0)
            {
                frontierCell.x = j;
                frontierCell.y = i;
                frontierCells_.push_back(frontierCell);
                double dist = sqrt(pow((100-i),2) + pow((100-j),2));
                if(dist < min_dist)
                {
                    // Temporarily store the shortest distance ogMap coordinate
                    minDistPose.x = j;
                    minDistPose.y = i;
                    min_dist = dist;
                }
            }
            prev = current;
        }
    }
    return minDistPose;
}


void FrontierExploration::processFrontier()
{
    // Continuously calculate the frontier cell and pose
    while(ros::ok())
    {
        if(imgBuffer_.imgBuffer.size()){
            imgBuffer_.imgMtx.lock();
            cv::Mat map = imgBuffer_.imgBuffer.front();
            imgBuffer_.imgMtx.unlock();
            // Clone a copy of map to frontier_map
            cv::Mat frontier_map = map.clone();

            // set frontier map image to white
            frontier_map = 255;

            ogMapGoalPose_ = computeFontierCell(map);

            for(auto &i : frontierCells_)
                frontier_map.at<uchar>(i.y,i.x) = 0;
            cv::Point pt;
            pt.x = ogMapGoalPose_.x;
            pt.y = ogMapGoalPose_.y;
            cv::Point robotPos;
            robotPos.x = 100;
            robotPos.y = 100;
            cv::circle(frontier_map, pt, 10, 0,1);
            cv::circle(frontier_map,robotPos,3,0,3);
            cv::imshow("frontier_map", frontier_map);
            cv::imshow("ogMap", map);
            cv::waitKey(10);
        }
    }
}

void FrontierExploration::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // if the image is color then choose it as color, else mono color
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
            cvImgPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        else
            cvImgPtr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    // If exception has occurred, return
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Lock and push back cv::Mat image data
    imgBuffer_.imgMtx.lock();
    imgBuffer_.imgBuffer.push_back(cvImgPtr_->image);
    if(imgBuffer_.imgBuffer.size() > 2)
        imgBuffer_.imgBuffer.pop_front();
    imgBuffer_.imgMtx.unlock();
}

void FrontierExploration::odomCallBack(const nav_msgs::OdometryConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    OgPose currentPose;
    currentPose.x = pose.position.x;
    currentPose.y = pose.position.y;
    currentPose.yaw = tf::getYaw(pose.orientation);
    poseBuffer.mtx.lock();
    poseBuffer.buffer.push_back(currentPose);
    poseBuffer.mtx.unlock();
    //    cout << "odom returned attributes: x = " << currentPose.x << ", y = " << currentPose.y << ", yaw = " << currentPose.yaw << endl;
}
