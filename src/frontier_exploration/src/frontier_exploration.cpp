#include "frontier_exploration.h"

FrontierExploration::FrontierExploration(ros::NodeHandle node)
    : nodeHandle_(node), imgTrans_(node)
{
    imgTransSub_ = imgTrans_.subscribe("map_image/full", 1, &FrontierExploration::imageCallback,this);
    odomSub_ = nodeHandle_.subscribe("odom", 1000, &FrontierExploration::odomCallBack,this);
    pathSub_ = nodeHandle_.subscribe("path", 20, &FrontierExploration::pathCallback,this);
    imageFbePublisher_ = imgTrans_.advertise("map_image/fbe",1);
    client_ = nodeHandle_.serviceClient<frontier_exploration::RequestGoal>("request_goal");
    resolution_ = 0.1;
}


FrontierExploration::~FrontierExploration()
{
    cv::destroyAllWindows();
}

std::deque<FrontierExploration::OgPose> FrontierExploration::getFrontierCells()
{
    return frontierCells_;
}

std::deque<FrontierExploration::OgPose> FrontierExploration::getGoalFrontierCells()
{
    return goalFrontierCells_;
}

FrontierExploration::OgPose FrontierExploration::computeGoalPose(std::deque<OgPose> frontierCells)
{
    int min_dist = 9999;
    OgPose minDistPose;
    // Calculate the frontier cell distance relative to the robot
    for (auto i : frontierCells)
    {
        double dist = sqrt(pow((ROBOT_LOCATION_CELL_Y-i.y),2) + pow((ROBOT_LOCATION_CELL_X-i.x),2));
        if(dist < min_dist)
        {
            // Temporarily store the shortest distance ogMap coordinate
            minDistPose.x = i.x;
            minDistPose.y = i.y;
            min_dist = dist;
        }
    }
    return minDistPose;
}

void FrontierExploration::calculateGoalPose()
{
    std::mutex mtx;
    while(ros::ok())
    {
        mtx.lock();
        // Display goal coordinate in reference of OgMap
        std::cout << "global robot coordinate x:" << poseBuffer.buffer.front().x << " y:" << poseBuffer.buffer.front().y
                  << " yaw:" << poseBuffer.buffer.front().yaw << std::endl;
        std::cout << "og map local goal coordinate x:" << ogMapReferenceGoalPose_.x << " y:" << ogMapReferenceGoalPose_.y <<  std::endl;

        // Calculate and display the goal pose in reference of the robot
        robotReferenceGoalPose_.x = (ogMapReferenceGoalPose_.x - 100)*resolution_;
        robotReferenceGoalPose_.y = (100 - ogMapReferenceGoalPose_.y)*resolution_;
        robotReferenceGoalPose_.yaw = atan2(robotReferenceGoalPose_.y,robotReferenceGoalPose_.x);
        std::cout << "(robot reference) goal pose x:"<< robotReferenceGoalPose_.x << " y:" << robotReferenceGoalPose_.y << " yaw:"
                  << robotReferenceGoalPose_.yaw << ", " << robotReferenceGoalPose_.yaw * 180/3.1415 << " Degree" << std::endl;

        // Calculate and display the goal pose in reference of the global coordinate system
        globalReferenceGoalPose_.x = robotReferenceGoalPose_.x + poseBuffer.buffer.front().x;
        globalReferenceGoalPose_.y = robotReferenceGoalPose_.y + poseBuffer.buffer.front().y;
        globalReferenceGoalPose_.yaw = robotReferenceGoalPose_.yaw;
        std::cout << "(global reference) goal pose x:" << globalReferenceGoalPose_.x
                  << " y:" <<  globalReferenceGoalPose_.y << " yaw:" << globalReferenceGoalPose_.yaw << std::endl << std::endl;
        frontier_exploration::RequestGoal srv;
        srv.request.x = globalReferenceGoalPose_.x;
        srv.request.y = globalReferenceGoalPose_.y;
        srv.request.yaw = globalReferenceGoalPose_.yaw;
        if (client_.call(srv))
            ROS_INFO("Receive Status: %ld", (int)srv.response.ack);
        else
            ROS_ERROR("No Response");
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::deque<FrontierExploration::OgPose> FrontierExploration::computeFontierCell(cv::Mat ogMap)
{
    // Define objects that is required to compute fontier cells
    std::deque<OgPose> frontierCellContainer;
    OgPose frontierCell;
    // Iterate each row pixels to find the frontier cell
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
                if(current == 255)
                    frontierCell.x = j;
                else
                    frontierCell.x = j-1;
                frontierCell.y = i;
                frontierCellContainer.push_back(frontierCell);
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
                // Store the frontier cell into the frontierCells container
                if(current == 255)
                    frontierCell.y = i;
                else
                    frontierCell.y = i-1;
                frontierCell.x = j;
                frontierCellContainer.push_back(frontierCell);
            }
            prev = current;
        }
    }
    return frontierCellContainer;
}

std::deque<FrontierExploration::OgPose> FrontierExploration::computeFrontierAtGoal(std::deque<OgPose> frontierCells, OgPose goalPose)
{
    // given goal pose and calculated frontier cells
    // Obtain a deque of frontier cells seem by goal
    std::deque<OgPose> goalFrontierCells;
   std::mutex mtx;
    for (auto i : frontierCells)
    {
        double frontierYaw = 0;
        double robotPoseYaw = 0;

        double frontierToRobotX = (i.x - 100)*resolution_;
        double frontierToRobotY = (100 - i.y)*resolution_;
        double frontierToRobotDist = sqrt(pow(frontierToRobotY,2) + pow(frontierToRobotX,2));
        double frontierToRobotYaw = atan2(frontierToRobotY,frontierToRobotX);

        // Convert pose negative yaw into positive angle
        if(frontierToRobotYaw < 0)
            frontierYaw = 6.28 + frontierToRobotYaw;
        else
            frontierYaw = frontierToRobotYaw;
        OgPose robotReferenceGoalPose;
        robotReferenceGoalPose.x = (goalPose.x - 100)*resolution_;
        robotReferenceGoalPose.y = (100 - goalPose.y)*resolution_;
        robotReferenceGoalPose.yaw = atan2(robotReferenceGoalPose.y,robotReferenceGoalPose.x);

        // Convert robot pose yaw into positive angle
        if(robotReferenceGoalPose.yaw < 0)
            robotPoseYaw = 6.28 + robotReferenceGoalPose.yaw;
        else
            robotPoseYaw = robotReferenceGoalPose.yaw;

        if((frontierYaw > robotPoseYaw - 1.27) && (frontierYaw < robotPoseYaw + 1.27) && frontierToRobotDist < 8)
            goalFrontierCells.push_back(i);
    }
    return goalFrontierCells;
}

void FrontierExploration::processFrontier()
{
    // Continuously calculate the frontier cell and pose
    while(ros::ok())
    {
        if(imgBuffer_.imgBuffer.size()){
            imgBuffer_.imgMtx.lock();
            OgMap_ = imgBuffer_.imgBuffer.front();
            // Clone a copy of OgMap to frontierMap_
            frontierMap_ = OgMap_.clone();
            // set frontier OgMap_ image to white
            frontierMap_ = 255;
            cv::cvtColor(frontierMap_,frontierMap_, cv::COLOR_GRAY2BGR);
            // calculate the goal pose in reference to the OgMap
            frontierCells_ = computeFontierCell(OgMap_);
            // Draw the frontier cells on the frontierMap_

            ogMapReferenceGoalPose_ = computeGoalPose(frontierCells_);

            goalFrontierCells_ =  computeFrontierAtGoal(frontierCells_, ogMapReferenceGoalPose_);

            for(auto &i : frontierCells_)
            {
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[0] = 0;
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[1] = 0;
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[2] = 0;
            }

            for(auto &i : goalFrontierCells_)
            {
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[0] = 0;
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[1] = 0;
                frontierMap_.at<cv::Vec3b>(i.y,i.x)[2] = 255;
            }

            frontierMap_.at<cv::Vec3b>(ogMapReferenceGoalPose_.y,ogMapReferenceGoalPose_.x)[0] = 255;
            frontierMap_.at<cv::Vec3b>(ogMapReferenceGoalPose_.y,ogMapReferenceGoalPose_.x)[1] = 0;
            frontierMap_.at<cv::Vec3b>(ogMapReferenceGoalPose_.y,ogMapReferenceGoalPose_.x)[2] = 0;

            cvImageFbe_.image = frontierMap_;
            imageFbePublisher_.publish(cvImageFbe_.toImageMsg());
            // Draw a circle at the goal pose and draw the robot location
            cv::circle(frontierMap_,cv::Point(ogMapReferenceGoalPose_.x,ogMapReferenceGoalPose_.y), 10, 0,1);
            cv::circle(frontierMap_,cv::Point(ROBOT_LOCATION_CELL_X,ROBOT_LOCATION_CELL_Y),2,0,2);
            cv::imshow("frontierMap_", frontierMap_);
            cv::imshow("ogMap", OgMap_);
            imgBuffer_.imgMtx.unlock();
            cv::waitKey(10000);
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
    // Extract pose information from odom topic.
    geometry_msgs::Pose pose = msg->pose.pose;
    // Create a OgPose storing current pose and store in buffer.
    OgPose currentPose;
    currentPose.x = pose.position.x;
    currentPose.y = pose.position.y;
    currentPose.yaw = tf::getYaw(pose.orientation);
    poseBuffer.mtx.lock();
    poseBuffer.buffer.push_back(currentPose);
    if(poseBuffer.buffer.size() > 5)
        poseBuffer.buffer.pop_front();
    poseBuffer.mtx.unlock();
}


void FrontierExploration::pathCallback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    std::cout << "recieved pose array" << std::endl;
}

