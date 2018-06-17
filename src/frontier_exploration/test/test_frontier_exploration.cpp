#include "gtest/gtest.h"
#include "../src/frontier_exploration.h"

TEST(FrontierGoalTest, Pass)
{
    cv::Mat testMap(200,200,CV_8UC1,255);
    testMap.at<uchar>(90,100) = 127;
    ros::NodeHandle nh;
    FrontierExploration testFrontier(nh);
    FrontierExploration::OgPose pose;
    // Test a single unexplored cell inside a map of free space.
    pose = testFrontier.computeFontierCell(testMap);
    ASSERT_EQ(91,pose.y);
    ASSERT_EQ(100,pose.x);

    // Test another a single unexplored cell inside a map of free space.
    testMap = 255;
    testMap.at<uchar>(120,160) = 127;
    pose = testFrontier.computeFontierCell(testMap);
    ASSERT_EQ(120,pose.y);
    ASSERT_EQ(159,pose.x);

    // Test when both unexplored cell above in the same map.
    testMap.at<uchar>(90,100) = 127;
    pose = testFrontier.computeFontierCell(testMap);
    ASSERT_EQ(91,pose.y);
    ASSERT_EQ(100,pose.x);
}

TEST(FrontierSeenByGoalTest, Pass)
{
    cv::Mat testMap(200,200,CV_8UC1,255);
    testMap.at<uchar>(30,150) = 127;
    testMap.at<uchar>(50,120) = 127;
    ros::NodeHandle nh;
    FrontierExploration testFrontier(nh);
    FrontierExploration::OgPose pose;
    // Test a single unexplored cell inside a map of free space.
    pose = testFrontier.computeFontierCell(testMap);
    testFrontier.computeFrontierAtGoal();
    std::deque<FrontierExploration::OgPose> dequePoses = testFrontier.getGoalFrontierCells();
    // Test another a single unexplored cell inside a map of free space.

    pose = dequePoses.front();
    dequePoses.pop_front();
    ASSERT_EQ(50,pose.y);
    ASSERT_EQ(119,pose.x);

    pose = dequePoses.front();
    dequePoses.pop_front();
    ASSERT_EQ(50,pose.y);
    ASSERT_EQ(121,pose.x);

    pose = dequePoses.front();
    dequePoses.pop_front();
    ASSERT_EQ(49,pose.y);
    ASSERT_EQ(120,pose.x);

    pose = dequePoses.front();
    dequePoses.pop_front();
    ASSERT_EQ(51,pose.y);
    ASSERT_EQ(120,pose.x);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "frontier_Gtest");
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
