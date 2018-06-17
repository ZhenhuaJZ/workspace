#include "gtest/gtest.h"
#include "../src/frontier_exploration.h"

TEST(FrontierGoalTest, Pass)
{
    cv::Mat testMap(200,200,CV_8UC1,255);
    testMap.at<uchar>(90,100) = 127;
    ros::NodeHandle nh;
    FrontierExploration testFrontier(nh);
    FrontierExploration::OgPose pose;
    std::deque<FrontierExploration::OgPose> frontierCells;
    // Test a single unexplored cell inside a map of free space.
    frontierCells = testFrontier.computeFontierCell(testMap);
    pose = testFrontier.computeGoalPose(frontierCells);
    ASSERT_EQ(91,pose.y);
    ASSERT_EQ(100,pose.x);

    // Test another a single unexplored cell inside a map of free space.
    testMap = 255;
    testMap.at<uchar>(120,160) = 127;
    frontierCells = testFrontier.computeFontierCell(testMap);
    pose = testFrontier.computeGoalPose(frontierCells);
    ASSERT_EQ(120,pose.y);
    ASSERT_EQ(159,pose.x);

    // Test when both unexplored cell above in the same map.
    testMap.at<uchar>(90,100) = 127;
    frontierCells = testFrontier.computeFontierCell(testMap);
    pose = testFrontier.computeGoalPose(frontierCells);
    ASSERT_EQ(91,pose.y);
    ASSERT_EQ(100,pose.x);
}

TEST(FrontierSeenByGoalTest, Pass)
{
    // Test two frontier point on the map
    cv::Mat testMap(200,200,CV_8UC1,255);
    testMap.at<uchar>(30,150) = 127;
    testMap.at<uchar>(50,120) = 127;
    ros::NodeHandle nh;
    FrontierExploration testFrontier(nh);
    FrontierExploration::OgPose pose;
    std::deque<FrontierExploration::OgPose> frontierCells;
    std::deque<FrontierExploration::OgPose> goalFrontierCells;
    // Compute frontier cells, goal pose and frontier cells seen by goal pose
    frontierCells = testFrontier.computeFontierCell(testMap);
    pose = testFrontier.computeGoalPose(frontierCells);
    goalFrontierCells = testFrontier.computeFrontierAtGoal(frontierCells,pose);
    // Test another a single unexplored cell inside a map of free space.

    // Test each pose seen by goal in the goalFrontierCells container
    pose = goalFrontierCells.front();
    goalFrontierCells.pop_front();
    ASSERT_EQ(50,pose.y);
    ASSERT_EQ(119,pose.x);

    pose = goalFrontierCells.front();
    goalFrontierCells.pop_front();
    ASSERT_EQ(50,pose.y);
    ASSERT_EQ(121,pose.x);

    pose = goalFrontierCells.front();
    goalFrontierCells.pop_front();
    ASSERT_EQ(49,pose.y);
    ASSERT_EQ(120,pose.x);

    pose = goalFrontierCells.front();
    goalFrontierCells.pop_front();
    ASSERT_EQ(51,pose.y);
    ASSERT_EQ(120,pose.x);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "frontier_Gtest");
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
