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



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "frontier_Gtest");
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
