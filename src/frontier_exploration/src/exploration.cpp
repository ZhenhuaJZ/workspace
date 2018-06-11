#include "frontier_exploration.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_exploration");

    ros::NodeHandle nh;
    std::cout << "node handle created" << std::endl;

    FrontierExploration frontier(nh);
    std::cout << "Exploration Object Created" << std::endl;

    FrontierExploration *frontierPtr = &frontier;
    std::cout << "Frontier pointer created" << std::endl;

    std::thread frontierProcessThread(&FrontierExploration::processFrontier, frontierPtr);
    std::thread goalPoseProcessThread(&FrontierExploration::calculateGoalPose, frontierPtr);

    ros::spin();
    ros::shutdown();

    frontierProcessThread.join();
    goalPoseProcessThread.join();

    cv::destroyAllWindows();

    std::cout << "quack" << std::endl;
}


