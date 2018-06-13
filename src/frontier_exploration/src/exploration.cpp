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

// tutorial project 1
// requrest_goal: The calculated goal sends via service call to PRM
// send goal pose and receive a path from PRM via a service call. no need to process path.
// Receives path on /path: only subscribe to the path topic, not service call
// bounu: point where there are most frontier cells
// scan goal frontiers: frontier cells seen by the laser scanner given the current explored map at the goal pose.
// The chosen frontier cell should be with in field of view of the laser sensor 181 scans.
// Robot head to the goal pose, and choose what are the frontier cells seen by the robot.
/*
 * The og map revceives, calculated a goal frontier cell given current laser scanner
 * The algorithm will calculate a goal pose to that frontier cell, then at that goal pose, what are the frontier cells seen by the field of view of laser.
 */

/*
 * Store coordinate into global coordinate
 */
