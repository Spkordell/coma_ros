/*!
 * \planner.cpp
 * \brief todo
 *
 * todo
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date March 18, 2015
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "coma_kinematics/solveIK.h"
#include "coma_simple_planner/configuration.h"
#include "coma_simple_planner/path_request.h"

/*!
 * \class motion_demo
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 */
class planner {
public:
	planner();

private:
	ros::NodeHandle n;

	ros::ServiceServer plannerService;
	ros::ServiceClient solverClient;

	bool plan_path(coma_simple_planner::path_request::Request &req, coma_simple_planner::path_request::Response &res);
};

/*!
 * Creates and runs the motion_demo node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //PLANNER_H
