/*!
 * \visualization.cpp
 * \brief todo
 *
 * todo
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date February 27, 2015
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "coma_rviz/vis.h"

/*!
 * \class motion_demo
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 */
class visualization {
public:
	visualization();

private:
	ros::NodeHandle n;
	void rodposCallback(const coma_rviz::vis::ConstPtr& msg);

	ros::Subscriber rod_pos_sub;
	ros::Publisher marker_pub;
};

/*!
 * Creates and runs the motion_demo node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //VISUALIZATION_H
