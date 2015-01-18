/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2014
 */

#ifndef IK_H_
#define IK_H_

#include <ros/ros.h>


class ik {
public:
	ik();

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

//	ros::Publisher step_cmd_out;
//	ros::Subscriber resp_in;
//	void resp_cback(const std_msgs::Char::ConstPtr& resp);

};

/*!
 * Creates and runs the ik node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //IK_H
