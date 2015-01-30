/*!
 * \motion_demo.cpp
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 11, 2014
 */

#ifndef MOTION_DEMO_H_
#define MOTION_DEMO_H_

#include <ros/ros.h>
#include <std_msgs/Char.h>

#include "coma_serial/teleop_command.h"

/*!
 * \class motion_demo
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 */
class motion_demo {
public:
	/*!
	 * Creates a motion_demo object that can be used control coma. ROS nodes, services, and publishers
	 * are created and maintained within this object.
	 */
	motion_demo();

	/*!
	 * Periodically publish commands to the arm controller
	 */
	void publish_cmd();

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

	ros::Publisher step_cmd_out; /*!< angular arm command topic */
	ros::Subscriber resp_in;

	void resp_cback(const std_msgs::Char::ConstPtr& resp);

	coma_serial::teleop_command cmd; /*!< stepper command */
	bool response_received;
};

/*!
 * Creates and runs the motion_demo node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //MOTION_DEMO_H
