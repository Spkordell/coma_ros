/*!
 * \coma_joy_teleop.cpp
 * \brief Allows for control of coma with a joystick.
 *
 * coma_joy_teleop creates a ROS node that allows the control of coma with a joystick.
 * This node listens to a /joy topic and sends messages to the angular_cmd and cartesian_cmd for the arm.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 11, 2014
 */

#ifndef COMA_JOY_TELEOP_H_
#define COMA_JOY_TELEOP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


/*!
 * \class coma_joy_teleop
 * \brief Allows for control of coma with a joystick.
 *
 * coma_joy_teleop creates a ROS node that allows the control of coma with a joystick.
 * This node listens to a /joy topic
 */
class coma_joy_teleop {
public:
	/*!
	 * Creates a coma_joy_teleop object that can be used control coma with a joystick. ROS nodes, services, and publishers
	 * are created and maintained within this object.
	 */
	coma_joy_teleop();

	/*!
	 * Periodically publish commands to the arm controller
	 */
	void publish_cmd();

private:
	/*!
	 * Joy topic callback function.
	 *
	 * \param joy the message for the joy topic
	 */
	void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle node; /*!< a handle for this ROS node */

	//ros::Publisher angular_cmd; /*!< angular arm command topic */
	//ros::Publisher cartesian_cmd; /*!< cartesian arm command topic */
	ros::Subscriber joy_sub; /*!< the joy topic */
};

/*!
 * Creates and runs the coma_joy_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //COMA_JOY_TELEOP_H
