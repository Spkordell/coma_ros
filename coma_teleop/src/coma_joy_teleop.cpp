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

#include <coma_teleop/coma_joy_teleop.h>

using namespace std;

coma_joy_teleop::coma_joy_teleop() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	// create the ROS topics
	//angular_cmd = node.advertise < wpi_jaco_msgs::AngularCommand > ("jaco_arm/angular_cmd", 10);
	//cartesian_cmd = node.advertise < wpi_jaco_msgs::CartesianCommand > ("jaco_arm/cartesian_cmd", 10);
	joy_sub = node.subscribe < sensor_msgs::Joy
			> ("joy", 10, &coma_joy_teleop::joy_cback, this);
}

void coma_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy) {
}

void coma_joy_teleop::publish_cmd() {
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "coma_joy_teleop");

	// initialize the joystick controller
	coma_joy_teleop controller;

	ros::Rate loop_rate(60);  //rate at which to publish arm velocity commands
	while (ros::ok()) {
		controller.publish_cmd();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
