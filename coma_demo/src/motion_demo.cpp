/*!
 * \motion_demo.cpp
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 11, 2014
 */

#include <coma_demo/motion_demo.h>

using namespace std;

motion_demo::motion_demo() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	// create the ROS topics
	//angular_cmd = node.advertise < wpi_jaco_msgs::AngularCommand > ("jaco_arm/angular_cmd", 10);
}


void motion_demo::publish_cmd() {
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "motion_demo");

	// initialize the joystick controller
	motion_demo controller;

	ros::Rate loop_rate(60);  //rate at which to publish arm velocity commands
	while (ros::ok()) {
		controller.publish_cmd();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
