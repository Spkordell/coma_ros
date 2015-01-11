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
	step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);

	ROS_INFO("COMA Motion Demo Node Started");
}


void motion_demo::publish_cmd() {
	static int counts[12];
	for (unsigned int i; i < 12; i++) {
		counts[i] += 1;
		cmd.stepper_counts[i] = counts[i];
	}
	step_cmd_out.publish(cmd);
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "motion_demo");

	// initialize the joystick controller
	motion_demo demo;

	ros::Rate loop_rate(100);  //rate at which to publish arm velocity commands
	while (ros::ok()) {
		demo.publish_cmd();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
