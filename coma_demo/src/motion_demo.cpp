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

	//initialize variables
	response_received = true;

	// create the ROS topics
	step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
	resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &motion_demo::resp_cback, this);

	ROS_INFO("COMA Motion Demo Node Started");
}


void motion_demo::resp_cback(const std_msgs::Char::ConstPtr& resp) {
	if (resp->data == 'R') {
		response_received = true;
	}
}

void motion_demo::publish_cmd() {
	static int counts[12];
	if (response_received) { //only publish if the board is ready for another command
		response_received = false;
		for (unsigned int i; i < 12; i++) {
			counts[i] += 20;
			cmd.stepper_counts[i] = counts[i];
		}
		step_cmd_out.publish(cmd);
	}
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "motion_demo");

	// initialize the joystick controller
	motion_demo demo;

	ros::Duration(3.0).sleep(); //short delay while everything initializes

	ros::Rate loop_rate(500);  //rate at which to publish arm velocity commands
	while (ros::ok()) {
		demo.publish_cmd();
		ros::spinOnce();
		loop_rate.sleep();
	}



	return EXIT_SUCCESS;
}
