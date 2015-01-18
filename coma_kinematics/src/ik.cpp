/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2014
 */

#include <coma_kinematics/ik.h>

using namespace std;

ik::ik() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	// create the ROS topics
	//step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
	//resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &ik::resp_cback, this);

	ROS_INFO("COMA IK Solver Started");
}



//void ik::resp_cback(const std_msgs::Char::ConstPtr& resp) {}
//void ik::publish_cmd() {}


//TODO: rename node "ik_server"

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "ik");

	// initialize the joystick controller
	ik solver;

	ros::spin();

//	ros::Rate loop_rate(500);  //rate at which to publish arm velocity commands
//	while (ros::ok()) {
//		ros::spinOnce();
//		loop_rate.sleep();
//	}



	return EXIT_SUCCESS;
}
