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

	//initialize arm position variables
	x_pos = 0;
	y_pos = 0;
	z_pos = 0.3;
	x_rot = 0;
	y_rot = 0;
	z_rot = -60;

	calibrated = false;
	initLeftTrigger = false;
	initRightTrigger = false;

	x_pos_multiplier = 0.02;
	y_pos_multiplier = 0.02;
	z_pos_multiplier = 0.01;

	x_rot_multiplier = 3.0;
	y_rot_multiplier = 3.0;
	z_rot_multiplier = 1.0;

}

void coma_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy) {
	if (!calibrated) {
		if (!initLeftTrigger && joy->axes.at(2) == 1.0)
			initLeftTrigger = true;

		if (!initRightTrigger && joy->axes.at(5) == 1.0)
			initRightTrigger = true;

		if (initLeftTrigger && initRightTrigger) {
			calibrated = true;
			ROS_INFO("Controller calibration complete!");
		}
	} else {

		x_pos -=x_pos_multiplier*(joy->axes.at(3));
		y_pos +=y_pos_multiplier*(joy->axes.at(4));
		z_pos -= z_pos_multiplier*(1 - joy->axes.at(2));
		z_pos += z_pos_multiplier*(1 - joy->axes.at(5));
		x_rot -=x_rot_multiplier*(joy->axes.at(0));
		y_rot +=y_rot_multiplier*(joy->axes.at(1));
		z_rot -= z_rot_multiplier*(joy->buttons.at(4));
		z_rot += z_rot_multiplier*(joy->buttons.at(5));

		if (x_pos > MAX_X_POSITION) {
			x_pos = MAX_X_POSITION;
		} else if (x_pos < MIN_X_POSITION) {
			x_pos = MIN_X_POSITION;
		}
		if (y_pos > MAX_Y_POSITION) {
			y_pos = MAX_Y_POSITION;
		} else if (y_pos < MIN_Y_POSITION) {
			y_pos = MIN_Y_POSITION;
		}
		if (z_pos > MAX_Z_POSITION) {
			z_pos = MAX_Z_POSITION;
		} else if (z_pos < MIN_Z_POSITION) {
			z_pos = MIN_Z_POSITION;
		}

		if (x_rot > MAX_X_ROTATION) {
			x_rot = MAX_X_ROTATION;
		} else if (x_rot < MIN_X_ROTATION) {
			x_rot = MIN_X_ROTATION;
		}
		if (y_rot > MAX_Y_ROTATION) {
			y_rot = MAX_Y_ROTATION;
		} else if (y_rot < MIN_Y_ROTATION) {
			y_rot = MIN_Y_ROTATION;
		}
		if (z_rot > MAX_Z_ROTATION) {
			z_rot = MAX_Z_ROTATION;
		} else if (z_rot < MIN_Z_ROTATION) {
			z_rot = MIN_Z_ROTATION;
		}

		cout << x_pos << '\t' << y_pos << '\t' << z_pos << '\t' << x_rot << '\t' << y_rot << '\t' << z_rot << endl;
	}
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
