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

	//initialize arm position variables
	x_pos = 0;
	y_pos = 0;
	z_pos = 0.3;
	x_rot = 0;
	y_rot = 0;
	z_rot = -60;
	old_x_pos = 0;
	old_y_pos = 0;
	old_z_pos = 0;
	old_x_rot = 0;
	old_y_rot = 0;
	old_z_rot = 0;

	calibrated = false;
	initLeftTrigger = false;
	initRightTrigger = false;
	motion_response_received = true;

	x_pos_multiplier = 0.02;
	y_pos_multiplier = 0.02;
	z_pos_multiplier = 0.01;

	x_rot_multiplier = 3.0;
	y_rot_multiplier = 3.0;
	z_rot_multiplier = 1.0;

	private_nh.param<bool>("send_motion_commands", send_motion_commands, true);

	// create the ROS topics
	if (send_motion_commands) {
		motion_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
		motion_resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &coma_joy_teleop::motion_resp_cback, this);
	}
	joy_sub = node.subscribe < sensor_msgs::Joy > ("joy", 1, &coma_joy_teleop::joy_cback, this);
	solverClient = node.serviceClient < coma_kinematics::solveIK > ("solve_ik");

	ROS_INFO("COMA Motion Demo Node Started");
	ROS_INFO("Calibrate the controller by pressing and releasing both triggers");

}

void coma_joy_teleop::motion_resp_cback(const std_msgs::Char::ConstPtr& resp) {
	if (resp->data == 'R') {
		motion_response_received = true;
	}
}

void coma_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy) {
	if (!calibrated) {
		ROS_INFO("Calibrate the controller by pressing and releasing both triggers");
		if (!initLeftTrigger && joy->axes.at(2) == 1.0)
			initLeftTrigger = true;

		if (!initRightTrigger && joy->axes.at(5) == 1.0)
			initRightTrigger = true;

		if (initLeftTrigger && initRightTrigger) {
			calibrated = true;
			ROS_INFO("Controller calibration complete!");
		}
	} else if (motion_response_received || !send_motion_commands) { //only publish if the board is ready for another command
		motion_response_received = false;
		x_pos -= x_pos_multiplier * (joy->axes.at(0));
		y_pos += y_pos_multiplier * (joy->axes.at(1));
		z_pos -= z_pos_multiplier * (1 - joy->axes.at(2));
		z_pos += z_pos_multiplier * (1 - joy->axes.at(5));
		x_rot -= x_rot_multiplier * (joy->axes.at(3));
		y_rot += y_rot_multiplier * (joy->axes.at(4));
		z_rot -= z_rot_multiplier * (joy->buttons.at(4));
		z_rot += z_rot_multiplier * (joy->buttons.at(5));

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

		if (x_pos != old_x_pos || y_pos != old_y_pos || z_pos != old_z_pos || x_rot != old_x_rot || y_rot != old_y_rot || z_rot != old_z_rot) {
			cout << x_pos << '\t' << y_pos << '\t' << z_pos << '\t' << x_rot << '\t' << y_rot << '\t' << z_rot << endl;
			old_x_pos = x_pos;
			old_y_pos = y_pos;
			old_z_pos = z_pos;
			old_x_rot = x_rot;
			old_y_rot = y_rot;
			old_z_rot = z_rot;

			coma_kinematics::solveIK srv;
			srv.request.x_pos = x_pos;
			srv.request.y_pos = y_pos;
			srv.request.z_pos = z_pos;
			srv.request.x_rot = x_rot;
			srv.request.y_rot = y_rot;
			srv.request.z_rot = z_rot;

			if (solverClient.call(srv)) {
				for (unsigned int leg; leg < 12; leg++) {
					//cout << srv.response.leg_lengths[leg] << endl;
					int steps = convert_length_to_step(leg, srv.response.leg_lengths[leg]);
					if (steps < 0) {
						ROS_ERROR("MINIMUM LEG LENGTH REACHED");
						motion_response_received = true; //we haven't sent anything so reset the ready flag
						return;
					} else {
						motion_cmd.stepper_counts[leg] = steps;
					}
				}
				if (send_motion_commands) {
					motion_cmd_out.publish(motion_cmd);
				}
			} else {
				ROS_ERROR("Failed to call solver service");
			}
		}
	}
}

int coma_joy_teleop::convert_length_to_step(int leg, double length) {
	static double homed_lengths[12] = { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 }; //todo: this needs to be the amount of leg remaining after homing for each rod
	return (length - homed_lengths[leg]) * STEPS_PER_METER;
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "coma_joy_teleop");

	// initialize the joystick controller
	coma_joy_teleop controller;

	ros::Duration(3.0).sleep(); //short delay while other things initialize initializes

	ros::spin();

	return EXIT_SUCCESS;
}
