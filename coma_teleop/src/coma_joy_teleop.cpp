/*!
 * \coma_joy_teleop.cpp
 * \brief Allows for control of coma with a joystick.
 *
 * coma_joy_teleop creates a ROS node that allows the control of coma with a joystick.
 * This node listens to a /joy topic and sends messages to the angular_cmd and cartesian_cmd for the arm.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 11, 2015
 */

#include <coma_teleop/coma_joy_teleop.h>

using namespace std;

coma_joy_teleop::coma_joy_teleop() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	//read in parameters
	private_nh.param<bool>("send_motion_commands", send_motion_commands, true);
	private_nh.param<bool>("use_real_ik", use_real_ik, true);

	//initialize arm position variables
	x_pos = INITIAL_X_POS;
	y_pos = INITIAL_Y_POS;
	z_pos = INITIAL_Z_POS;
	x_rot = INITIAL_X_ROT;
	y_rot = INITIAL_Y_ROT;
	z_rot = INITIAL_Z_ROT;
	gripper_open = true;
	home = false;
	old_x_pos = INITIAL_X_POS;
	old_y_pos = INITIAL_Y_POS;
	old_z_pos = INITIAL_Z_POS;
	old_x_rot = INITIAL_X_ROT;
	old_y_rot = INITIAL_Y_ROT;
	old_z_rot = INITIAL_Z_ROT;
	old_gripper_open = true;
	old_home = false;

	calibrated = false;
	initLeftTrigger = false;
	initRightTrigger = false;
	motion_response_received = true;

	if (use_real_ik) {
		x_pos_multiplier = 0.02;
		y_pos_multiplier = 0.02;
		z_pos_multiplier = 0.01;
		x_rot_multiplier = 3.0;
		y_rot_multiplier = 3.0;
		z_rot_multiplier = 1.0;
	} else {
		x_pos_multiplier = 0.001;
		y_pos_multiplier = 0.001;
		z_pos_multiplier = 0.001;
		x_rot_multiplier = 0.01;
		y_rot_multiplier = 0.01;
		z_rot_multiplier = 0.01;
	}

	// create the ROS topics
	if (send_motion_commands) {
		motion_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
		motion_resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &coma_joy_teleop::motion_resp_cback, this);
	}
	joy_sub = node.subscribe < sensor_msgs::Joy > ("joy", 1, &coma_joy_teleop::joy_cback, this);
	if (use_real_ik) {
		solverClient = node.serviceClient < coma_kinematics::solveIK > ("solve_ik");
	} else {
		for (unsigned int i = 0; i < 12; i++) {
			//initialize leg lengths to zero
			leg_lengths[i] = homed_lengths[i];
		}
	}
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
		x_pos -= x_pos_multiplier * (joy->axes.at(0));		//Left stick horizontal
		y_pos += y_pos_multiplier * (joy->axes.at(1));		//Left Stick vertical
		z_pos -= z_pos_multiplier * (1 - joy->axes.at(2));  //Left trigger
		z_pos += z_pos_multiplier * (1 - joy->axes.at(5));  //Right trigger
		x_rot -= x_rot_multiplier * (joy->axes.at(3));		//Right stick horizontal
		y_rot += y_rot_multiplier * (joy->axes.at(4));		//Right stick vertical
		z_rot -= z_rot_multiplier * (joy->buttons.at(4));	//Left button
		z_rot += z_rot_multiplier * (joy->buttons.at(5));	//Right button
		if (joy->buttons.at(0)) { 							//A button
			gripper_open = true;
		} else if (joy->buttons.at(1)) {					// B button
			gripper_open = false;
		}
		home = joy->buttons.at(2);							//X button

		if (use_real_ik) {
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
		}
		if (home) {
			x_pos = INITIAL_X_POS;
			y_pos = INITIAL_Y_POS;
			z_pos = INITIAL_Z_POS;
			x_rot = INITIAL_X_ROT;
			y_rot = INITIAL_Y_ROT;
			z_rot = INITIAL_Z_ROT;
		}

		if (home != old_home) {
			old_home = home;
			if (home) {
				motion_cmd.home = true;
				motion_cmd.wrist_flex = 0;
				motion_cmd.wrist_rot = 0;
				motion_cmd.gripper_open = gripper_open;
				for (unsigned int leg; leg < 12; leg++) {
					motion_cmd.stepper_counts[leg] = 0;
				}
				if (send_motion_commands) {
					motion_response_received = false;
					motion_cmd_out.publish(motion_cmd);
				}
			}
		}

		if (x_pos != old_x_pos || y_pos != old_y_pos || z_pos != old_z_pos || x_rot != old_x_rot || y_rot != old_y_rot || z_rot != old_z_rot
				|| gripper_open != old_gripper_open) {

			cout << "------------------------------------------------------------" << endl;
			cout << x_pos << '\t' << y_pos << '\t' << z_pos << '\t' << x_rot << '\t' << y_rot << '\t' << z_rot << endl;

			if (use_real_ik) {
				coma_kinematics::solveIK srv;
				srv.request.x_pos = x_pos;
				srv.request.y_pos = y_pos;
				srv.request.z_pos = z_pos;
				srv.request.x_rot = x_rot;
				srv.request.y_rot = y_rot;
				srv.request.z_rot = z_rot;

				if (solverClient.call(srv)) {
					for (unsigned int leg; leg < 12; leg++) {
						cout << srv.response.leg_lengths[leg] << endl;
						int steps = convert_length_to_step(leg, srv.response.leg_lengths[leg]);
						if (steps < 0) {
							ROS_ERROR("MINIMUM LEG LENGTH REACHED");
							old_x_pos = x_pos;
							old_y_pos = y_pos;
							old_z_pos = z_pos;
							old_x_rot = x_rot;
							old_y_rot = y_rot;
							old_z_rot = z_rot;
							old_gripper_open = gripper_open;
							return;
						} else {
							motion_cmd.stepper_counts[leg] = steps;
						}
					}
#ifdef INCLUDE_WRIST
					cout << "wrist_rot: " << deg(srv.response.wrist_rot) << endl;
					cout << "wrist_flex: " << deg(srv.response.wrist_flex) << endl;
					motion_cmd.wrist_flex = -1*int(deg(srv.response.wrist_flex));
					motion_cmd.wrist_rot = -1*int(deg(srv.response.wrist_rot));
#else
					motion_cmd.wrist_flex = 0;
					motion_cmd.wrist_rot = 0;
#endif
					motion_cmd.gripper_open = gripper_open;
					if (gripper_open) {
						cout << "gripper open" << endl;
					} else {
						cout << "gripper closed" << endl;
					}
					motion_cmd.home = false;
					if (send_motion_commands) {
						motion_response_received = false;
						motion_cmd_out.publish(motion_cmd);
					} else {
						for (unsigned int leg = 0; leg < 11; leg++) {
							cout << motion_cmd.stepper_counts[leg] << ':';
						}
						cout << motion_cmd.stepper_counts[11] << endl;
					}
				} else {
					ROS_ERROR("Failed to call solver service");
				}
			} else { //use fake ik

				//rotate about z
				if (leg_lengths[1] + (z_rot - old_z_rot) > homed_lengths[1] && leg_lengths[3] + (z_rot - old_z_rot) > homed_lengths[3]
						&& leg_lengths[5] + (z_rot - old_z_rot) > homed_lengths[5]) {
					leg_lengths[1] += z_rot - old_z_rot;
					leg_lengths[3] += z_rot - old_z_rot;
					leg_lengths[5] += z_rot - old_z_rot;
				} else {
					leg_lengths[0] -= z_rot - old_z_rot;
					leg_lengths[2] -= z_rot - old_z_rot;
					leg_lengths[4] -= z_rot - old_z_rot;
				}
				//translate by z
				bool motion_possible = true;
				for (unsigned int leg = 0; leg < 6; leg++) {
					motion_possible &= leg_lengths[leg] + (z_pos - old_z_pos) > homed_lengths[leg];
				}
				if (motion_possible) {
					for (unsigned int leg = 0; leg < 6; leg++) {
						leg_lengths[leg] += (z_pos - old_z_pos);
					}
				}
				motion_possible = true;
				for (unsigned int leg = 6; leg < 12; leg++) {
					motion_possible &= leg_lengths[leg] + (z_pos - old_z_pos) > homed_lengths[leg];
				}
				if (motion_possible) {
					for (unsigned int leg = 6; leg < 12; leg++) {
						leg_lengths[leg] += (z_pos - old_z_pos);
					}
				}

				//rotate about x and y
				double theta = atan2((y_rot - old_y_rot), (x_rot - old_x_rot));
				double r = sqrt((x_rot - old_x_rot)*(x_rot - old_x_rot)+(y_rot - old_y_rot)*(y_rot - old_y_rot));
				if (theta <= .1745 && theta > -0.1745) {
					leg_lengths[0] += r;
					leg_lengths[1] += r;
				}
				if (theta <= 1.92 && theta > .1745) {
					leg_lengths[1] += r;
					leg_lengths[2] += r;
				}
				if (theta <= 2.269 && theta > 1.92) {
					leg_lengths[2] += r;
					leg_lengths[3] += r;
				}
				if (theta <= M_PI && theta > 2.269 || theta <= -2.269 && theta > -M_PI) {
					leg_lengths[3] += r;
					leg_lengths[4] += r;
				}
				if (theta <= -1.92 && theta > -2.269) {
					leg_lengths[4] += r;
					leg_lengths[5] += r;
				}
				if (theta <= -.1745 && theta > -1.92) {
					leg_lengths[5] += r;
					leg_lengths[0] += r;
				}

				//translate about x and y
				theta = atan2((y_pos - old_y_pos), (x_pos - old_x_pos));
				r = sqrt((x_pos - old_x_pos)*(x_pos - old_x_pos)+(y_pos - old_y_pos)*(y_pos - old_y_pos));
				if (theta <= .1745 && theta > -0.1745) {
					leg_lengths[11] += r;
					leg_lengths[8] += r;
				}
				if (theta <= 1.92 && theta > .1745) {
					leg_lengths[6] += r;
					leg_lengths[9] += r;
				}
				if (theta <= 2.269 && theta > 1.92) {
					leg_lengths[7] += r;
					leg_lengths[10] += r;
				}
				if (theta <= M_PI && theta > 2.269 || theta <= -2.269 && theta > -M_PI) {
					leg_lengths[8] += r;
					leg_lengths[11] += r;
				}
				if (theta <= -1.92 && theta > -2.269) {
					leg_lengths[9] += r;
					leg_lengths[6] += r;
				}
				if (theta <= -.1745 && theta > -1.92) {
					leg_lengths[10] += r;
					leg_lengths[7] += r;
				}


				for (unsigned int leg; leg < 12; leg++) {
					cout << leg_lengths[leg] << endl;
					int steps = convert_length_to_step(leg, leg_lengths[leg]);
					if (steps < 0) {
						ROS_ERROR("MINIMUM LEG LENGTH REACHED");
						old_x_pos = x_pos;
						old_y_pos = y_pos;
						old_z_pos = z_pos;
						old_x_rot = x_rot;
						old_y_rot = y_rot;
						old_z_rot = z_rot;
						old_gripper_open = gripper_open;
						return;
					} else {
						motion_cmd.stepper_counts[leg] = steps;
					}
				}
				motion_cmd.wrist_flex = 0;
				motion_cmd.wrist_rot = 0;
				motion_cmd.gripper_open = gripper_open;
				if (gripper_open) {
					cout << "gripper open" << endl;
				} else {
					cout << "gripper closed" << endl;
				}
				motion_cmd.home = false;
				if (send_motion_commands) {
					motion_response_received = false;
					motion_cmd_out.publish(motion_cmd);
				} else {
					for (unsigned int leg = 0; leg < 11; leg++) {
						cout << motion_cmd.stepper_counts[leg] << ':';
					}
					cout << motion_cmd.stepper_counts[11] << endl;
				}

			}

			old_x_pos = x_pos;
			old_y_pos = y_pos;
			old_z_pos = z_pos;
			old_x_rot = x_rot;
			old_y_rot = y_rot;
			old_z_rot = z_rot;
			old_gripper_open = gripper_open;

		}
	}
}

int coma_joy_teleop::convert_length_to_step(int leg, double length) {
	return (length - homed_lengths[leg]) * STEPS_PER_METER;
}

double coma_joy_teleop::deg(double radians) {
	return radians * (180.0 / M_PI);
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
