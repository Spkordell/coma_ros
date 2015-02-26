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
#include <std_msgs/Char.h>

#include "coma_kinematics/solveIK.h"
#include "coma_serial/teleop_command.h"

//#define INCLUDE_WRIST //if defined, model will include a 2DOF wrist

#define MIN_X_POSITION -0.3
#define MIN_Y_POSITION -0.3
#define MIN_Z_POSITION 0.33
#define MIN_X_ROTATION -90.0
#define MIN_Y_ROTATION -90.0
#define MIN_Z_ROTATION -120.0

#define MAX_X_POSITION 0.3
#define MAX_Y_POSITION 0.3
#define MAX_Z_POSITION 0.9
#define MAX_X_ROTATION 90.0
#define MAX_Y_ROTATION 90.0
#define MAX_Z_ROTATION 0.0

#define INITIAL_X_POS 0
#define INITIAL_Y_POS 0
#define INITIAL_Z_POS 0.33
#define INITIAL_X_ROT 0
#define INITIAL_Y_ROT 0
#define INITIAL_Z_ROT -60
//#define INITIAL_Z_ROT 0

#define SPROCKET_RADIUS 0.020 // radius in m
#define STEPS_PER_REVOLUTION 200
#define SPROCKET_CIRCUMFERENCE 2*M_PI*SPROCKET_RADIUS
#define STEPS_PER_METER STEPS_PER_REVOLUTION/(SPROCKET_CIRCUMFERENCE)

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

private:
	/*!
	 * Joy topic callback function.
	 *
	 * \param joy the message for the joy topic
	 */
	void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

	void motion_resp_cback(const std_msgs::Char::ConstPtr& resp);

	int convert_length_to_step(int leg, double length);

	static double deg(double radians);

	ros::NodeHandle node; /*!< a handle for this ROS node */

	ros::Publisher motion_cmd_out; /*!< angular arm command topic */
	ros::Subscriber motion_resp_in;
	ros::Subscriber joy_sub; /*!< the joy topic */
	ros::ServiceClient solverClient;

	double x_pos;
	double y_pos;
	double z_pos;
	double x_rot;
	double y_rot;
	double z_rot;
	bool gripper_open;
	bool home;
	double old_x_pos;
	double old_y_pos;
	double old_z_pos;
	double old_x_rot;
	double old_y_rot;
	double old_z_rot;
	bool old_gripper_open;
	bool old_home;

	double x_pos_multiplier;
	double y_pos_multiplier;
	double z_pos_multiplier;
	double x_rot_multiplier;
	double y_rot_multiplier;
	double z_rot_multiplier;

	bool initLeftTrigger; /*!< flag for whether the left trigger is initialized */
	bool initRightTrigger; /*!< flag for whether the right trigger is initialized */
	bool calibrated; /*!< flag for whether the controller is calibrated */
	bool send_motion_commands; /*!< if true, node will send motion commands to the manipulator */

	coma_serial::teleop_command motion_cmd; /*!< stepper command */
	bool motion_response_received;
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
