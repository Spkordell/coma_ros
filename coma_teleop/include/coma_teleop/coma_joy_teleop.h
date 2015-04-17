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

#ifndef COMA_JOY_TELEOP_H_
#define COMA_JOY_TELEOP_H_

#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Char.h>

#include "coma_simple_planner/path_request.h"
#include "coma_kinematics/solveIK.h"
#include "coma_serial/teleop_command.h"

#define INCLUDE_WRIST //if defined, model will include a 2DOF wrist
#define USE_MULTITHREADING //if defined, solver will be launched in seperate thread

#define MIN_X_POSITION -0.6
#define MIN_Y_POSITION -0.6
#define MIN_Z_POSITION 0.33
#define MIN_X_ROTATION -90.0
#define MIN_Y_ROTATION -90.0
#define MIN_Z_ROTATION -120.0

#define MAX_X_POSITION 0.6
#define MAX_Y_POSITION 0.6
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

#define MIN_LENGTH_BOTTOM 0.22
#define MIN_LENGTH_TOP 0.34

//#define MAX_DISPLACEMENT 0.4953
//#define MAX_DISPLACEMENT 0.69
#define MAX_DISPLACEMENT 3.0
#define MAX_LENGTH_TOP MIN_LENGTH_TOP+MAX_DISPLACEMENT
#define MAX_LENGTH_BOTTOM MIN_LENGTH_BOTTOM+MAX_DISPLACEMENT
#define MAX_STEPS_TOP MAX_LENGTH_TOP*STEPS_PER_METER
#define MAX_STEPS_BOTTOM MAX_LENGTH_BOTTOM*STEPS_PER_METER

#define FAKE_IK_TOP 0
#define FAKE_IK_BOTTOM 1
#define FAKE_IK_BOTH 2
#ifdef INCLUDE_WRIST
#define FAKE_IK_WRIST 3
#endif

#ifdef INCLUDE_WRIST
#define MIN_WRIST_ROTATE 0
#define MIN_WRIST_FLEX 0
#define MAX_WRIST_ROTATE 180
#define MAX_WRIST_FLEX 180
#endif

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

	//callbacks
	void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);
	void motion_resp_cback(const std_msgs::Char::ConstPtr& resp);

	//helper methods
	int convert_length_to_step(int leg, double length);
	void transmit_leg_lengths(double lengths[12], double wrist_flex, double wrist_rot);
	void solve_thread();
	static double deg(double radians);
	static double rad(double degrees);

	ros::NodeHandle node; /*!< a handle for this ROS node */

	ros::Publisher motion_cmd_out; /*!< angular arm command topic */
	ros::Subscriber motion_resp_in;
	ros::Subscriber joy_sub; /*!< the joy topic */
	ros::ServiceClient solverClient;
	ros::ServiceClient pathSolverClient;

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

	bool y_button_pressed;
	bool left_arrow_button_pressed;
	bool right_arrow_button_pressed;
	bool up_arrow_button_pressed;
	bool down_arrow_button_pressed;
	bool old_y_button_pressed;
	bool old_left_arrow_button_pressed;
	bool old_right_arrow_button_pressed;
	bool old_up_arrow_button_pressed;
	bool old_down_arrow_button_pressed;

	double x_pos_multiplier;
	double y_pos_multiplier;
	double z_pos_multiplier;
	double x_rot_multiplier;
	double y_rot_multiplier;
	double z_rot_multiplier;
	double wrist_flex_multiplier;
	double wrist_rotate_multiplier;

	double homed_lengths[12] = { MIN_LENGTH_TOP, MIN_LENGTH_TOP, MIN_LENGTH_TOP, MIN_LENGTH_TOP, MIN_LENGTH_TOP, MIN_LENGTH_TOP, MIN_LENGTH_BOTTOM,
	MIN_LENGTH_BOTTOM, MIN_LENGTH_BOTTOM, MIN_LENGTH_BOTTOM, MIN_LENGTH_BOTTOM, MIN_LENGTH_BOTTOM };
	double leg_lengths[12]; //only used in fake_ik mode (0-5 = top, 6-11 = bottom)

	double wrist_rotate = 0;
	double wrist_flex = 0;

	bool initLeftTrigger; /*!< flag for whether the left trigger is initialized */
	bool initRightTrigger; /*!< flag for whether the right trigger is initialized */
	bool calibrated; /*!< flag for whether the controller is calibrated */
	bool solver_running; /*if true, solver is already busy running in another thread*/
	bool solution_out_of_date; /*if true, the current solution does not match the desired pose and the solver needs to be ran again */
	char fake_ik_mode;
	Eigen::Vector3d pos_vec[12];

	//parameters
	bool send_motion_commands; /*!< if true, node will send motion commands to the manipulator */
	bool use_real_ik; /*!< if true, node will send motion commands to the ik solver, if false, node will approximate ik */
	bool smooth_path; /*!< if true, node will send desired positions to the path planner to receive a smoother path to the goal */

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
