/*!
 * \serial_node.h
 * \brief Handles serial communication with the continuum manipulator's control board
 *
 * Handles serial communication with the continuum manipulator's control board
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 10, 2015
 */

#ifndef SERIAL_NODE_H_
#define SERIAL_NODE_H_

#include <ros/ros.h>
#include <boost/asio.hpp>

#include "coma_serial/command.h"

class serial_node {
public:
	/*!
	 * Creates a serial_node object
	 */
	serial_node();

	void writeString(std::string s);
	std::string readLine();

private:
	ros::NodeHandle nh; /*!< a handle for this ros node */

	ros::Subscriber step_cmd_in; /*!< the step_cmd_in topic */

	//parameters
	std::string port; /*!< the port to use for sending the serial data */
	int baud; /*!< the baud rate for communication */

	boost::asio::io_service io;
	boost::asio::serial_port serial;

	void step_cmd_cback(const coma_serial::command::ConstPtr& cmd);
};

/*!
 * Creates and runs the serial_node node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //SERIAL_NODE_H_
