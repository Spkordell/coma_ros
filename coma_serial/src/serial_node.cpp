/*!
 * \serial_node.cpp
 * \brief Handles serial communication with the continuum manipulator's control board
 *
 * Handles serial communication with the continuum manipulator's control board
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 10, 2015
 */

#include "coma_serial/serial_node.h"

using namespace std;

serial_node::serial_node() :
		io(), serial(io, "/dev/ttyUSB0") {
	//a private handle for this ROS node
	ros::NodeHandle node("~");
	nh = node;

	//Read in parameters
	node.param < string > ("port", port, "/dev/ttyUSB0"); //todo: this needs to be implemented (Currently always opens the default port)
	node.param<int>("baud", baud, 115200);
	node.param < string > ("mode", mode_string, "teleop");

	//create the ROS topics
	if (mode_string.compare("teleop") == 0) {
		mode = TELEOP;
		step_cmd_in = node.subscribe < coma_serial::teleop_command
				> ("step_cmd", 0, &serial_node::step_cmd_cback, this);
	} else {
		mode = PATH;
		step_cmd_in = node.subscribe < coma_serial::path_command
				> ("step_cmd", 0, &serial_node::step_cmd_cback, this);
	}

	//set serial communication options
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

	ROS_INFO("COMA Serial Node Started");
}

void serial_node::writeString(std::string s) {
	boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
}

std::string serial_node::readLine() {
	//Reading data char by char, code is optimized for simplicity, not speed
	using namespace boost;
	char c;
	std::string result;
	for (;;) {
		asio::read(serial, asio::buffer(&c, 1));
		switch (c) {
		case '\r':
			break;
		case '\n':
			return result;
		default:
			result += c;
		}
	}
}

char serial_node::readChar() {
	using namespace boost;
	char c;
	asio::read(serial, asio::buffer(&c, 1));
	return c;
}

void serial_node::writeChar(char c) {
	char s[2];
	s[0] = c;
	s[1] = '\0';
	boost::asio::write(serial, boost::asio::buffer(s, 1)); //use teleop mode
}

void serial_node::step_cmd_cback(
		const coma_serial::path_command::ConstPtr& cmd) {
	char buffer[30];
	sprintf(buffer, "%ld:%d:%ld\r", cmd->timestamp, cmd->stepper, cmd->counts);
	std::string out_buffer(buffer);
	writeString(out_buffer);
	ROS_INFO("%s", out_buffer.c_str());
}

void serial_node::step_cmd_cback(
		const coma_serial::teleop_command::ConstPtr& cmd) {

	ostringstream buffer("");
	for (int i = 0; i < 11; i++) {
		buffer << cmd->stepper_counts[i] << ':';
	}
	buffer << cmd->stepper_counts[11] << '\r';
	writeString(buffer.str());
	ROS_INFO("%s", buffer.str().c_str());
}

mode_type serial_node::get_mode() {
	return this->mode;
}

int main(int argc, char **argv) {

	//initialize ROS and the node
	ros::init(argc, argv, "serial_node");
	try {
		serial_node node;

		//while (node.readChar() != 'I'); //wait for board to be initialized
		if (node.get_mode() == TELEOP) {
			node.writeChar('T'); //use teleop mode
		} else {
			node.writeChar('P'); //use path mode
		}

		//cout << node.readLine() << endl;

		ros::spin();
		/*
		 ros::Rate r(40);

		 while (ros::ok()) {
		 ros::spinOnce();
		 r.sleep();
		 }
		 */

	} catch (boost::system::system_error& e) {
		cerr << "Serial Communication Error: " << e.what() << endl;
		return 1;

	}

	return EXIT_SUCCESS;

}
