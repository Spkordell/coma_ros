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

	//create the ROS topics
	step_cmd_in = node.subscribe < coma_serial::command
			> ("step_cmd", 0, &serial_node::step_cmd_cback, this);

	//set serial communication options

	serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
	//serial.open(port);

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

void serial_node::step_cmd_cback(const coma_serial::command::ConstPtr& cmd) {
	char buffer[30];
	sprintf(buffer, "%ld:%d:%ld\r",cmd->timestamp, cmd->stepper, cmd->counts);
	std::string out_buffer(buffer);
	writeString(out_buffer);
	ROS_INFO("%s",out_buffer.c_str());
}

int main(int argc, char **argv) {

	//initialize ROS and the node
	ros::init(argc, argv, "serial_node");

	try {
		serial_node node;

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
