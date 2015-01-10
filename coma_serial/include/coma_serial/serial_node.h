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

class serial_node
{
public:
  /*!
   * Creates a serial_node object
   */
  serial_node();

  void writeString(std::string s);
  std::string readLine();

private:
  ros::NodeHandle nh; /*!< a handle for this ros node */

 //ros::Subscriber cmd_vel_in; /*!< the cmd_vel_in topic */

  //parameters
  std::string port; /*!< the port to use for sending the serial data */
  int baud; /*!< the baud rate for communication */


  boost::asio::io_service io;
  boost::asio::serial_port serial;


  /*!
   * callback for receiving command velocities
   * \param vel The cmd_vel
   */
  //void cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& vel);
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
