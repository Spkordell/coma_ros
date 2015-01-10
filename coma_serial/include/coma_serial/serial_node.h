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


class serial_node
{
public:
  /*!
   * Creates a serial_node object
   */
  serial_node();


private:
  ros::NodeHandle nh; /*!< a handle for this ros node */

 //ros::Subscriber cmd_vel_in; /*!< the cmd_vel_in topic */

  //parameters
  //int cameraCount; /*!< the number of cameras */


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
