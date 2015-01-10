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

serial_node::serial_node()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");
  nh = node;

  //Read in parameters
  //node.param<int>("camera_count", cameraCount, 1);


  //initialize variables

  //create the ROS topics
  //cmd_vel_in = node.subscribe < geometry_msgs::Twist > ("cmd_vel", 1, &serial_node::cmd_vel_cback, this);

  ROS_INFO("COMA Serial Node Started");
}


//void serial_node::cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& vel)
//{
//  cmdVelTimer.stop();
//  driving = true;
//  cmdVelTimer.start();
//}


int main(int argc, char **argv)
{
  //initialize ROS and the node
  ros::init(argc, argv, "serial_node");
  serial_node node;

  ros::Rate r(40);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
