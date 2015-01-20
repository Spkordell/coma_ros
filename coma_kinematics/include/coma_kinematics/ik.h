/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2014
 */

#ifndef IK_H_
#define IK_H_

#include <ros/ros.h>

#include "ceres/ceres.h"
#include "coma_kinematics/cosserat_rod.h"

struct CostFunctor {
	template<typename T>
	bool operator()(const T* const x, T* residual) const {
		residual[0] = T(10.0) - x[0];
		return true;
	}
};

class ik {
public:
	ik();
	void solvetest();

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

//	ros::Publisher step_cmd_out;
//	ros::Subscriber resp_in;
//	void resp_cback(const std_msgs::Char::ConstPtr& resp);

};

/*!
 * Creates and runs the ik node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //IK_H
