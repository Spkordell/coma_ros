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
#include <unsupported/Eigen/MatrixFunctions>

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
	void solve();

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

	//position of the leg constraints in the local frame at the bottom
    Eigen::Vector3d p1_init;
    Eigen::Vector3d p2_init;
    Eigen::Vector3d p3_init;
    Eigen::Vector3d p4_init;
    Eigen::Vector3d p5_init;
    Eigen::Vector3d p6_init;
    Eigen::Vector3d p7_init;
    Eigen::Vector3d p8_init;
    Eigen::Vector3d p9_init;
    Eigen::Vector3d p10_init;
    Eigen::Vector3d p11_init;
    Eigen::Vector3d p12_init;

    //position of the leg constraints in the local frame at the top
    Eigen::Vector3d p6_final;
    Eigen::Vector3d p1_final;
    Eigen::Vector3d p2_final;
    Eigen::Vector3d p3_final;
    Eigen::Vector3d p4_final;
    Eigen::Vector3d p5_final;
    Eigen::Vector3d p12_final;
    Eigen::Vector3d p7_final;
    Eigen::Vector3d p8_final;
    Eigen::Vector3d p9_final;
    Eigen::Vector3d p10_final;
    Eigen::Vector3d p11_final;


//	ros::Publisher step_cmd_out;
//	ros::Subscriber resp_in;
//	void resp_cback(const std_msgs::Char::ConstPtr& resp);


	static double rad(double degrees);
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
