/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2014
 */

#include <coma_kinematics/ik.h>

using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

ik::ik() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	// create the ROS topics
	//step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
	//resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &ik::resp_cback, this);

	ROS_INFO("COMA IK Solver Started");
}

//void ik::resp_cback(const std_msgs::Char::ConstPtr& resp) {}
//void ik::publish_cmd() {}

void ik::solvetest() {
	// The variable to solve for with its initial value.
	double initial_x = 5.0;
	double x = initial_x;

	// Build the problem.
	Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(
			new CostFunctor);
	problem.AddResidualBlock(cost_function, NULL, &x);

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n";
	std::cout << "x : " << initial_x << " -> " << x << "\n";
}

//TODO: rename node "ik_server"

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "ik");

	// initialize the joystick controller
	ik solver;

	cosserat_rod rod;
	rod.integrate();

	solver.solvetest();

	ros::spin();

//	ros::Rate loop_rate(500);  //rate at which to publish arm velocity commands
//	while (ros::ok()) {
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return EXIT_SUCCESS;
}
