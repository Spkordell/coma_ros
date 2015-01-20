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

using Eigen::Vector3d;
using Eigen::Matrix3d;

double ik::rad(double degrees) {
    return degrees*(M_PI/180.0);
}

ik::ik() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	// create the ROS topics
	//step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
	//resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &ik::resp_cback, this);

	//initialize system parameters
    double r_in = 0.06126; 	//radius of inner leg hole pattern
    double r_out = 0.06770; //radius of outer leg hole pattern
    double deg = 10; 		//separation in degrees between the hole pairs that are close

    //position of the leg constraints in the local frame at the bottom
    p1_init = *(new Vector3d(r_in*cos(rad(0-deg)), r_in*sin(rad(0-deg)), 0));
    p2_init = *(new Vector3d(r_in*cos(rad(0+deg)), r_in*sin(rad(0+deg)), 0));
    p3_init = *(new Vector3d(r_in*cos(rad(120-deg)), r_in*sin(rad(120-deg)), 0));
    p4_init = *(new Vector3d(r_in*cos(rad(120+deg)), r_in*sin(rad(120+deg)), 0));
    p5_init = *(new Vector3d(r_in*cos(rad(240-deg)), r_in*sin(rad(240-deg)), 0));
    p6_init = *(new Vector3d(r_in*cos(rad(240+deg)), r_in*sin(rad(240+deg)), 0));
    p7_init = *(new Vector3d(r_out*cos(rad(60-deg)), r_out*sin(rad(60-deg)), 0));
    p8_init = *(new Vector3d(r_out*cos(rad(60+deg)), r_out*sin(rad(60+deg)), 0));
    p9_init = *(new Vector3d(r_out*cos(rad(180-deg)), r_out*sin(rad(180-deg)), 0));
    p10_init = *(new Vector3d(r_out*cos(rad(180+deg)), r_out*sin(rad(180+deg)), 0));
    p11_init = *(new Vector3d(r_out*cos(rad(300-deg)), r_out*sin(rad(300-deg)), 0));
    p12_init = *(new Vector3d(r_out*cos(rad(300+deg)), r_out*sin(rad(300+deg)), 0));

    //position of the leg constraints in the local frame at the top
    p6_final = *(new Vector3d(r_in*cos(rad(0-deg)), r_in*sin(rad(0-deg)), 0));
    p1_final = *(new Vector3d(r_in*cos(rad(0+deg)), r_in*sin(rad(0+deg)), 0));
    p2_final = *(new Vector3d(r_in*cos(rad(120-deg)), r_in*sin(rad(120-deg)), 0));
    p3_final = *(new Vector3d(r_in*cos(rad(120+deg)), r_in*sin(rad(120+deg)), 0));
    p4_final = *(new Vector3d(r_in*cos(rad(240-deg)), r_in*sin(rad(240-deg)), 0));
    p5_final = *(new Vector3d(r_in*cos(rad(240+deg)), r_in*sin(rad(240+deg)), 0));
    p12_final = *(new Vector3d(r_out*cos(rad(0-deg)), r_out*sin(rad(0-deg)), 0));
    p7_final = *(new Vector3d(r_out*cos(rad(0+deg)), r_out*sin(rad(0+deg)), 0));
    p8_final = *(new Vector3d(r_out*cos(rad(120-deg)), r_out*sin(rad(120-deg)), 0));
    p9_final = *(new Vector3d(r_out*cos(rad(120+deg)), r_out*sin(rad(120+deg)), 0));
    p10_final = *(new Vector3d(r_out*cos(rad(240-deg)), r_out*sin(rad(240-deg)), 0));
    p11_final = *(new Vector3d(r_out*cos(rad(240+deg)), r_out*sin(rad(240+deg)), 0));

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

void ik::solve() {
    //this matrix is the u matrix of unknowns described in the REACH paper.
	boost::array<double, 7*12> guess_init;
	std::fill(guess_init.begin(), guess_init.end(), 0);
	for (unsigned int i = 7*12-1; i >= 6*12; i--) {
		guess_init[i] = 0.12; //initialize leg lengths to 0.12 m
	}

	//set desired forces, moments, position, and rotation
    Vector3d F(0, 0, 0); //applied force at end effector
    Vector3d L(0, 0, 0); //applied moment at end effector
    Matrix3d Rd = cosserat_rod::hat(*(new Vector3d(rad(0),rad(0),rad(-60)))); //desired end effector orientation
    Eigen::MatrixExponential<Matrix3d> Rdm(Rd);
    Rdm.compute(Rd);
    Vector3d pd(0.0, 0.0, 0.6); //desired end effector position


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
	solver.solve();

	//ros::spin();

//	ros::Rate loop_rate(500);  //rate at which to publish arm velocity commands
//	while (ros::ok()) {
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	return EXIT_SUCCESS;
}
