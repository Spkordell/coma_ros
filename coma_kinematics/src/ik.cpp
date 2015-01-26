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
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using Eigen::Vector3d;
using Eigen::Matrix3d;

double ik::rad(double degrees) {
	return degrees * (M_PI / 180.0);
}

ik::ik() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	//initialize system parameters
	double r_in = 0.06126; 	//radius of inner leg hole pattern
	double r_out = 0.06770; //radius of outer leg hole pattern
	double deg = 10; //separation in degrees between the hole pairs that are close

	ikfunctor = new IKFunctor;
	singleikfunctor = new SingleIKFunctor;

	//position of the leg constraints in the local frame at the bottom
	ikfunctor->p1_init = *(new Vector3d(r_in * cos(rad(0 - deg)),
			r_in * sin(rad(0 - deg)), 0));
	ikfunctor->p2_init = *(new Vector3d(r_in * cos(rad(0 + deg)),
			r_in * sin(rad(0 + deg)), 0));
	ikfunctor->p3_init = *(new Vector3d(r_in * cos(rad(120 - deg)),
			r_in * sin(rad(120 - deg)), 0));
	ikfunctor->p4_init = *(new Vector3d(r_in * cos(rad(120 + deg)),
			r_in * sin(rad(120 + deg)), 0));
	ikfunctor->p5_init = *(new Vector3d(r_in * cos(rad(240 - deg)),
			r_in * sin(rad(240 - deg)), 0));
	ikfunctor->p6_init = *(new Vector3d(r_in * cos(rad(240 + deg)),
			r_in * sin(rad(240 + deg)), 0));
	ikfunctor->p7_init = *(new Vector3d(r_out * cos(rad(60 - deg)),
			r_out * sin(rad(60 - deg)), 0));
	ikfunctor->p8_init = *(new Vector3d(r_out * cos(rad(60 + deg)),
			r_out * sin(rad(60 + deg)), 0));
	ikfunctor->p9_init = *(new Vector3d(r_out * cos(rad(180 - deg)),
			r_out * sin(rad(180 - deg)), 0));
	ikfunctor->p10_init = *(new Vector3d(r_out * cos(rad(180 + deg)),
			r_out * sin(rad(180 + deg)), 0));
	ikfunctor->p11_init = *(new Vector3d(r_out * cos(rad(300 - deg)),
			r_out * sin(rad(300 - deg)), 0));
	ikfunctor->p12_init = *(new Vector3d(r_out * cos(rad(300 + deg)),
			r_out * sin(rad(300 + deg)), 0));

	//position of the leg constraints in the local frame at the top
	ikfunctor->p6_final = *(new Vector3d(r_in * cos(rad(0 - deg)),
			r_in * sin(rad(0 - deg)), 0));
	ikfunctor->p1_final = *(new Vector3d(r_in * cos(rad(0 + deg)),
			r_in * sin(rad(0 + deg)), 0));
	ikfunctor->p2_final = *(new Vector3d(r_in * cos(rad(120 - deg)),
			r_in * sin(rad(120 - deg)), 0));
	ikfunctor->p3_final = *(new Vector3d(r_in * cos(rad(120 + deg)),
			r_in * sin(rad(120 + deg)), 0));
	ikfunctor->p4_final = *(new Vector3d(r_in * cos(rad(240 - deg)),
			r_in * sin(rad(240 - deg)), 0));
	ikfunctor->p5_final = *(new Vector3d(r_in * cos(rad(240 + deg)),
			r_in * sin(rad(240 + deg)), 0));
	ikfunctor->p12_final = *(new Vector3d(r_out * cos(rad(0 - deg)),
			r_out * sin(rad(0 - deg)), 0));
	ikfunctor->p7_final = *(new Vector3d(r_out * cos(rad(0 + deg)),
			r_out * sin(rad(0 + deg)), 0));
	ikfunctor->p8_final = *(new Vector3d(r_out * cos(rad(120 - deg)),
			r_out * sin(rad(120 - deg)), 0));
	ikfunctor->p9_final = *(new Vector3d(r_out * cos(rad(120 + deg)),
			r_out * sin(rad(120 + deg)), 0));
	ikfunctor->p10_final = *(new Vector3d(r_out * cos(rad(240 - deg)),
			r_out * sin(rad(240 - deg)), 0));
	ikfunctor->p11_final = *(new Vector3d(r_out * cos(rad(240 + deg)),
			r_out * sin(rad(240 + deg)), 0));

	//initialize first guess
	for (unsigned int i = 0; i < GS - 12; i++) {
		guess_init[i] = 0;
	}
	for (unsigned int i = GS - 12; i < GS; i++) {
		guess_init[i] = 0.15; //initialize leg lengths to 0.15 m
	}

	// create the ROS service
	solverService = node.advertiseService("solve_ik", &ik::solve_ik, this);

	ROS_INFO("COMA IK Solver Server Started");
}

bool ik::solve_ik(coma_kinematics::solveIK::Request &req,
		coma_kinematics::solveIK::Response &res) {
	Vector3d pd(req.x_pos, req.y_pos, req.z_pos); //desired end effector position
	Vector3d rot(ik::rad(req.x_rot), ik::rad(req.y_rot), ik::rad(req.z_rot));
	Matrix3d Rd = cosserat_rod::hat(rot); //desired end effector orientation
	Eigen::MatrixExponential < Matrix3d > Rdm(Rd);
	Rdm.compute(Rd);
	double leg_lengths[12];

	solve(pd, Rd, leg_lengths);

	for (unsigned int i = 0; i < 12; i++) {
		res.leg_lengths[i] = leg_lengths[i];
	}

	return true;
}

void ik::solve(Vector3d pd, Matrix3d Rd, double* leg_lengths) {
	//set desired forces, moments, position, and rotation
	Vector3d F(0, 0, 0); //applied force at end effector
	Vector3d L(0, 0, 0); //applied moment at end effector

	ikfunctor->F = F;
	ikfunctor->L = L;
	ikfunctor->Rd = Rd;
	ikfunctor->pd = pd;

	//build the problem
	Problem problem;

	//set up the residual
	CostFunction* cost_function = new NumericDiffCostFunction<IKFunctor,
			ceres::CENTRAL, GS - 6, GS>(ikfunctor);
	problem.AddResidualBlock(cost_function, NULL, guess_init);

	//run the solver
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	options.max_num_iterations = 500;
	options.parameter_tolerance = 1E-12;
	options.function_tolerance = 1E-12;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n";

	///solve for the bottom lengths
	Eigen::Matrix<double, 6, 1> bottom_lengths;

	double single_guess_init[7];
	Problem problem_single;
	CostFunction* cost_function_single = new NumericDiffCostFunction<
			SingleIKFunctor, ceres::CENTRAL, 6, 7>(singleikfunctor);
	problem_single.AddResidualBlock(cost_function_single, NULL,
			single_guess_init);

	Eigen::Vector3d p_init[6] = { ikfunctor->p1_init, ikfunctor->p2_init,
			ikfunctor->p3_init, ikfunctor->p4_init, ikfunctor->p5_init,
			ikfunctor->p6_init };
	Eigen::Matrix3d R_final[6] = { ikfunctor->R1_init_s, ikfunctor->R2_init_s,
			ikfunctor->R3_init_s, ikfunctor->R4_init_s, ikfunctor->R5_init_s,
			ikfunctor->R6_init_s };
	Eigen::Vector3d p_final[6] = { ikfunctor->p1_init_s.head<3>(),
			ikfunctor->p2_init_s.head<3>(), ikfunctor->p3_init_s.head<3>(),
			ikfunctor->p4_init_s.head<3>(), ikfunctor->p5_init_s.head<3>(),
			ikfunctor->p6_init_s.head<3>() };

	for (unsigned int j; j < 6; j++) {
		for (unsigned int i = 0; i < 7; i++) {
			single_guess_init[i] = 0;
		}
		single_guess_init[6] = 0.3;
		singleikfunctor->p_init = p_init[j];
		singleikfunctor->R_final = R_final[j];
		singleikfunctor->p_final = p_final[j];
		Solve(options, &problem_single, &summary);
		std::cout << summary.BriefReport() << "\n";
		bottom_lengths[j] = single_guess_init[6];
	}

	//cout << bottom_lengths.transpose() << endl;

	for (unsigned int i = 6 * 12; i < 7 * 12; i++) {
		leg_lengths[i - 72] = guess_init[i]
				+ ((i < 78) ? bottom_lengths[i - 72] : 0);
		cout << leg_lengths[i - 72] << endl;
	}

}

//TODO: rename node "ik_server"
int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "ik");

	//initialize the solver
	ik solver;

	//solve for the starting position
	Matrix3d Rd = cosserat_rod::hat(
			*(new Vector3d(ik::rad(0), ik::rad(0), ik::rad(-60)))); //desired end effector orientation
	Eigen::MatrixExponential < Matrix3d > Rdm(Rd);
	Rdm.compute(Rd);
	Vector3d pd(0.0, 0.0, 0.3); //desired end effector position
	double leg_lengths[12];

	//solve the starting position
	solver.solve(pd, Rd, leg_lengths);

	ros::spin();

	return EXIT_SUCCESS;
}
