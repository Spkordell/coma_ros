/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2015
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

double ik::deg(double radians) {
	return radians * (180.0 / M_PI);
}

Eigen::Matrix<double, 3, 3> ik::hat(Eigen::Matrix<double, 3, 1> u) {
	Eigen::Matrix<double, 3, 3> uhat;
	uhat << 0, -u(2), u(1), u(2), 0, -u(0), -u(1), u(0), 0;
	return uhat;
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
	ikfunctor->p1_init_ = *(new Vector3d(r_in * cos(rad(0 - deg)), r_in * sin(rad(0 - deg)), 0));
	ikfunctor->p2_init_ = *(new Vector3d(r_in * cos(rad(0 + deg)), r_in * sin(rad(0 + deg)), 0));
	ikfunctor->p3_init_ = *(new Vector3d(r_in * cos(rad(120 - deg)), r_in * sin(rad(120 - deg)), 0));
	ikfunctor->p4_init_ = *(new Vector3d(r_in * cos(rad(120 + deg)), r_in * sin(rad(120 + deg)), 0));
	ikfunctor->p5_init_ = *(new Vector3d(r_in * cos(rad(240 - deg)), r_in * sin(rad(240 - deg)), 0));
	ikfunctor->p6_init_ = *(new Vector3d(r_in * cos(rad(240 + deg)), r_in * sin(rad(240 + deg)), 0));
	ikfunctor->p7_init_ = *(new Vector3d(r_out * cos(rad(60 - deg)), r_out * sin(rad(60 - deg)), 0));
	ikfunctor->p8_init_ = *(new Vector3d(r_out * cos(rad(60 + deg)), r_out * sin(rad(60 + deg)), 0));
	ikfunctor->p9_init_ = *(new Vector3d(r_out * cos(rad(180 - deg)), r_out * sin(rad(180 - deg)), 0));
	ikfunctor->p10_init_ = *(new Vector3d(r_out * cos(rad(180 + deg)), r_out * sin(rad(180 + deg)), 0));
	ikfunctor->p11_init_ = *(new Vector3d(r_out * cos(rad(300 - deg)), r_out * sin(rad(300 - deg)), 0));
	ikfunctor->p12_init_ = *(new Vector3d(r_out * cos(rad(300 + deg)), r_out * sin(rad(300 + deg)), 0));

	//position of the leg constraints in the local frame at the top
	ikfunctor->p6_final_ = *(new Vector3d(r_in * cos(rad(0 - deg)), r_in * sin(rad(0 - deg)), 0));
	ikfunctor->p1_final_ = *(new Vector3d(r_in * cos(rad(0 + deg)), r_in * sin(rad(0 + deg)), 0));
	ikfunctor->p2_final_ = *(new Vector3d(r_in * cos(rad(120 - deg)), r_in * sin(rad(120 - deg)), 0));
	ikfunctor->p3_final_ = *(new Vector3d(r_in * cos(rad(120 + deg)), r_in * sin(rad(120 + deg)), 0));
	ikfunctor->p4_final_ = *(new Vector3d(r_in * cos(rad(240 - deg)), r_in * sin(rad(240 - deg)), 0));
	ikfunctor->p5_final_ = *(new Vector3d(r_in * cos(rad(240 + deg)), r_in * sin(rad(240 + deg)), 0));
	ikfunctor->p12_final_ = *(new Vector3d(r_out * cos(rad(0 - deg)), r_out * sin(rad(0 - deg)), 0));
	ikfunctor->p7_final_ = *(new Vector3d(r_out * cos(rad(0 + deg)), r_out * sin(rad(0 + deg)), 0));
	ikfunctor->p8_final_ = *(new Vector3d(r_out * cos(rad(120 - deg)), r_out * sin(rad(120 - deg)), 0));
	ikfunctor->p9_final_ = *(new Vector3d(r_out * cos(rad(120 + deg)), r_out * sin(rad(120 + deg)), 0));
	ikfunctor->p10_final_ = *(new Vector3d(r_out * cos(rad(240 - deg)), r_out * sin(rad(240 - deg)), 0));
	ikfunctor->p11_final_ = *(new Vector3d(r_out * cos(rad(240 + deg)), r_out * sin(rad(240 + deg)), 0));

	ikfunctor->visualization_enabled = true;

	//initialize first guess
	for (unsigned int i = 0; i < GS - 12; i++) {
		guess_init[i] = 0;
	}
	for (unsigned int i = GS - 6; i < GS; i++) {
		//guess_init[i] = MIN_LEG_LENGTH_BOTTOM+1E-12; //initialize bottom leg lengths
		guess_init[i] = 0.3;
	}
	for (unsigned int i = GS - 12; i < GS-6; i++) {
		//guess_init[i] = MIN_LEG_LENGTH_TOP+1E-12; //initialize top leg lengths
		guess_init[i] = 0.3;
	}


	for (unsigned int rod = 0; rod < 6; rod++) {
		for (unsigned int i = 0; i < 7; i++) {
			single_guess_init[rod][i] = 0;
		}
		single_guess_init[rod][6] = 0.3;
	}

	//initialize solver
	cost_function = new AutoDiffCostFunction<IKFunctor, GS - 6, GS>(ikfunctor);
	problem.AddResidualBlock(cost_function, NULL, guess_init);

	for (unsigned int rod = 0; rod < 6; rod++) {
		cost_function_single[rod] = new AutoDiffCostFunction<SingleIKFunctor, 6, 7>(singleikfunctor);
		problem_single[rod].AddResidualBlock(cost_function_single[rod], NULL, single_guess_init[rod]);
	}

	options.minimizer_progress_to_stdout = false;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	//options.linear_solver_type = ceres::DENSE_QR;
	options.num_threads = 2;
	options.max_num_iterations = 500;
	options.function_tolerance = 1E-99;
	options.parameter_tolerance = 1E-30;


	// create the ROS service
	solverService = node.advertiseService("solve_ik", &ik::solve_ik, this);

	if (ikfunctor->visualization_enabled) {
		vis_pub = node.advertise<coma_rviz::vis>("rod_pos", 350);
	}

	ROS_INFO("COMA IK Solver Server Started");
}

bool ik::solve_ik(coma_kinematics::solveIK::Request &req, coma_kinematics::solveIK::Response &res) {
	Vector3d pd(req.x_pos, req.y_pos, req.z_pos); //desired end effector position
	Vector3d rot(ik::rad(req.x_rot), ik::rad(req.y_rot), ik::rad(req.z_rot));
	Matrix3d Rd = ik::hat(rot); //desired end effector orientation
	Eigen::MatrixExponential < Matrix3d > Rdm(Rd);
	Rdm.compute(Rd);
	double leg_lengths[12];
	double wrist_angles[2];

	for (unsigned int i = 0; i < GS - 12; i++) {
		guess_init[i] = 0;
	}
	for (unsigned int i = GS - 6; i < GS; i++) {
		guess_init[i] = 0.3; //bottom
	}
	for (unsigned int i = GS - 12; i < GS-6; i++) {
		guess_init[i] = 0.3; //top
	}

	solve(pd, Rd, leg_lengths, wrist_angles);

	for (unsigned int i = 0; i < 12; i++) {
		res.leg_lengths[i] = leg_lengths[i];
	}
	res.wrist_rot = wrist_angles[0];
	res.wrist_flex = wrist_angles[1];

	if (ikfunctor->visualization_enabled) {
		coma_rviz::vis viz;
		for (unsigned int i = 0; i < 12; i++) {
			viz.rod[i].x = std::vector<double>(INTEGRATION_STEPS+1);
			viz.rod[i].y = std::vector<double>(INTEGRATION_STEPS+1);
			viz.rod[i].z = std::vector<double>(INTEGRATION_STEPS+1);
			for (unsigned int j = 0; j < INTEGRATION_STEPS+1; j++) {
				viz.rod[i].x[j] = ikfunctor->rodpositions[i](j,0);
				viz.rod[i].y[j] = ikfunctor->rodpositions[i](j,1);
				viz.rod[i].z[j] = ikfunctor->rodpositions[i](j,2);
			}
		}
		vis_pub.publish(viz);
	}
	return true;
}

void ik::solve(Vector3d pd, Matrix3d Rd, double* leg_lengths, double* wrist_angles) {

	using Eigen::Vector3d;
	using Eigen::Matrix3d;

	//set desired forces, moments, position, and rotation
	Vector3d F(0, 0, 0); //applied force at end effector
	Vector3d L(0, 0, 0); //applied moment at end effector

	ikfunctor->F_ = F;
	ikfunctor->L_ = L;
	ikfunctor->Rd_ = Rd;
	ikfunctor->pd_ = pd;

	//run the solver
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	//std:: cout << summary.FullReport() << std::endl;


	///solve for the bottom lengths
	Eigen::Matrix<double, 6, 1> bottom_lengths;

	Eigen::Vector3d p_init[6] = { ikfunctor->p1_init_, ikfunctor->p2_init_, ikfunctor->p3_init_, ikfunctor->p4_init_, ikfunctor->p5_init_, ikfunctor->p6_init_ };
	Eigen::Matrix3d R_final[6] = { ikfunctor->R1_init_s, ikfunctor->R2_init_s, ikfunctor->R3_init_s, ikfunctor->R4_init_s, ikfunctor->R5_init_s,
			ikfunctor->R6_init_s };
	Eigen::Vector3d p_final[6] = { ikfunctor->p1_init_s.head(3), ikfunctor->p2_init_s.head(3), ikfunctor->p3_init_s.head(3), ikfunctor->p4_init_s.head(3),
			ikfunctor->p5_init_s.head(3), ikfunctor->p6_init_s.head(3) };

	for (unsigned int rod = 0; rod < 6; rod++) {
		singleikfunctor->p_init_ = p_init[rod];
		singleikfunctor->R_final_ = R_final[rod];
		singleikfunctor->p_final_ = p_final[rod];
		Solve(options, &(problem_single[rod]), &summary);
		//std::cout << summary.BriefReport() << "\n";
		bottom_lengths[rod] = single_guess_init[rod][6];
	}

#ifdef INCLUDE_WRIST
	for (unsigned int rod = 0; rod < 12; rod++) {
		leg_lengths[rod] = guess_init[(6 * 12 + 2) + rod] + ((rod < 6) ? bottom_lengths[rod] : 0);
	}
	wrist_angles[0] = guess_init[GS-13]; //wrist rotation
	wrist_angles[1] = guess_init[GS-14]; //wrist flex
	//std::cout << "rot: " << deg(guess_init[GS-13]) << std::endl;
	//std::cout << "flex: " << deg(guess_init[GS-14]) << std::endl;
#else
	for (unsigned int rod = 0; rod < 12; rod++) {
		leg_lengths[rod] = guess_init[(6 * 12) + rod] + ((rod < 6) ? bottom_lengths[rod] : 0);
	}
#endif

}

//TODO: rename node "ik_server"
int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "ik");

	//initialize the solver
	ik solver;

	//solve for the starting position
//	Matrix3d Rd = cosserat_rod::hat(
//			*(new Vector3d(ik::rad(0), ik::rad(0), ik::rad(-60)))); //desired end effector orientation
//	Eigen::MatrixExponential < Matrix3d > Rdm(Rd);
//	Rdm.compute(Rd);
//	Vector3d pd(0.0, 0.0, 0.3); //desired end effector position
//	double leg_lengths[12];
//
//	//solve the starting position
//	solver.solve(pd, Rd, leg_lengths);

	ros::spin();

	return EXIT_SUCCESS;
}
