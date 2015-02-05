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

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "ceres/ceres.h"
#include "coma_kinematics/cosserat_rod.h"
#include "coma_kinematics/solveIK.h"

#define GS 7*12 //define the guess size
#define INTEGRATION_STEP_SIZE 10

#define use_multithreading
//#define use_matrix_log

class ik {
public:
	ik();
	void solvetest();
	void solve(Eigen::Vector3d pd, Eigen::Matrix3d Rd, double* leg_lengths);
	static double rad(double degrees);

	struct IKFunctor {
		bool operator()(const double* const x, double* residual) const {
			using Eigen::Vector3d;
			using Eigen::Vector4d;
			using Eigen::RowVector4d;
			using Eigen::Matrix;
			using Eigen::Matrix3d;
			using Eigen::Matrix4d;
			typedef Matrix<double, 9, 1> Vector9d;
			typedef Matrix<double, 18, 1> Vector18d;

			//extract leg lengths from guess
			double L1 = x[GS - 12];
			double L2 = x[GS - 11];
			double L3 = x[GS - 10];
			double L4 = x[GS - 9];
			double L5 = x[GS - 8];
			double L6 = x[GS - 7];
			double L7 = x[GS - 6];
			double L8 = x[GS - 5];
			double L9 = x[GS - 4];
			double L10 = x[GS - 3];
			double L11 = x[GS - 2];
			double L12 = x[GS - 1];

			//extract values for top link
			double theta7 = x[41];
			double theta8 = x[47];
			double theta9 = x[53];
			double theta10 = x[59];
			double theta11 = x[65];
			double theta12 = x[71];
			Vector9d R7_init;
			Vector9d R8_init;
			Vector9d R9_init;
			Vector9d R10_init;
			Vector9d R11_init;
			Vector9d R12_init;
			//Matrix3t R7_init; R7_init << cos(theta7), -sin(theta7), 0, sin(theta7), cos(theta7), 0, 0, 0, 1; //this matrix is "reshaped" (converted into a vector column-by-column)
			R7_init << cos(theta7), sin(theta7), 0.0, -sin(theta7), cos(theta7), 0.0, 0.0, 0.0, 1.0;
			R8_init << cos(theta8), sin(theta8), 0.0, -sin(theta8), cos(theta8), 0.0, 0.0, 0.0, 1.0;
			R9_init << cos(theta9), sin(theta9), 0.0, -sin(theta9), cos(theta9), 0.0, 0.0, 0.0, 1.0;
			R10_init << cos(theta10), sin(theta10), 0.0, -sin(theta10), cos(theta10), 0.0, 0.0, 0.0, 1.0;
			R11_init << cos(theta11), sin(theta11), 0.0, -sin(theta11), cos(theta11), 0.0, 0.0, 0.0, 1.0;
			R12_init << cos(theta12), sin(theta12), 0.0, -sin(theta12), cos(theta12), 0.0, 0.0, 0.0, 1.0;
			Vector3d n7_init(x[36], x[37], x[38]);
			Vector3d n8_init(x[42], x[43], x[44]);
			Vector3d n9_init(x[48], x[49], x[50]);
			Vector3d n10_init(x[54], x[55], x[56]);
			Vector3d n11_init(x[60], x[61], x[62]);
			Vector3d n12_init(x[66], x[67], x[68]);
			Vector3d m7_init(x[39], x[40], 0.0);
			Vector3d m8_init(x[45], x[46], 0.0);
			Vector3d m9_init(x[51], x[52], 0.0);
			Vector3d m10_init(x[57], x[58], 0.0);
			Vector3d m11_init(x[63], x[64], 0.0);
			Vector3d m12_init(x[69], x[70], 0.0);
			Vector18d y7_init;
			Vector18d y8_init;
			Vector18d y9_init;
			Vector18d y10_init;
			Vector18d y11_init;
			Vector18d y12_init;
			y7_init << p7_init, R7_init, n7_init, m7_init;
			y8_init << p8_init, R8_init, n8_init, m8_init;
			y9_init << p9_init, R9_init, n9_init, m9_init;
			y10_init << p10_init, R10_init, n10_init, m10_init;
			y11_init << p11_init, R11_init, n11_init, m11_init;
			y12_init << p12_init, R12_init, n12_init, m12_init;

			//perform integration on top link
			cosserat_rod cr7(y7_init);
			cosserat_rod cr8(y8_init);
			cosserat_rod cr9(y9_init);
			cosserat_rod cr10(y10_init);
			cosserat_rod cr11(y11_init);
			cosserat_rod cr12(y12_init);
#ifdef use_multithreading
			boost::thread t7(boost::bind(&cosserat_rod::integrate, &cr7, 0, L7, L7 / INTEGRATION_STEP_SIZE));
			boost::thread t8(boost::bind(&cosserat_rod::integrate, &cr8, 0, L8, L8 / INTEGRATION_STEP_SIZE));
			boost::thread t9(boost::bind(&cosserat_rod::integrate, &cr9, 0, L9, L9 / INTEGRATION_STEP_SIZE));
			boost::thread t10(boost::bind(&cosserat_rod::integrate, &cr10, 0, L10, L10 / INTEGRATION_STEP_SIZE));
			boost::thread t11(boost::bind(&cosserat_rod::integrate, &cr11, 0, L11, L11 / INTEGRATION_STEP_SIZE));
			boost::thread t12(boost::bind(&cosserat_rod::integrate, &cr12, 0, L12, L12 / INTEGRATION_STEP_SIZE));
			t7.join();
			t8.join();
			t9.join();
			t10.join();
			t11.join();
			t12.join();
			Vector18d y7 = cr7.result;
			Vector18d y8 = cr8.result;
			Vector18d y9 = cr9.result;
			Vector18d y10 = cr10.result;
			Vector18d y11 = cr11.result;
			Vector18d y12 = cr12.result;
#else
			Vector18d y7 = cr7.integrate(0, L7, L7 / INTEGRATION_STEP_SIZE);
			Vector18d y8 = cr8.integrate(0, L8, L8 / INTEGRATION_STEP_SIZE);
			Vector18d y9 = cr9.integrate(0, L9, L9 / INTEGRATION_STEP_SIZE);
			Vector18d y10 = cr10.integrate(0, L10, L10 / INTEGRATION_STEP_SIZE);
			Vector18d y11 = cr11.integrate(0, L11, L11 / INTEGRATION_STEP_SIZE);
			Vector18d y12 = cr12.integrate(0, L12, L12 / INTEGRATION_STEP_SIZE);
#endif

			//extract results from top link integration
			Vector3d p7_end(y7[0], y7[1], y7[2]);
			Vector3d p8_end(y8[0], y8[1], y8[2]);
			Vector3d p9_end(y9[0], y9[1], y9[2]);
			Vector3d p10_end(y10[0], y10[1], y10[2]);
			Vector3d p11_end(y11[0], y11[1], y11[2]);
			Vector3d p12_end(y12[0], y12[1], y12[2]);
			Matrix3d R7_end;
			Matrix3d R8_end;
			Matrix3d R9_end;
			Matrix3d R10_end;
			Matrix3d R11_end;
			Matrix3d R12_end;
			R7_end << y7[3], y7[6], y7[9], y7[4], y7[7], y7[10], y7[5], y7[8], y7[11];
			R8_end << y8[3], y8[6], y8[9], y8[4], y8[7], y8[10], y8[5], y8[8], y8[11];
			R9_end << y9[3], y9[6], y9[9], y9[4], y9[7], y9[10], y9[5], y9[8], y9[11];
			R10_end << y10[3], y10[6], y10[9], y10[4], y10[7], y10[10], y10[5], y10[8], y10[11];
			R11_end << y11[3], y11[6], y11[9], y11[4], y11[7], y11[10], y11[5], y11[8], y11[11];
			R12_end << y12[3], y12[6], y12[9], y12[4], y12[7], y12[10], y12[5], y12[8], y12[11];
//			Vector3d n7_end(y7[12], y7[13], y7[14]);
//			Vector3d n8_end(y8[12], y8[13], y8[14]);
//			Vector3d n9_end(y9[12], y9[13], y9[14]);
//			Vector3d n10_end(y10[12], y10[13], y10[14]);
//			Vector3d n11_end(y11[12], y11[13], y11[14]);
//			Vector3d n712_end(y12[12], y12[13], y12[14]);
//			Vector3d m7_end(y7[15], y7[16], y7[17]);
//			Vector3d m8_end(y8[15], y8[16], y8[17]);
//			Vector3d m9_end(y9[15], y9[16], y9[17]);
//			Vector3d m10_end(y10[15], y10[16], y10[17]);
//			Vector3d m11_end(y11[15], y11[16], y11[17]);
//			Vector3d m12_end(y12[15], y12[16], y12[17]);

//extract values for bottom link
			double theta1 = x[5];
			double theta2 = x[11];
			double theta3 = x[17];
			double theta4 = x[23];
			double theta5 = x[29];
			double theta6 = x[35];

			Eigen::Matrix3d R1_init_m;
			Eigen::Matrix3d R2_init_m;
			Eigen::Matrix3d R3_init_m;
			Eigen::Matrix3d R4_init_m;
			Eigen::Matrix3d R5_init_m;
			Eigen::Matrix3d R6_init_m;
			R1_init_m << cos(theta1), -sin(theta1), 0.0, sin(theta1), cos(theta1), 0.0, 0.0, 0.0, 1.0;
			R2_init_m << cos(theta2), -sin(theta2), 0.0, sin(theta2), cos(theta2), 0.0, 0.0, 0.0, 1.0;
			R3_init_m << cos(theta3), -sin(theta3), 0.0, sin(theta3), cos(theta3), 0.0, 0.0, 0.0, 1.0;
			R4_init_m << cos(theta4), -sin(theta4), 0.0, sin(theta4), cos(theta4), 0.0, 0.0, 0.0, 1.0;
			R5_init_m << cos(theta5), -sin(theta5), 0.0, sin(theta5), cos(theta5), 0.0, 0.0, 0.0, 1.0;
			R6_init_m << cos(theta6), -sin(theta6), 0.0, sin(theta6), cos(theta6), 0.0, 0.0, 0.0, 1.0;
			R1_init_m = R1_init_m * R7_end;
			R2_init_m = R2_init_m * R7_end;
			R3_init_m = R3_init_m * R7_end;
			R4_init_m = R4_init_m * R7_end;
			R5_init_m = R5_init_m * R7_end;
			R6_init_m = R6_init_m * R7_end;
			const_cast<IKFunctor*>(this)->R1_init_s = R1_init_m;
			const_cast<IKFunctor*>(this)->R2_init_s = R2_init_m;
			const_cast<IKFunctor*>(this)->R3_init_s = R3_init_m;
			const_cast<IKFunctor*>(this)->R4_init_s = R4_init_m;
			const_cast<IKFunctor*>(this)->R5_init_s = R5_init_m;
			const_cast<IKFunctor*>(this)->R6_init_s = R6_init_m;
			Vector9d R1_init;
			Vector9d R2_init;
			Vector9d R3_init;
			Vector9d R4_init;
			Vector9d R5_init;
			Vector9d R6_init;
			R1_init << R1_init_m(0, 0), R1_init_m(1, 0), R1_init_m(2, 0), R1_init_m(0, 1), R1_init_m(1, 1), R1_init_m(2, 1), R1_init_m(0, 2), R1_init_m(1, 2), R1_init_m(
					2, 2);
			R2_init << R2_init_m(0, 0), R2_init_m(1, 0), R2_init_m(2, 0), R2_init_m(0, 1), R2_init_m(1, 1), R2_init_m(2, 1), R2_init_m(0, 2), R2_init_m(1, 2), R2_init_m(
					2, 2);
			R3_init << R3_init_m(0, 0), R3_init_m(1, 0), R3_init_m(2, 0), R3_init_m(0, 1), R3_init_m(1, 1), R3_init_m(2, 1), R3_init_m(0, 2), R3_init_m(1, 2), R3_init_m(
					2, 2);
			R4_init << R4_init_m(0, 0), R4_init_m(1, 0), R4_init_m(2, 0), R4_init_m(0, 1), R4_init_m(1, 1), R4_init_m(2, 1), R4_init_m(0, 2), R4_init_m(1, 2), R4_init_m(
					2, 2);
			R5_init << R5_init_m(0, 0), R5_init_m(1, 0), R5_init_m(2, 0), R5_init_m(0, 1), R5_init_m(1, 1), R5_init_m(2, 1), R5_init_m(0, 2), R5_init_m(1, 2), R5_init_m(
					2, 2);
			R6_init << R6_init_m(0, 0), R6_init_m(1, 0), R6_init_m(2, 0), R6_init_m(0, 1), R6_init_m(1, 1), R6_init_m(2, 1), R6_init_m(0, 2), R6_init_m(1, 2), R6_init_m(
					2, 2);
			Vector3d n1_init(x[0], x[1], x[2]);
			Vector3d n2_init(x[6], x[7], x[8]);
			Vector3d n3_init(x[12], x[13], x[14]);
			Vector3d n4_init(x[18], x[19], x[20]);
			Vector3d n5_init(x[24], x[25], x[26]);
			Vector3d n6_init(x[30], x[31], x[32]);
			Vector3d m1_init(x[3], x[4], 0);
			Vector3d m2_init(x[9], x[10], 0);
			Vector3d m3_init(x[15], x[16], 0);
			Vector3d m4_init(x[21], x[22], 0);
			Vector3d m5_init(x[27], x[28], 0);
			Vector3d m6_init(x[33], x[34], 0);

			//centroid of all the bottom segment leg ends
			Vector3d p_cb = (p7_end + p8_end + p9_end + p10_end + p11_end + p12_end) / 6;

			//transformation from base to mid_plate
			Matrix4d T_mid;
			T_mid.block<3, 3>(0, 0) = R7_end;
			T_mid.block<3, 1>(0, 3) = p_cb;
			RowVector4d zzzo(0, 0, 0, 1);
			T_mid.block<1, 4>(3, 0) = zzzo;
			Vector4d p1_init_2(p1_init(0), p1_init(1), p1_init(2), 1);
			Vector4d p2_init_2(p2_init(0), p2_init(1), p2_init(2), 1);
			Vector4d p3_init_2(p3_init(0), p3_init(1), p3_init(2), 1);
			Vector4d p4_init_2(p4_init(0), p4_init(1), p4_init(2), 1);
			Vector4d p5_init_2(p5_init(0), p5_init(1), p5_init(2), 1);
			Vector4d p6_init_2(p6_init(0), p6_init(1), p6_init(2), 1);
			p1_init_2 = T_mid * p1_init_2;
			p2_init_2 = T_mid * p2_init_2;
			p3_init_2 = T_mid * p3_init_2;
			p4_init_2 = T_mid * p4_init_2;
			p5_init_2 = T_mid * p5_init_2;
			p6_init_2 = T_mid * p6_init_2;
			const_cast<IKFunctor*>(this)->p1_init_s = p1_init_2;
			const_cast<IKFunctor*>(this)->p2_init_s = p2_init_2;
			const_cast<IKFunctor*>(this)->p3_init_s = p3_init_2;
			const_cast<IKFunctor*>(this)->p4_init_s = p4_init_2;
			const_cast<IKFunctor*>(this)->p5_init_s = p5_init_2;
			const_cast<IKFunctor*>(this)->p6_init_s = p6_init_2;
			Vector18d y1_init;
			Vector18d y2_init;
			Vector18d y3_init;
			Vector18d y4_init;
			Vector18d y5_init;
			Vector18d y6_init;
			y1_init << p1_init_2.head<3>(), R1_init, n1_init, m1_init;
			y2_init << p2_init_2.head<3>(), R2_init, n2_init, m2_init;
			y3_init << p3_init_2.head<3>(), R3_init, n3_init, m3_init;
			y4_init << p4_init_2.head<3>(), R4_init, n4_init, m4_init;
			y5_init << p5_init_2.head<3>(), R5_init, n5_init, m5_init;
			y6_init << p6_init_2.head<3>(), R6_init, n6_init, m6_init;

			//perform integration on bottom link
			cosserat_rod cr1(y1_init);
			cosserat_rod cr2(y2_init);
			cosserat_rod cr3(y3_init);
			cosserat_rod cr4(y4_init);
			cosserat_rod cr5(y5_init);
			cosserat_rod cr6(y6_init);
#ifdef use_multithreading
			boost::thread t1(boost::bind(&cosserat_rod::integrate, &cr1, 0, L1, L1 / INTEGRATION_STEP_SIZE));
			boost::thread t2(boost::bind(&cosserat_rod::integrate, &cr2, 0, L2, L2 / INTEGRATION_STEP_SIZE));
			boost::thread t3(boost::bind(&cosserat_rod::integrate, &cr3, 0, L3, L3 / INTEGRATION_STEP_SIZE));
			boost::thread t4(boost::bind(&cosserat_rod::integrate, &cr4, 0, L4, L4 / INTEGRATION_STEP_SIZE));
			boost::thread t5(boost::bind(&cosserat_rod::integrate, &cr5, 0, L5, L5 / INTEGRATION_STEP_SIZE));
			boost::thread t6(boost::bind(&cosserat_rod::integrate, &cr6, 0, L6, L6 / INTEGRATION_STEP_SIZE));
			t1.join();
			t2.join();
			t3.join();
			t4.join();
			t5.join();
			t6.join();
			Vector18d y1 = cr1.result;
			Vector18d y2 = cr2.result;
			Vector18d y3 = cr3.result;
			Vector18d y4 = cr4.result;
			Vector18d y5 = cr5.result;
			Vector18d y6 = cr6.result;
#else
			Vector18d y1 = cr1.integrate(0, L1, L1 / INTEGRATION_STEP_SIZE);
			Vector18d y2 = cr2.integrate(0, L2, L2 / INTEGRATION_STEP_SIZE);
			Vector18d y3 = cr3.integrate(0, L3, L3 / INTEGRATION_STEP_SIZE);
			Vector18d y4 = cr4.integrate(0, L4, L4 / INTEGRATION_STEP_SIZE);
			Vector18d y5 = cr5.integrate(0, L5, L5 / INTEGRATION_STEP_SIZE);
			Vector18d y6 = cr6.integrate(0, L6, L6 / INTEGRATION_STEP_SIZE);
#endif

			//extract results from bottom link integration
			Vector3d p1_end(y1[0], y1[1], y1[2]);
			Vector3d p2_end(y2[0], y2[1], y2[2]);
			Vector3d p3_end(y3[0], y3[1], y3[2]);
			Vector3d p4_end(y4[0], y4[1], y4[2]);
			Vector3d p5_end(y5[0], y5[1], y5[2]);
			Vector3d p6_end(y6[0], y6[1], y6[2]);

			Matrix3d R1_end;
			Matrix3d R2_end;
			Matrix3d R3_end;
			Matrix3d R4_end;
			Matrix3d R5_end;
			Matrix3d R6_end;
			R1_end << y1[3], y1[6], y1[9], y1[4], y1[7], y1[10], y1[5], y1[8], y1[11];
			R2_end << y2[3], y2[6], y2[9], y2[4], y2[7], y2[10], y2[5], y2[8], y2[11];
			R3_end << y3[3], y3[6], y3[9], y3[4], y3[7], y3[10], y3[5], y3[8], y3[11];
			R4_end << y4[3], y4[6], y4[9], y4[4], y4[7], y4[10], y4[5], y4[8], y4[11];
			R5_end << y5[3], y5[6], y5[9], y5[4], y5[7], y5[10], y5[5], y5[8], y5[11];
			R6_end << y6[3], y6[6], y6[9], y6[4], y6[7], y6[10], y6[5], y6[8], y6[11];
			Vector3d n1_end(y1[12], y1[13], y1[14]);
			Vector3d n2_end(y2[12], y2[13], y2[14]);
			Vector3d n3_end(y3[12], y3[13], y3[14]);
			Vector3d n4_end(y4[12], y4[13], y4[14]);
			Vector3d n5_end(y5[12], y5[13], y5[14]);
			Vector3d n6_end(y6[12], y6[13], y6[14]);
			Vector3d m1_end(y1[15], y1[16], y1[17]);
			Vector3d m2_end(y2[15], y2[16], y2[17]);
			Vector3d m3_end(y3[15], y3[16], y3[17]);
			Vector3d m4_end(y4[15], y4[16], y4[17]);
			Vector3d m5_end(y5[15], y5[16], y5[17]);
			Vector3d m6_end(y6[15], y6[16], y6[17]);

			//centroid of all top segment leg ends
			Vector3d p_ct = (p1_end + p2_end + p3_end + p4_end + p5_end + p6_end) / 6;

			//residual of equilibrium conditions
			Vector3d res_eq_F_top = (n1_end + n2_end + n3_end + n4_end + n5_end + n6_end) - F;
			Vector3d res_eq_L_top = (cosserat_rod::hat(p1_end - p_ct) * n1_end + cosserat_rod::hat(p2_end - p_ct) * n2_end
					+ cosserat_rod::hat(p3_end - p_ct) * n3_end + cosserat_rod::hat(p4_end - p_ct) * n4_end + cosserat_rod::hat(p5_end - p_ct) * n5_end
					+ cosserat_rod::hat(p6_end - p_ct) * n6_end + m1_end + m2_end + m3_end + m4_end + m5_end + m6_end) - L;

			//These are analogous to loop closure equations because they are only satisfied when the positions of the rod ends have the same relative positions as the connection pattern in the top plate
			Vector3d res_p1 = pd + Rd * p1_final - p1_end;
			Vector3d res_p2 = pd + Rd * p2_final - p2_end;
			Vector3d res_p3 = pd + Rd * p3_final - p3_end;
			Vector3d res_p4 = pd + Rd * p4_final - p4_end;
			Vector3d res_p5 = pd + Rd * p5_final - p5_end;
			Vector3d res_p6 = pd + Rd * p6_final - p6_end;
			Vector3d res_p7 = p7_end - R7_end * (p7_final - p7_final) - p7_end;
			Vector3d res_p8 = p7_end - R7_end * (p7_final - p8_final) - p8_end;
			Vector3d res_p9 = p7_end - R7_end * (p7_final - p9_final) - p9_end;
			Vector3d res_p10 = p7_end - R7_end * (p7_final - p10_final) - p10_end;
			Vector3d res_p11 = p7_end - R7_end * (p7_final - p11_final) - p11_end;
			Vector3d res_p12 = p7_end - R7_end * (p7_final - p12_final) - p12_end;

			//force a common material orientation for all the distal rod ends
#ifdef use_matrix_log
			Vector3d res_R1(0.0, 0.0, 0.0);
			Vector3d res_R2(0.0, 0.0, 0.0);
			Vector3d res_R3(0.0, 0.0, 0.0);
			Vector3d res_R4(0.0, 0.0, 0.0);
			Vector3d res_R5(0.0, 0.0, 0.0);
			Vector3d res_R6(0.0, 0.0, 0.0);
			Vector3d res_R7(0.0, 0.0, 0.0);
			Vector3d res_R8(0.0, 0.0, 0.0);
			Vector3d res_R9(0.0, 0.0, 0.0);
			Vector3d res_R10(0.0, 0.0, 0.0);
			Vector3d res_R11(0.0, 0.0, 0.0);
			Vector3d res_R12(0.0, 0.0, 0.0);
			Matrix3d Rdt = Rd.transpose();
			if (!(Rdt * R1_end).isZero()) res_R1 = cosserat_rod::vee((Rdt * R1_end).log());
			if (!(Rdt * R2_end).isZero()) res_R2 = cosserat_rod::vee((Rdt * R2_end).log());
			if (!(Rdt * R3_end).isZero()) res_R3 = cosserat_rod::vee((Rdt * R3_end).log());
			if (!(Rdt * R4_end).isZero()) res_R4 = cosserat_rod::vee((Rdt * R4_end).log());
			if (!(Rdt * R5_end).isZero()) res_R5 = cosserat_rod::vee((Rdt * R5_end).log());
			if (!(Rdt * R6_end).isZero()) res_R6 = cosserat_rod::vee((Rdt * R6_end).log());
			if (!(R7_end.transpose() * R7_end).isZero()) res_R7 = cosserat_rod::vee((R7_end.transpose() * R7_end).log());
			if (!(R8_end.transpose() * R7_end).isZero()) res_R8 = cosserat_rod::vee((R8_end.transpose() * R7_end).log());
			if (!(R9_end.transpose() * R7_end).isZero()) res_R9 = cosserat_rod::vee((R9_end.transpose() * R7_end).log());
			if (!(R10_end.transpose() * R7_end).isZero()) res_R10 = cosserat_rod::vee((R10_end.transpose() * R7_end).log());
			if (!(R11_end.transpose() * R7_end).isZero()) res_R11 = cosserat_rod::vee((R11_end.transpose() * R7_end).log());
			if (!(R12_end.transpose() * R7_end).isZero()) res_R12 = cosserat_rod::vee((R12_end.transpose() * R7_end).log());
#else
			Matrix<double, 2, 3> rodrigues;
			rodrigues << 1, 0, 0, 0, 1, 0;
			Matrix3d Rdt = Rd.transpose();
			Vector3d res_R1;
			Vector3d res_R2;
			Vector3d res_R3;
			Vector3d res_R4;
			Vector3d res_R5;
			Vector3d res_R6;
			Vector3d res_R7;
			Vector3d res_R8;
			Vector3d res_R9;
			Vector3d res_R10;
			Vector3d res_R11;
			Vector3d res_R12;
			res_R1 << rodrigues * cosserat_rod::vee((R1_end.transpose() * Rd - R1_end * Rdt)), 0;
			res_R2 << rodrigues * cosserat_rod::vee((R2_end.transpose() * Rd - R2_end * Rdt)), 0;
			res_R3 << rodrigues * cosserat_rod::vee((R3_end.transpose() * Rd - R3_end * Rdt)), 0;
			res_R4 << rodrigues * cosserat_rod::vee((R4_end.transpose() * Rd - R4_end * Rdt)), 0;
			res_R5 << rodrigues * cosserat_rod::vee((R5_end.transpose() * Rd - R5_end * Rdt)), 0;
			res_R6 << rodrigues * cosserat_rod::vee((R6_end.transpose() * Rd - R6_end * Rdt)), 0;
			res_R7 << rodrigues * cosserat_rod::vee((R7_end.transpose() * R7_end - R7_end * R7_end.transpose())), 0;
			res_R8 << rodrigues * cosserat_rod::vee((R8_end.transpose() * R7_end - R8_end * R7_end.transpose())), 0;
			res_R9 << rodrigues * cosserat_rod::vee((R9_end.transpose() * R7_end - R9_end * R7_end.transpose())), 0;
			res_R10 << rodrigues * cosserat_rod::vee((R10_end.transpose() * R7_end - R10_end * R7_end.transpose())), 0;
			res_R11 << rodrigues * cosserat_rod::vee((R11_end.transpose() * R7_end - R11_end * R7_end.transpose())), 0;
			res_R12 << rodrigues * cosserat_rod::vee((R12_end.transpose() * R7_end - R12_end * R7_end.transpose())), 0;
#endif

			double l_c = 0.01; //characteristic length converts rotation error to meters
			Matrix<double, GS - 6, 1> res;
			res << res_eq_F_top, res_eq_L_top, res_p1, res_p2, res_p3, res_p4, res_p5, res_p6, res_p7, res_p8, res_p9, res_p10, res_p11, res_p12, res_R1 * l_c, res_R2
					* l_c, res_R3 * l_c, res_R4 * l_c, res_R5 * l_c, res_R6 * l_c, res_R7 * l_c, res_R8 * l_c, res_R9 * l_c, res_R10 * l_c, res_R11 * l_c, res_R12
					* l_c;

			for (unsigned int i = 0; i < GS - 6; i++) {
				residual[i] = res[i];
			}

			return true;
		}

		//desired positions and forces
		Eigen::Vector3d F;  //applied force at end effector
		Eigen::Vector3d L;  //applied moment at end effector
		Eigen::Matrix3d Rd; //desired end effector orientation
		Eigen::Vector3d pd; //desired end effector position

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

		Eigen::Matrix3d R1_init_s;
		Eigen::Matrix3d R2_init_s;
		Eigen::Matrix3d R3_init_s;
		Eigen::Matrix3d R4_init_s;
		Eigen::Matrix3d R5_init_s;
		Eigen::Matrix3d R6_init_s;

		Eigen::Vector4d p1_init_s;
		Eigen::Vector4d p2_init_s;
		Eigen::Vector4d p3_init_s;
		Eigen::Vector4d p4_init_s;
		Eigen::Vector4d p5_init_s;
		Eigen::Vector4d p6_init_s;
	};

	struct SingleIKFunctor {
		bool operator()(const double* const x, double* residual) const {
			using Eigen::Vector3d;
			using Eigen::Vector4d;
			using Eigen::RowVector4d;
			using Eigen::Matrix;
			using Eigen::Matrix3d;
			using Eigen::Matrix4d;
			typedef Matrix<double, 9, 1> Vector9d;
			typedef Matrix<double, 18, 1> Vector18d;
			using std::cout;
			using std::endl;

			double L = x[6];
			double theta = x[5];
			Vector9d R_init;
			R_init << cos(theta), sin(theta), 0.0, -sin(theta), cos(theta), 0.0, 0.0, 0.0, 1.0;
			Vector3d n_init(x[0], x[1], x[2]);
			Vector3d m_init(x[3], x[4], 0);

			Vector18d y_init;
			y_init << p_init, R_init, n_init, m_init;

			cosserat_rod cr(y_init);
			Vector18d y = cr.integrate(0, L, L / INTEGRATION_STEP_SIZE);

			Vector3d p_end(y[0], y[1], y[2]);
			Matrix3d R_end;
			R_end << y[3], y[6], y[9], y[4], y[7], y[10], y[5], y[8], y[11];
			//Vector3d n_end(y[12], y[13], y[14]);
			//Vector3d m_end(y[15], y[16], y[17]);

			Vector3d res_p = p_final - p_end;
//			Matrix3d R_final3x3;
//			R_final3x3 << R_Final[0], R_Final[3], R_Final[6], R_Final[1], R_Final[4], R_Final[7], R_Final[2], R_Final[5], R_Final[8];

#ifdef use_matrix_log
			Vector3d res_R(0.0, 0.0, 0.0);
			if (!(R_final.transpose() * R_end).isZero()) res_R = cosserat_rod::vee((R_final.transpose() * R_end).log());
#else
			Matrix<double, 2, 3> rodrigues;
			rodrigues << 1, 0, 0, 0, 1, 0;
			Vector3d res_R;
			res_R << rodrigues * cosserat_rod::vee((R_end.transpose() * R_final - R_end * R_final.transpose())), 0;
#endif

			double l_c = 0.01; //characteristic length converts rotation error to meters
			Matrix<double, 6, 1> res;
			res << res_p, res_R * l_c;

			for (unsigned int i = 0; i < 6; i++) {
				residual[i] = res[i];
			}

			return true;
		}
		Eigen::Vector3d p_init;
		Eigen::Vector3d p_final;
		Eigen::Matrix3d R_final;
	};

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

	IKFunctor* ikfunctor;
	SingleIKFunctor* singleikfunctor;
	double guess_init[GS];
	double single_guess_init[7];
	ceres::Problem problem;
	ceres::Problem problem_single;
	ceres::CostFunction* cost_function;
	ceres::CostFunction* cost_function_single;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	ros::ServiceServer solverService;

	bool solve_ik(coma_kinematics::solveIK::Request &req, coma_kinematics::solveIK::Response &res);

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
