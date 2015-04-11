/*!
 * \ik.cpp
 * \brief Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * Solves two-segment inverse kinematics for parallel continuum manipulators
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 18, 2015
 */

#ifndef IK_H_
#define IK_H_

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "ceres/ceres.h"
#include "coma_kinematics/cosserat_rod.h"
#include "coma_kinematics/solveIK.h"
#include "coma_kinematics/defines.h"
#include "coma_rviz/vis.h"

template<typename T> struct identity {
	typedef T type;
};

class ik {
public:
	ik();
	void solvetest();
	void solve(Eigen::Vector3d pd, Eigen::Matrix3d Rd, double* leg_lengths, double* wrist_angles);
	static double rad(double degrees);
	static double deg(double radians);
	Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> u);

	template<typename T> inline static Eigen::Matrix<T, 3, 3> M3DtoT(Eigen::Matrix<double, 3, 3> a) {
		Eigen::Matrix<T, 3, 3> b;
		b << T(a(0, 0)), T(a(0, 1)), T(a(0, 2)), T(a(1, 0)), T(a(1, 1)), T(a(1, 2)), T(a(2, 0)), T(a(2, 1)), T(a(2, 2));
		return b;
	}
	template<typename T> inline static Eigen::Matrix<T, 3, 1> V3DtoT(Eigen::Matrix<double, 3, 1> a) {
		Eigen::Matrix<T, 3, 1> b;
		b << T(a[0]), T(a[1]), T(a[2]);
		return b;
	}
	template<typename T> inline static Eigen::Matrix<T, 4, 1> V4DtoT(Eigen::Matrix<double, 4, 1> a) {
		Eigen::Matrix<T, 4, 1> b;
		b << T(a[0]), T(a[1]), T(a[2]), T(a[3]);
		return b;
	}

	struct IKFunctor {
		template<typename T>
		bool operator()(const T* const x, T* residual) const {
			//last 12 indices of x are leg lengths
#ifdef INCLUDE_WRIST
			//index GS-13 is wrist roll
			//index GS-14 is wrist flex
#endif

			using Eigen::Matrix;
			typedef Matrix<T, 3, 3> Matrix3t;
			typedef Matrix<T, 4, 4> Matrix4t;
			typedef Matrix<T, 3, 1> Vector3t;
			typedef Matrix<T, 4, 1> Vector4t;
			typedef Matrix<T, 1, 4> RowVector4t;
			typedef Matrix<T, 9, 1> Vector9t;
			typedef Matrix<T, 18, 1> Vector18t;

			//convert double valued matrices into T type matrices
			//desired positions and forces
			Vector3t F = V3DtoT<T>(F_);  //applied force at end effector
			Vector3t L = V3DtoT<T>(L_);  //applied moment at end effector
			Matrix3t Rd = M3DtoT<T>(Rd_); //desired end effector orientation
			Vector3t pd = V3DtoT<T>(pd_); //desired end effector position
			//position of the leg constraints in the local frame at the bottom
			Vector3t p1_init = V3DtoT<T>(p1_init_);
			Vector3t p2_init = V3DtoT<T>(p2_init_);
			Vector3t p3_init = V3DtoT<T>(p3_init_);
			Vector3t p4_init = V3DtoT<T>(p4_init_);
			Vector3t p5_init = V3DtoT<T>(p5_init_);
			Vector3t p6_init = V3DtoT<T>(p6_init_);
			Vector3t p7_init = V3DtoT<T>(p7_init_);
			Vector3t p8_init = V3DtoT<T>(p8_init_);
			Vector3t p9_init = V3DtoT<T>(p9_init_);
			Vector3t p10_init = V3DtoT<T>(p10_init_);
			Vector3t p11_init = V3DtoT<T>(p11_init_);
			Vector3t p12_init = V3DtoT<T>(p12_init_);
			//position of the leg constraints in the local frame at the top
			Vector3t p1_final = V3DtoT<T>(p1_final_);
			Vector3t p2_final = V3DtoT<T>(p2_final_);
			Vector3t p3_final = V3DtoT<T>(p3_final_);
			Vector3t p4_final = V3DtoT<T>(p4_final_);
			Vector3t p5_final = V3DtoT<T>(p5_final_);
			Vector3t p6_final = V3DtoT<T>(p6_final_);
			Vector3t p7_final = V3DtoT<T>(p7_final_);
			Vector3t p8_final = V3DtoT<T>(p8_final_);
			Vector3t p9_final = V3DtoT<T>(p9_final_);
			Vector3t p10_final = V3DtoT<T>(p10_final_);
			Vector3t p11_final = V3DtoT<T>(p11_final_);
			Vector3t p12_final = V3DtoT<T>(p12_final_);
//			Matrix3t R1_init_s = M3DtoT<T>(R1_init_s_);
//			Matrix3t R2_init_s = M3DtoT<T>(R2_init_s_);
//			Matrix3t R3_init_s = M3DtoT<T>(R3_init_s_);
//			Matrix3t R4_init_s = M3DtoT<T>(R4_init_s_);
//			Matrix3t R5_init_s = M3DtoT<T>(R5_init_s_);
//			Matrix3t R6_init_s = M3DtoT<T>(R6_init_s_);
//			Vector4t p1_init_s = V4DtoT<T>(p1_init_s_);
//			Vector4t p2_init_s = V4DtoT<T>(p2_init_s_);
//			Vector4t p3_init_s = V4DtoT<T>(p3_init_s_);
//			Vector4t p4_init_s = V4DtoT<T>(p4_init_s_);
//			Vector4t p5_init_s = V4DtoT<T>(p5_init_s_);
//			Vector4t p6_init_s = V4DtoT<T>(p6_init_s_);

#ifdef INCLUDE_WRIST
			//extract wrist angles
			T wrist_roll = x[GS - 13];
			T wrist_flex = x[GS - 14];
#endif
			//extract leg lengths from guess
			T L1 = x[GS - 12];
			T L2 = x[GS - 11];
			T L3 = x[GS - 10];
			T L4 = x[GS - 9];
			T L5 = x[GS - 8];
			T L6 = x[GS - 7];
			T L7 = x[GS - 6];
			T L8 = x[GS - 5];
			T L9 = x[GS - 4];
			T L10 = x[GS - 3];
			T L11 = x[GS - 2];
			T L12 = x[GS - 1];

			//extract values for top link
			T theta7 = x[41];
			T theta8 = x[47];
			T theta9 = x[53];
			T theta10 = x[59];
			T theta11 = x[65];
			T theta12 = x[71];
			Vector9t R7_init;
			Vector9t R8_init;
			Vector9t R9_init;
			Vector9t R10_init;
			Vector9t R11_init;
			Vector9t R12_init;
			//Matrix3t R7_init; R7_init << cos(theta7), -sin(theta7), 0, sin(theta7), cos(theta7), 0, 0, 0, 1; //this matrix is "reshaped" (converted into a vector column-by-column)
			R7_init << cos(theta7), sin(theta7), T(0.0), -sin(theta7), cos(theta7), T(0.0), T(0.0), T(0.0), T(1.0);
			R8_init << cos(theta8), sin(theta8), T(0.0), -sin(theta8), cos(theta8), T(0.0), T(0.0), T(0.0), T(1.0);
			R9_init << cos(theta9), sin(theta9), T(0.0), -sin(theta9), cos(theta9), T(0.0), T(0.0), T(0.0), T(1.0);
			R10_init << cos(theta10), sin(theta10), T(0.0), -sin(theta10), cos(theta10), T(0.0), T(0.0), T(0.0), T(1.0);
			R11_init << cos(theta11), sin(theta11), T(0.0), -sin(theta11), cos(theta11), T(0.0), T(0.0), T(0.0), T(1.0);
			R12_init << cos(theta12), sin(theta12), T(0.0), -sin(theta12), cos(theta12), T(0.0), T(0.0), T(0.0), T(1.0);

			Vector3t n7_init;
			Vector3t n8_init;
			Vector3t n9_init;
			Vector3t n10_init;
			Vector3t n11_init;
			Vector3t n12_init;
			Vector3t m7_init;
			Vector3t m8_init;
			Vector3t m9_init;
			Vector3t m10_init;
			Vector3t m11_init;
			Vector3t m12_init;
			n7_init << x[36], x[37], x[38];
			n8_init << x[42], x[43], x[44];
			n9_init << x[48], x[49], x[50];
			n10_init << x[54], x[55], x[56];
			n11_init << x[60], x[61], x[62];
			n12_init << x[66], x[67], x[68];
			m7_init << x[39], x[40], T(0.0);
			m8_init << x[45], x[46], T(0.0);
			m9_init << x[51], x[52], T(0.0);
			m10_init << x[57], x[58], T(0.0);
			m11_init << x[63], x[64], T(0.0);
			m12_init << x[69], x[70], T(0.0);

			Vector18t y7_init;
			Vector18t y8_init;
			Vector18t y9_init;
			Vector18t y10_init;
			Vector18t y11_init;
			Vector18t y12_init;
			y7_init << p7_init, R7_init, n7_init, m7_init;
			y8_init << p8_init, R8_init, n8_init, m8_init;
			y9_init << p9_init, R9_init, n9_init, m9_init;
			y10_init << p10_init, R10_init, n10_init, m10_init;
			y11_init << p11_init, R11_init, n11_init, m11_init;
			y12_init << p12_init, R12_init, n12_init, m12_init;

			//perform integration on top link
			cosserat_rod<T> cr7;
			cosserat_rod<T> cr8;
			cosserat_rod<T> cr9;
			cosserat_rod<T> cr10;
			cosserat_rod<T> cr11;
			cosserat_rod<T> cr12;
			cr7.set_init_state(y7_init);
			cr8.set_init_state(y8_init);
			cr9.set_init_state(y9_init);
			cr10.set_init_state(y10_init);
			cr11.set_init_state(y11_init);
			cr12.set_init_state(y12_init);
			cr7.save_positions = visualization_enabled;
			cr8.save_positions = visualization_enabled;
			cr9.save_positions = visualization_enabled;
			cr10.save_positions = visualization_enabled;
			cr11.save_positions = visualization_enabled;
			cr12.save_positions = visualization_enabled;
#ifdef USE_MULTITHREADING

			boost::thread t7(boost::bind(&cosserat_rod<T>::integrate, &cr7, T(0), L7, L7 / T(INTEGRATION_STEPS)));
			boost::thread t8(boost::bind(&cosserat_rod<T>::integrate, &cr8, T(0), L8, L8 / T(INTEGRATION_STEPS)));
			boost::thread t9(boost::bind(&cosserat_rod<T>::integrate, &cr9, T(0), L9, L9 / T(INTEGRATION_STEPS)));
			boost::thread t10(boost::bind(&cosserat_rod<T>::integrate, &cr10, T(0), L10, L10 / T(INTEGRATION_STEPS)));
			boost::thread t11(boost::bind(&cosserat_rod<T>::integrate, &cr11, T(0), L11, L11 / T(INTEGRATION_STEPS)));
			boost::thread t12(boost::bind(&cosserat_rod<T>::integrate, &cr12, T(0), L12, L12 / T(INTEGRATION_STEPS)));

			t7.join();
			t8.join();
			t9.join();
			t10.join();
			t11.join();
			t12.join();

			Vector18t y7 = cr7.result;
			Vector18t y8 = cr8.result;
			Vector18t y9 = cr9.result;
			Vector18t y10 = cr10.result;
			Vector18t y11 = cr11.result;
			Vector18t y12 = cr12.result;

#else
			cr7.integrate(T(0), L7, L7 / T(INTEGRATION_STEPS));
			cr8.integrate(T(0), L8, L8 / T(INTEGRATION_STEPS));
			cr9.integrate(T(0), L9, L9 / T(INTEGRATION_STEPS));
			cr10.integrate(T(0), L10, L10 / T(INTEGRATION_STEPS));
			cr11.integrate(T(0), L11, L11 / T(INTEGRATION_STEPS));
			cr12.integrate(T(0), L12, L12 / T(INTEGRATION_STEPS));
			Vector18t y7 = cr7.result;
			Vector18t y8 = cr8.result;
			Vector18t y9 = cr9.result;
			Vector18t y10 = cr10.result;
			Vector18t y11 = cr11.result;
			Vector18t y12 = cr12.result;
#endif
			if (visualization_enabled) {
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[6], cr7.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[7], cr8.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[8], cr9.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[9], cr10.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[10], cr11.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[11], cr12.positions, identity<T>());
			}

			//extract results from top link integration
			Vector3t p7_end;
			Vector3t p8_end;
			Vector3t p9_end;
			Vector3t p10_end;
			Vector3t p11_end;
			Vector3t p12_end;
			p7_end << y7[0], y7[1], y7[2];
			p8_end << y8[0], y8[1], y8[2];
			p9_end << y9[0], y9[1], y9[2];
			p10_end << y10[0], y10[1], y10[2];
			p11_end << y11[0], y11[1], y11[2];
			p12_end << y12[0], y12[1], y12[2];
			Matrix3t R7_end;
			Matrix3t R8_end;
			Matrix3t R9_end;
			Matrix3t R10_end;
			Matrix3t R11_end;
			Matrix3t R12_end;
			R7_end << y7[3], y7[6], y7[9], y7[4], y7[7], y7[10], y7[5], y7[8], y7[11];
			R8_end << y8[3], y8[6], y8[9], y8[4], y8[7], y8[10], y8[5], y8[8], y8[11];
			R9_end << y9[3], y9[6], y9[9], y9[4], y9[7], y9[10], y9[5], y9[8], y9[11];
			R10_end << y10[3], y10[6], y10[9], y10[4], y10[7], y10[10], y10[5], y10[8], y10[11];
			R11_end << y11[3], y11[6], y11[9], y11[4], y11[7], y11[10], y11[5], y11[8], y11[11];
			R12_end << y12[3], y12[6], y12[9], y12[4], y12[7], y12[10], y12[5], y12[8], y12[11];
//			Vector3t n7_end(y7[12], y7[13], y7[14]);
//			Vector3t n8_end(y8[12], y8[13], y8[14]);
//			Vector3t n9_end(y9[12], y9[13], y9[14]);
//			Vector3t n10_end(y10[12], y10[13], y10[14]);
//			Vector3t n11_end(y11[12], y11[13], y11[14]);
//			Vector3t n712_end(y12[12], y12[13], y12[14]);
//			Vector3t m7_end(y7[15], y7[16], y7[17]);
//			Vector3t m8_end(y8[15], y8[16], y8[17]);
//			Vector3t m9_end(y9[15], y9[16], y9[17]);
//			Vector3t m10_end(y10[15], y10[16], y10[17]);
//			Vector3t m11_end(y11[15], y11[16], y11[17]);
//			Vector3t m12_end(y12[15], y12[16], y12[17]);

//extract values for bottom link
			T theta1 = x[5];
			T theta2 = x[11];
			T theta3 = x[17];
			T theta4 = x[23];
			T theta5 = x[29];
			T theta6 = x[35];

			Matrix3t R1_init_m;
			Matrix3t R2_init_m;
			Matrix3t R3_init_m;
			Matrix3t R4_init_m;
			Matrix3t R5_init_m;
			Matrix3t R6_init_m;
			R1_init_m << cos(theta1), -sin(theta1), T(0.0), sin(theta1), cos(theta1), T(0.0), T(0.0), T(0.0), T(1.0);
			R2_init_m << cos(theta2), -sin(theta2), T(0.0), sin(theta2), cos(theta2), T(0.0), T(0.0), T(0.0), T(1.0);
			R3_init_m << cos(theta3), -sin(theta3), T(0.0), sin(theta3), cos(theta3), T(0.0), T(0.0), T(0.0), T(1.0);
			R4_init_m << cos(theta4), -sin(theta4), T(0.0), sin(theta4), cos(theta4), T(0.0), T(0.0), T(0.0), T(1.0);
			R5_init_m << cos(theta5), -sin(theta5), T(0.0), sin(theta5), cos(theta5), T(0.0), T(0.0), T(0.0), T(1.0);
			R6_init_m << cos(theta6), -sin(theta6), T(0.0), sin(theta6), cos(theta6), T(0.0), T(0.0), T(0.0), T(1.0);
			R1_init_m = R1_init_m * R7_end;
			R2_init_m = R2_init_m * R7_end;
			R3_init_m = R3_init_m * R7_end;
			R4_init_m = R4_init_m * R7_end;
			R5_init_m = R5_init_m * R7_end;
			R6_init_m = R6_init_m * R7_end;

			//StoreRinit<T>()(this, R1_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R1_init_s, R1_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R2_init_s, R2_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R3_init_s, R3_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R4_init_s, R4_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R5_init_s, R5_init_m, identity<T>());
			StoreM3d<T>()(const_cast<IKFunctor*>(this)->R6_init_s, R6_init_m, identity<T>());

			Vector9t R1_init;
			Vector9t R2_init;
			Vector9t R3_init;
			Vector9t R4_init;
			Vector9t R5_init;
			Vector9t R6_init;
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
			Vector3t n1_init;
			Vector3t n2_init;
			Vector3t n3_init;
			Vector3t n4_init;
			Vector3t n5_init;
			Vector3t n6_init;
			Vector3t m1_init;
			Vector3t m2_init;
			Vector3t m3_init;
			Vector3t m4_init;
			Vector3t m5_init;
			Vector3t m6_init;
			n1_init << x[0], x[1], x[2];
			n2_init << x[6], x[7], x[8];
			n3_init << x[12], x[13], x[14];
			n4_init << x[18], x[19], x[20];
			n5_init << x[24], x[25], x[26];
			n6_init << x[30], x[31], x[32];
			m1_init << x[3], x[4], T(0);
			m2_init << x[9], x[10], T(0);
			m3_init << x[15], x[16], T(0);
			m4_init << x[21], x[22], T(0);
			m5_init << x[27], x[28], T(0);
			m6_init << x[33], x[34], T(0);

			//centroid of all the bottom segment leg ends
			Vector3t p_cb = (p7_end + p8_end + p9_end + p10_end + p11_end + p12_end) / T(6.0);

			//transformation from base to mid_plate
			Matrix4t T_mid;
			T_mid.block(0, 0, 3, 3) = R7_end;
			T_mid.block(0, 3, 3, 1) = p_cb;
			RowVector4t zzzo;
			zzzo << T(0), T(0), T(0), T(1);
			T_mid.block(3, 0, 1, 4) = zzzo;
			Vector4t p1_init_2;
			Vector4t p2_init_2;
			Vector4t p3_init_2;
			Vector4t p4_init_2;
			Vector4t p5_init_2;
			Vector4t p6_init_2;
			p1_init_2 << p1_init(0), p1_init(1), p1_init(2), T(1);
			p2_init_2 << p2_init(0), p2_init(1), p2_init(2), T(1);
			p3_init_2 << p3_init(0), p3_init(1), p3_init(2), T(1);
			p4_init_2 << p4_init(0), p4_init(1), p4_init(2), T(1);
			p5_init_2 << p5_init(0), p5_init(1), p5_init(2), T(1);
			p6_init_2 << p6_init(0), p6_init(1), p6_init(2), T(1);
			p1_init_2 = T_mid * p1_init_2;
			p2_init_2 = T_mid * p2_init_2;
			p3_init_2 = T_mid * p3_init_2;
			p4_init_2 = T_mid * p4_init_2;
			p5_init_2 = T_mid * p5_init_2;
			p6_init_2 = T_mid * p6_init_2;
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p1_init_s, p1_init_2, identity<T>());
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p2_init_s, p2_init_2, identity<T>());
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p3_init_s, p3_init_2, identity<T>());
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p4_init_s, p4_init_2, identity<T>());
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p5_init_s, p5_init_2, identity<T>());
			StoreV4d<T>()(const_cast<IKFunctor*>(this)->p6_init_s, p6_init_2, identity<T>());
			Vector18t y1_init;
			Vector18t y2_init;
			Vector18t y3_init;
			Vector18t y4_init;
			Vector18t y5_init;
			Vector18t y6_init;
			y1_init << p1_init_2.head(3), R1_init, n1_init, m1_init;
			y2_init << p2_init_2.head(3), R2_init, n2_init, m2_init;
			y3_init << p3_init_2.head(3), R3_init, n3_init, m3_init;
			y4_init << p4_init_2.head(3), R4_init, n4_init, m4_init;
			y5_init << p5_init_2.head(3), R5_init, n5_init, m5_init;
			y6_init << p6_init_2.head(3), R6_init, n6_init, m6_init;

			//perform integration on bottom link
			cosserat_rod<T> cr1;
			cosserat_rod<T> cr2;
			cosserat_rod<T> cr3;
			cosserat_rod<T> cr4;
			cosserat_rod<T> cr5;
			cosserat_rod<T> cr6;
			cr1.set_init_state(y1_init);
			cr2.set_init_state(y2_init);
			cr3.set_init_state(y3_init);
			cr4.set_init_state(y4_init);
			cr5.set_init_state(y5_init);
			cr6.set_init_state(y6_init);
			cr1.save_positions = visualization_enabled;
			cr2.save_positions = visualization_enabled;
			cr3.save_positions = visualization_enabled;
			cr4.save_positions = visualization_enabled;
			cr5.save_positions = visualization_enabled;
			cr6.save_positions = visualization_enabled;
#ifdef USE_MULTITHREADING
			boost::thread t1(boost::bind(&cosserat_rod<T>::integrate, &cr1, T(0), L1, L1 / T(INTEGRATION_STEPS)));
			boost::thread t2(boost::bind(&cosserat_rod<T>::integrate, &cr2, T(0), L2, L2 / T(INTEGRATION_STEPS)));
			boost::thread t3(boost::bind(&cosserat_rod<T>::integrate, &cr3, T(0), L3, L3 / T(INTEGRATION_STEPS)));
			boost::thread t4(boost::bind(&cosserat_rod<T>::integrate, &cr4, T(0), L4, L4 / T(INTEGRATION_STEPS)));
			boost::thread t5(boost::bind(&cosserat_rod<T>::integrate, &cr5, T(0), L5, L5 / T(INTEGRATION_STEPS)));
			boost::thread t6(boost::bind(&cosserat_rod<T>::integrate, &cr6, T(0), L6, L6 / T(INTEGRATION_STEPS)));
			t1.join();
			t2.join();
			t3.join();
			t4.join();
			t5.join();
			t6.join();
			Vector18t y1 = cr1.result;
			Vector18t y2 = cr2.result;
			Vector18t y3 = cr3.result;
			Vector18t y4 = cr4.result;
			Vector18t y5 = cr5.result;
			Vector18t y6 = cr6.result;
#else
			cr1.integrate(T(0), L1, L1 / T(INTEGRATION_STEPS));
			cr2.integrate(T(0), L2, L2 / T(INTEGRATION_STEPS));
			cr3.integrate(T(0), L3, L3 / T(INTEGRATION_STEPS));
			cr4.integrate(T(0), L4, L4 / T(INTEGRATION_STEPS));
			cr5.integrate(T(0), L5, L5 / T(INTEGRATION_STEPS));
			cr6.integrate(T(0), L6, L6 / T(INTEGRATION_STEPS));
			Vector18t y1 = cr1.result;
			Vector18t y2 = cr2.result;
			Vector18t y3 = cr3.result;
			Vector18t y4 = cr4.result;
			Vector18t y5 = cr5.result;
			Vector18t y6 = cr6.result;
#endif
			if (visualization_enabled) {
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[0], cr1.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[1], cr2.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[2], cr3.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[3], cr4.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[4], cr5.positions, identity<T>());
				StoreM21_3d<T>()(const_cast<IKFunctor*>(this)->rodpositions[5], cr6.positions, identity<T>());
			}

			//extract results from bottom link integration
			Vector3t p1_end;
			Vector3t p2_end;
			Vector3t p3_end;
			Vector3t p4_end;
			Vector3t p5_end;
			Vector3t p6_end;
			p1_end << y1[0], y1[1], y1[2];
			p2_end << y2[0], y2[1], y2[2];
			p3_end << y3[0], y3[1], y3[2];
			p4_end << y4[0], y4[1], y4[2];
			p5_end << y5[0], y5[1], y5[2];
			p6_end << y6[0], y6[1], y6[2];
			Matrix3t R1_end;
			Matrix3t R2_end;
			Matrix3t R3_end;
			Matrix3t R4_end;
			Matrix3t R5_end;
			Matrix3t R6_end;
			R1_end << y1[3], y1[6], y1[9], y1[4], y1[7], y1[10], y1[5], y1[8], y1[11];
			R2_end << y2[3], y2[6], y2[9], y2[4], y2[7], y2[10], y2[5], y2[8], y2[11];
			R3_end << y3[3], y3[6], y3[9], y3[4], y3[7], y3[10], y3[5], y3[8], y3[11];
			R4_end << y4[3], y4[6], y4[9], y4[4], y4[7], y4[10], y4[5], y4[8], y4[11];
			R5_end << y5[3], y5[6], y5[9], y5[4], y5[7], y5[10], y5[5], y5[8], y5[11];
			R6_end << y6[3], y6[6], y6[9], y6[4], y6[7], y6[10], y6[5], y6[8], y6[11];
			Vector3t n1_end;
			Vector3t n2_end;
			Vector3t n3_end;
			Vector3t n4_end;
			Vector3t n5_end;
			Vector3t n6_end;
			Vector3t m1_end;
			Vector3t m2_end;
			Vector3t m3_end;
			Vector3t m4_end;
			Vector3t m5_end;
			Vector3t m6_end;
			n1_end << y1[12], y1[13], y1[14];
			n2_end << y2[12], y2[13], y2[14];
			n3_end << y3[12], y3[13], y3[14];
			n4_end << y4[12], y4[13], y4[14];
			n5_end << y5[12], y5[13], y5[14];
			n6_end << y6[12], y6[13], y6[14];
			m1_end << y1[15], y1[16], y1[17];
			m2_end << y2[15], y2[16], y2[17];
			m3_end << y3[15], y3[16], y3[17];
			m4_end << y4[15], y4[16], y4[17];
			m5_end << y5[15], y5[16], y5[17];
			m6_end << y6[15], y6[16], y6[17];

			//centroid of all top segment leg ends
			Vector3t p_ct = (p1_end + p2_end + p3_end + p4_end + p5_end + p6_end) / T(6.0);

			//residual of equilibrium conditions
			Vector3t res_eq_F_top = (n1_end + n2_end + n3_end + n4_end + n5_end + n6_end) - F;
			Vector3t res_eq_L_top = (cosserat_rod<T>::hat(p1_end - p_ct) * n1_end + cosserat_rod<T>::hat(p2_end - p_ct) * n2_end
					+ cosserat_rod<T>::hat(p3_end - p_ct) * n3_end + cosserat_rod<T>::hat(p4_end - p_ct) * n4_end + cosserat_rod<T>::hat(p5_end - p_ct) * n5_end
					+ cosserat_rod<T>::hat(p6_end - p_ct) * n6_end + m1_end + m2_end + m3_end + m4_end + m5_end + m6_end) - L;

#ifdef INCLUDE_WRIST
			Matrix4t T_rotation_flex; //Transformation from the rotation joint to the flex joint
			Matrix4t T_flex_gripper;//Transformation from the flex joint to the gripper
			T_rotation_flex << cos(wrist_roll), -sin(wrist_roll), T(0), T(0), sin(wrist_roll), cos(wrist_roll), T(0), T(0), T(0), T(0), T(1), T(
					DIST_TO_FLEX_JOINT), T(0), T(0), T(0), T(1);
			T_flex_gripper << T(1), T(0), T(0), T(0), T(0), cos(wrist_flex), -sin(wrist_flex), T(0), T(0), sin(wrist_flex), cos(wrist_flex), T(DIST_TO_GRIPPER), T(
					0), T(0), T(0), T(1);

			Matrix4t T_rotation_gripper = T_flex_gripper * T_rotation_flex;//Transformation from the rotation joint to the gripper

			//invert the T_rotation_gripper matrix;
			Matrix4t T_gripper_rotation = T_rotation_gripper;
			T_gripper_rotation.block(0, 0, 3, 3) = T_rotation_gripper.block(0, 0, 3, 3).transpose();
			T_gripper_rotation.block(0, 3, 3, 1) = -T_gripper_rotation.block(0, 0, 3, 3) * T_rotation_gripper.block(0, 3, 3, 1);

			Vector4t pdEEt;
			pdEEt << pd, T(1);
			Vector3t pdEE = (T_gripper_rotation * pdEEt).head(3);

			Matrix4t RdEEt = Matrix<T, 4, 4>::Zero();
			RdEEt.block(0, 0, 3, 3) = Rd;
			RdEEt(3, 3) = T(1);
			Matrix3t RdEE = (T_gripper_rotation * RdEEt).block(0, 0, 3, 3);

#else
			Vector3t pdEE = pd;
			Matrix3t RdEE = Rd;
#endif

			//These are analogous to loop closure equations because they are only satisfied when the positions of the rod ends have the same relative positions as the connection pattern in the top plate
//			Vector3t res_p1 = pdEE + RdEE * p1_final - p1_end;
//			Vector3t res_p2 = pdEE + RdEE * p2_final - p2_end;
//			Vector3t res_p3 = pdEE + RdEE * p3_final - p3_end;
//			Vector3t res_p4 = pdEE + RdEE * p4_final - p4_end;
//			Vector3t res_p5 = pdEE + RdEE * p5_final - p5_end;
//			Vector3t res_p6 = pdEE + RdEE * p6_final - p6_end;
//			Vector3t res_p7 = p7_end - R7_end * (p7_final - p7_final) - p7_end;
//			Vector3t res_p8 = p7_end - R7_end * (p7_final - p8_final) - p8_end;
//			Vector3t res_p9 = p7_end - R7_end * (p7_final - p9_final) - p9_end;
//			Vector3t res_p10 = p7_end - R7_end * (p7_final - p10_final) - p10_end;
//			Vector3t res_p11 = p7_end - R7_end * (p7_final - p11_final) - p11_end;
//			Vector3t res_p12 = p7_end - R7_end * (p7_final - p12_final) - p12_end;
			Vector3t min_length_res;
			min_length_res << T(1e99), T(1e99), T(1e99);
			Vector3t res_p1 = ((L1 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p1_final - p1_end : min_length_res);
			Vector3t res_p2 = ((L2 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p2_final - p2_end : min_length_res);
			Vector3t res_p3 = ((L3 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p3_final - p3_end : min_length_res);
			Vector3t res_p4 = ((L4 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p4_final - p4_end : min_length_res);
			Vector3t res_p5 = ((L5 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p5_final - p5_end : min_length_res);
			Vector3t res_p6 = ((L6 > T(MIN_LEG_LENGTH_TOP)) ? pdEE + RdEE * p6_final - p6_end : min_length_res);
			Vector3t res_p7 = ((L7 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p7_final) - p7_end : min_length_res);
			Vector3t res_p8 = ((L8 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p8_final) - p8_end : min_length_res);
			Vector3t res_p9 = ((L9 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p9_final) - p9_end : min_length_res);
			Vector3t res_p10 = ((L10 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p10_final) - p10_end : min_length_res);
			Vector3t res_p11 = ((L11 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p11_final) - p11_end : min_length_res);
			Vector3t res_p12 = ((L12 > T(MIN_LEG_LENGTH_BOTTOM)) ? p7_end - R7_end * (p7_final - p12_final) - p12_end : min_length_res);

			//force a common material orientation for all the distal rod ends
#ifdef USE_MATRIX_LOG
			Vector3t res_R1;
			Vector3t res_R2;
			Vector3t res_R3;
			Vector3t res_R4;
			Vector3t res_R5;
			Vector3t res_R6;
			Vector3t res_R7;
			Vector3t res_R8;
			Vector3t res_R9;
			Vector3t res_R10;
			Vector3t res_R11;
			Vector3t res_R12;
			res_R1 << T(0.0), T(0.0), T(0.0);
			res_R2 << T(0.0), T(0.0), T(0.0);
			res_R3 << T(0.0), T(0.0), T(0.0);
			res_R4 << T(0.0), T(0.0), T(0.0);
			res_R5 << T(0.0), T(0.0), T(0.0);
			res_R6 << T(0.0), T(0.0), T(0.0);
			res_R7 << T(0.0), T(0.0), T(0.0);
			res_R8 << T(0.0), T(0.0), T(0.0);
			res_R9 << T(0.0), T(0.0), T(0.0);
			res_R10 << T(0.0), T(0.0), T(0.0);
			res_R11 << T(0.0), T(0.0), T(0.0);
			res_R12 << T(0.0), T(0.0), T(0.0);
			Matrix3t Rdt = RdEE.transpose();
			//Eigen::MatrixLogarithmAtomic < Matrix3t > Rdtm;
			//Rdtm.compute(Rdt);
			if (!(Rdt * R1_end).isZero()) res_R1 = cosserat_rod<T>::vee((Rdt * R1_end).log());
			if (!(Rdt * R2_end).isZero()) res_R2 = cosserat_rod<T>::vee((Rdt * R2_end).log());
			if (!(Rdt * R3_end).isZero()) res_R3 = cosserat_rod<T>::vee((Rdt * R3_end).log());
			if (!(Rdt * R4_end).isZero()) res_R4 = cosserat_rod<T>::vee((Rdt * R4_end).log());
			if (!(Rdt * R5_end).isZero()) res_R5 = cosserat_rod<T>::vee((Rdt * R5_end).log());
			if (!(Rdt * R6_end).isZero()) res_R6 = cosserat_rod<T>::vee((Rdt * R6_end).log());
			if (!(R7_end.transpose() * R7_end).isZero()) res_R7 = cosserat_rod<T>::vee((R7_end.transpose() * R7_end).log());
			if (!(R8_end.transpose() * R7_end).isZero()) res_R8 = cosserat_rod<T>::vee((R8_end.transpose() * R7_end).log());
			if (!(R9_end.transpose() * R7_end).isZero()) res_R9 = cosserat_rod<T>::vee((R9_end.transpose() * R7_end).log());
			if (!(R10_end.transpose() * R7_end).isZero()) res_R10 = cosserat_rod<T>::vee((R10_end.transpose() * R7_end).log());
			if (!(R11_end.transpose() * R7_end).isZero()) res_R11 = cosserat_rod<T>::vee((R11_end.transpose() * R7_end).log());
			if (!(R12_end.transpose() * R7_end).isZero()) res_R12 = cosserat_rod<T>::vee((R12_end.transpose() * R7_end).log());
#else
			Matrix<T, 2, 3> rodrigues;
			rodrigues << T(1), T(0), T(0), T(0), T(1), T(0);
			Matrix3t Rdt = RdEE.transpose();
			Vector3t res_R1;
			Vector3t res_R2;
			Vector3t res_R3;
			Vector3t res_R4;
			Vector3t res_R5;
			Vector3t res_R6;
			Vector3t res_R7;
			Vector3t res_R8;
			Vector3t res_R9;
			Vector3t res_R10;
			Vector3t res_R11;
			Vector3t res_R12;
			res_R1 << rodrigues * cosserat_rod<T>::vee((R1_end.transpose() * RdEE - R1_end * Rdt)), T(0);
			res_R2 << rodrigues * cosserat_rod<T>::vee((R2_end.transpose() * RdEE - R2_end * Rdt)), T(0);
			res_R3 << rodrigues * cosserat_rod<T>::vee((R3_end.transpose() * RdEE - R3_end * Rdt)), T(0);
			res_R4 << rodrigues * cosserat_rod<T>::vee((R4_end.transpose() * RdEE - R4_end * Rdt)), T(0);
			res_R5 << rodrigues * cosserat_rod<T>::vee((R5_end.transpose() * RdEE - R5_end * Rdt)), T(0);
			res_R6 << rodrigues * cosserat_rod<T>::vee((R6_end.transpose() * RdEE - R6_end * Rdt)), T(0);
			res_R7 << rodrigues * cosserat_rod<T>::vee((R7_end.transpose() * R7_end - R7_end * R7_end.transpose())), T(0);
			res_R8 << rodrigues * cosserat_rod<T>::vee((R8_end.transpose() * R7_end - R8_end * R7_end.transpose())), T(0);
			res_R9 << rodrigues * cosserat_rod<T>::vee((R9_end.transpose() * R7_end - R9_end * R7_end.transpose())), T(0);
			res_R10 << rodrigues * cosserat_rod<T>::vee((R10_end.transpose() * R7_end - R10_end * R7_end.transpose())), T(0);
			res_R11 << rodrigues * cosserat_rod<T>::vee((R11_end.transpose() * R7_end - R11_end * R7_end.transpose())), T(0);
			res_R12 << rodrigues * cosserat_rod<T>::vee((R12_end.transpose() * R7_end - R12_end * R7_end.transpose())), T(0);
#endif

			T l_c = T(0.01); //characteristic length converts rotation error to meters
			Matrix<T, GS - 6, 1> res;
			res << res_eq_F_top, res_eq_L_top, res_p1, res_p2, res_p3, res_p4, res_p5, res_p6, res_p7, res_p8, res_p9, res_p10, res_p11, res_p12, res_R1 * l_c, res_R2
					* l_c, res_R3 * l_c, res_R4 * l_c, res_R5 * l_c, res_R6 * l_c, res_R7 * l_c, res_R8 * l_c, res_R9 * l_c, res_R10 * l_c, res_R11 * l_c, res_R12
					* l_c;

			for (unsigned int i = 0; i < GS - 6; i++) {
				residual[i] = res[i];
			}

			return true;
		}

		bool visualization_enabled;

		//desired positions and forces
		Eigen::Vector3d F_;  //applied force at end effector
		Eigen::Vector3d L_;  //applied moment at end effector
		Eigen::Matrix3d Rd_; //desired end effector orientation
		Eigen::Vector3d pd_; //desired end effector position

		//position of the leg constraints in the local frame at the bottom
		Eigen::Vector3d p1_init_;
		Eigen::Vector3d p2_init_;
		Eigen::Vector3d p3_init_;
		Eigen::Vector3d p4_init_;
		Eigen::Vector3d p5_init_;
		Eigen::Vector3d p6_init_;
		Eigen::Vector3d p7_init_;
		Eigen::Vector3d p8_init_;
		Eigen::Vector3d p9_init_;
		Eigen::Vector3d p10_init_;
		Eigen::Vector3d p11_init_;
		Eigen::Vector3d p12_init_;

		//position of the leg constraints in the local frame at the top
		Eigen::Vector3d p6_final_;
		Eigen::Vector3d p1_final_;
		Eigen::Vector3d p2_final_;
		Eigen::Vector3d p3_final_;
		Eigen::Vector3d p4_final_;
		Eigen::Vector3d p5_final_;
		Eigen::Vector3d p12_final_;
		Eigen::Vector3d p7_final_;
		Eigen::Vector3d p8_final_;
		Eigen::Vector3d p9_final_;
		Eigen::Vector3d p10_final_;
		Eigen::Vector3d p11_final_;

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

		//position integrations for each rod, saved for visualization
		Eigen::Matrix<double, INTEGRATION_STEPS + 1, 3> rodpositions[12];

	private:
		template<typename T> struct StoreM3d {
			template<typename TP> void operator()(Eigen::Matrix<double, 3, 3> &to, Eigen::Matrix<T, 3, 3> from, identity<TP>) const {
			}

			void operator()(Eigen::Matrix<double, 3, 3> &to, Eigen::Matrix<T, 3, 3> from, identity<double>) const {
				to = from;
			}
		};

		template<typename T> struct StoreV4d {
			template<typename TP> void operator()(Eigen::Matrix<double, 4, 1> &to, Eigen::Matrix<T, 4, 1> from, identity<TP>) const {
			}

			void operator()(Eigen::Matrix<double, 4, 1> &to, Eigen::Matrix<T, 4, 1> from, identity<double>) const {
				to = from;
			}
		};

		template<typename T> struct StoreM21_3d {
			template<typename TP> void operator()(Eigen::Matrix<double, INTEGRATION_STEPS + 1, 3> &to, Eigen::Matrix<T, INTEGRATION_STEPS + 1, 3> from,
					identity<TP>) const {
			}

			void operator()(Eigen::Matrix<double, INTEGRATION_STEPS + 1, 3> &to, Eigen::Matrix<T, INTEGRATION_STEPS + 1, 3> from, identity<double>) const {
				to = from;
			}
		};

	};

	struct SingleIKFunctor {
		template<typename T>
		bool operator()(const T* const x, T* residual) const {

			using Eigen::Matrix;
			typedef Matrix<T, 3, 3> Matrix3t;
			typedef Matrix<T, 4, 4> Matrix4t;
			typedef Matrix<T, 3, 1> Vector3t;
			typedef Matrix<T, 4, 1> Vector4t;
			typedef Matrix<T, 1, 4> RowVector4t;
			typedef Matrix<T, 9, 1> Vector9t;
			typedef Matrix<T, 18, 1> Vector18t;

			using std::cout;
			using std::endl;

			Vector3t p_init = V3DtoT<T>(p_init_);
			Vector3t p_final = V3DtoT<T>(p_final_);
			Matrix3t R_final = M3DtoT<T>(R_final_);

			T L = x[6];
			T theta = x[5];
			Vector9t R_init;
			R_init << cos(theta), sin(theta), T(0.0), -sin(theta), cos(theta), T(0.0), T(0.0), T(0.0), T(1.0);
			Vector3t n_init;
			n_init << x[0], x[1], x[2];
			Vector3t m_init;
			m_init << x[3], x[4], T(0.0);

			Vector18t y_init;
			y_init << p_init, R_init, n_init, m_init;

			cosserat_rod<T> cr;
			cr.set_init_state(y_init);
			Vector18t y = cr.integrate(T(0), L, L / T(INTEGRATION_STEPS));

			Vector3t p_end;
			p_end << y[0], y[1], y[2];
			Matrix3t R_end;
			R_end << y[3], y[6], y[9], y[4], y[7], y[10], y[5], y[8], y[11];
			//Vector3t n_end(y[12], y[13], y[14]);
			//Vector3t m_end(y[15], y[16], y[17]);
			Vector3t res_p = p_final - p_end;
			// Matrix3d R_final3x3;
			// R_final3x3 << R_Final[0], R_Final[3], R_Final[6], R_Final[1], R_Final[4], R_Final[7], R_Final[2], R_Final[5], R_Final[8];
#ifdef USE_MATRIX_LOG
			Vector3t res_R;
			res_R << T(0.0), T(0.0), T(0.0);
			if (!(R_final.transpose() * R_end).isZero()) res_R = cosserat_rod<T>::vee((R_final.transpose() * R_end).log());
#else
			Matrix<T, 2, 3> rodrigues;
			rodrigues << T(1), T(0), T(0), T(0), T(1), T(0);
			Vector3t res_R;
			res_R << rodrigues * cosserat_rod<T>::vee((R_end.transpose() * R_final - R_end * R_final.transpose())), T(0);
#endif
			T l_c = T(0.01); //characteristic length converts rotation error to meters
			Matrix<T, 6, 1> res;
			res << res_p, res_R * l_c;
			for (unsigned int i = 0; i < 6; i++) {
				residual[i] = res[i];
			}

			return true;
		}
		Eigen::Vector3d p_init_;
		Eigen::Vector3d p_final_;
		Eigen::Matrix3d R_final_;
	};

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

	IKFunctor* ikfunctor;
	SingleIKFunctor* singleikfunctor;
	double guess_init[GS];
	double single_guess_init[6][7];
	ceres::Problem problem;
	ceres::Problem problem_single[6];
	ceres::CostFunction* cost_function;
	ceres::CostFunction* cost_function_single[6];
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	ros::ServiceServer solverService;
	ros::Publisher vis_pub;

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
