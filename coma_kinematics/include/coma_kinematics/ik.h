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

#define GS 7*12 //define the guess size

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

	struct IKFunctor {
		bool operator()(const double* const x, double* residual) const {
			using Eigen::Matrix;
			using Eigen::Vector3d;
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
			R10_init << cos(theta10), sin(theta10), 0.0, -sin(theta10), cos(
					theta10), 0.0, 0.0, 0.0, 1.0;
			R11_init << cos(theta11), sin(theta11), 0.0, -sin(theta11), cos(
					theta11), 0.0, 0.0, 0.0, 1.0;
			R12_init << cos(theta12), sin(theta12), 0.0, -sin(theta12), cos(
					theta12), 0.0, 0.0, 0.0, 1.0;

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

			Vector18d y7_init; y7_init << p7_init, R7_init, n7_init, m7_init;
			Vector18d y8_init; y8_init << p8_init, R8_init, n8_init, m8_init;
			Vector18d y9_init; y9_init << p9_init, R9_init, n9_init, m9_init;
			Vector18d y10_init; y10_init << p10_init, R10_init, n10_init, m10_init;
			Vector18d y11_init; y11_init << p11_init, R11_init, n11_init, m11_init;
			Vector18d y12_init; y12_init << p12_init, R12_init, n12_init, m12_init;


			cosserat_rod cr7(y7_init);
			cr7.integrate(0,L7,L7/20);


			/*




			 f = @(x, y) deriv(x, y, K_se_inv, K_bt_inv);

			 [~,y7]=ode45(f,linspace(0,L7,20),y7_init);
			 [~,y8]=ode45(f,linspace(0,L8,20),y8_init);
			 [~,y9]=ode45(f,linspace(0,L9,20),y9_init);
			 [~,y10]=ode45(f,linspace(0,L10,20),y10_init);
			 [~,y11]=ode45(f,linspace(0,L11,20),y11_init);
			 [~,y12]=ode45(f,linspace(0,L12,20),y12_init);

			 p7_end=y7(end,1:3)';
			 R7_end=reshape(y7(end,4:12),3,3);
			 n7_end=y7(end,13:15)';
			 m7_end=y7(end,16:18)';

			 p8_end=y8(end,1:3)';
			 R8_end=reshape(y8(end,4:12),3,3);
			 n8_end=y8(end,13:15)';
			 m8_end=y8(end,16:18)';

			 p9_end=y9(end,1:3)';
			 R9_end=reshape(y9(end,4:12),3,3);
			 n9_end=y9(end,13:15)';
			 m9_end=y9(end,16:18)';

			 p10_end=y10(end,1:3)';
			 R10_end=reshape(y10(end,4:12),3,3);
			 n10_end=y10(end,13:15)';
			 m10_end=y10(end,16:18)';

			 p11_end=y11(end,1:3)';
			 R11_end=reshape(y11(end,4:12),3,3);
			 n11_end=y11(end,13:15)';
			 m11_end=y11(end,16:18)';

			 p12_end=y12(end,1:3)';
			 R12_end=reshape(y12(end,4:12),3,3);
			 n12_end=y12(end,13:15)';
			 m12_end=y12(end,16:18)';



			 %todo, use init forces from bottom link for top link?
			 %note: R7_end is constrained to be perpendicular to R8..12, it
			 %therefore equals the rotation matrix of the mid-plate
			 theta1=x(6);
			 R1_init=[cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0;0 0 1]*R7_end;
			 R1_init=reshape(R1_init,9,1);
			 n1_init=x(1:3);
			 m1_init(1:2,1)=x(4:5);
			 m1_init(3)=0;

			 theta2=x(12);
			 R2_init=[cos(theta2) -sin(theta2) 0; sin(theta2) cos(theta2) 0;0 0 1]*R7_end;
			 R2_init=reshape(R2_init,9,1);
			 n2_init=x(7:9);
			 m2_init(1:2,1)=x(10:11)';
			 m2_init(3)=0;

			 theta3=x(18);
			 R3_init=[cos(theta3) -sin(theta3) 0; sin(theta3) cos(theta3) 0;0 0 1]*R7_end;
			 R3_init=reshape(R3_init,9,1);
			 n3_init=x(13:15);
			 m3_init(1:2,1)=x(16:17)';
			 m3_init(3)=0;

			 theta4=x(24);
			 R4_init=[cos(theta4) -sin(theta4) 0; sin(theta4) cos(theta4) 0;0 0 1]*R7_end;
			 R4_init=reshape(R4_init,9,1);
			 n4_init=x(19:21);
			 m4_init(1:2,1)=x(22:23)';
			 m4_init(3)=0;

			 theta5=x(30);
			 R5_init=[cos(theta5) -sin(theta5) 0; sin(theta5) cos(theta5) 0;0 0 1]*R7_end;
			 R5_init=reshape(R5_init,9,1);
			 n5_init=x(25:27);
			 m5_init(1:2,1)=x(28:29)';
			 m5_init(3)=0;

			 theta6=x(36);
			 R6_init=[cos(theta6) -sin(theta6) 0; sin(theta6) cos(theta6) 0;0 0 1]*R7_end;
			 R6_init=reshape(R6_init,9,1);
			 n6_init=x(31:33);
			 m6_init(1:2,1)=x(34:35)';
			 m6_init(3)=0;
			 %done extracting everything from the u matrix


			 %centroid of all bottom segment leg ends
			 p_cb=(p7_end+p8_end+p9_end+p10_end+p11_end+p12_end)/6;

			 %transformation matrix from base to mid_plate
			 T_mid = [R7_end p_cb; 0 0 0 1];

			 %position of top plate legs at mid plate in base reference frame
			 p1_init_2 = T_mid*[p1_init; 1];
			 p2_init_2 = T_mid*[p2_init; 1];
			 p3_init_2 = T_mid*[p3_init; 1];
			 p4_init_2 = T_mid*[p4_init; 1];
			 p5_init_2 = T_mid*[p5_init; 1];
			 p6_init_2 = T_mid*[p6_init; 1];

			 y1_init=[p1_init_2(1:3);R1_init;n1_init;m1_init];
			 y2_init=[p2_init_2(1:3);R2_init;n2_init;m2_init];
			 y3_init=[p3_init_2(1:3);R3_init;n3_init;m3_init];
			 y4_init=[p4_init_2(1:3);R4_init;n4_init;m4_init];
			 y5_init=[p5_init_2(1:3);R5_init;n5_init;m5_init];
			 y6_init=[p6_init_2(1:3);R6_init;n6_init;m6_init];

			 %Integrate all rods from base to tip for the current x
			 %ode45(odefun, tspan (20 increments of 0 to rod length), inititial
			 %conditions)
			 [~,y1]=ode45(f,linspace(0,L1,20),y1_init);
			 [~,y2]=ode45(f,linspace(0,L2,20),y2_init);
			 [~,y3]=ode45(f,linspace(0,L3,20),y3_init);
			 [~,y4]=ode45(f,linspace(0,L4,20),y4_init);
			 [~,y5]=ode45(f,linspace(0,L5,20),y5_init);
			 [~,y6]=ode45(f,linspace(0,L6,20),y6_init);

			 %Get the state of each rod at the tip
			 p1_end=y1(end,1:3)';
			 R1_end=reshape(y1(end,4:12),3,3);
			 n1_end=y1(end,13:15)';
			 m1_end=y1(end,16:18)';

			 p2_end=y2(end,1:3)';
			 R2_end=reshape(y2(end,4:12),3,3);
			 n2_end=y2(end,13:15)';
			 m2_end=y2(end,16:18)';

			 p3_end=y3(end,1:3)';
			 R3_end=reshape(y3(end,4:12),3,3);
			 n3_end=y3(end,13:15)';
			 m3_end=y3(end,16:18)';

			 p4_end=y4(end,1:3)';
			 R4_end=reshape(y4(end,4:12),3,3);
			 n4_end=y4(end,13:15)';
			 m4_end=y4(end,16:18)';

			 p5_end=y5(end,1:3)';
			 R5_end=reshape(y5(end,4:12),3,3);
			 n5_end=y5(end,13:15)';
			 m5_end=y5(end,16:18)';

			 p6_end=y6(end,1:3)';
			 R6_end=reshape(y6(end,4:12),3,3);
			 n6_end=y6(end,13:15)';
			 m6_end=y6(end,16:18)';



			 %centroid of all top segment leg ends
			 p_ct=(p1_end+p2_end+p3_end+p4_end+p5_end+p6_end)/6;
			 %Residual of equilibrium conditions
			 res_eq_top=[(n1_end+n2_end+n3_end+n4_end+n5_end+n6_end)-F;
			 (hat(p1_end-p_ct)*n1_end+hat(p2_end-p_ct)*n2_end+...
			 +hat(p3_end-p_ct)*n3_end+hat(p4_end-p_ct)*n4_end+...
			 +hat(p5_end-p_ct)*n5_end+hat(p6_end-p_ct)*n6_end+...
			 +m1_end+m2_end+m3_end+m4_end+m5_end+m6_end)-L];


			 %          res_eq_mid = [(n7_end+n8_end+n9_end+n10_end+n11_end+n12_end)-(n1_init+n2_init+n3_init+n4_init+n5_init+n6_init);
			 %             (hat(p7_end-p_cb)*n7_end+hat(p8_end-p_cb)*n8_end+...
			 %              +hat(p9_end-p_cb)*n9_end+hat(p10_end-p_cb)*n10_end+...
			 %              +hat(p11_end-p_cb)*n11_end+hat(p12_end-p_cb)*n12_end+...
			 %              +m1_end+m2_end+m3_end+m4_end+m5_end+m6_end)-...
			 %              (hat(p1_init-p_cb)*n1_init+hat(p2_init-p_cb)*n2_init+...
			 %              +hat(p3_init-p_cb)*n3_init+hat(p4_init-p_cb)*n4_init+...
			 %              +hat(p5_init-p_cb)*n5_init+hat(p6_init-p_cb)*n6_init+...
			 %              +m1_init+m2_init+m3_init+m4_init+m5_init+m6_end)];


			 %These are analogous to loop closure equations because they are
			 %only satisfied when the positions of the rod ends have the same
			 %relative positions as the connection pattern in the top plate
			 res_p1=(pd+Rd*p1_final-p1_end);
			 res_p2=(pd+Rd*p2_final-p2_end);
			 res_p3=(pd+Rd*p3_final-p3_end);
			 res_p4=(pd+Rd*p4_final-p4_end);
			 res_p5=(pd+Rd*p5_final-p5_end);
			 res_p6=(pd+Rd*p6_final-p6_end);
			 res_p7=(p7_end-R7_end*(p7_final-p7_final)-p7_end);  % TODO: always evaluates to 0. Can remove?
			 res_p8=(p7_end-R7_end*(p7_final-p8_final)-p8_end);
			 res_p9=(p7_end-R7_end*(p7_final-p9_final)-p9_end);
			 res_p10=(p7_end-R7_end*(p7_final-p10_final)-p10_end);
			 res_p11=(p7_end-R7_end*(p7_final-p11_final)-p11_end);
			 res_p12=(p7_end-R7_end*(p7_final-p12_final)-p12_end);

			 %force a common material orientation for all the distal rod ends
			 res_R1=real(vee(logm(Rd'*R1_end)));
			 res_R2=real(vee(logm(Rd'*R2_end)));
			 res_R3=real(vee(logm(Rd'*R3_end)));
			 res_R4=real(vee(logm(Rd'*R4_end)));
			 res_R5=real(vee(logm(Rd'*R5_end)));
			 res_R6=real(vee(logm(Rd'*R6_end)));
			 res_R7=real(vee(logm(R7_end'*R7_end))); % TODO: always evaluates to 0. Can remove?
			 res_R8=real(vee(logm(R8_end'*R7_end)));
			 res_R9=real(vee(logm(R9_end'*R7_end)));
			 res_R10=real(vee(logm(R10_end'*R7_end)));
			 res_R11=real(vee(logm(R11_end'*R7_end)));
			 res_R12=real(vee(logm(R12_end'*R7_end)));
			 %todo: that matrix before the log for the above constraints?


			 l_c=0.01; %characteristic length converts rotation error to meters


			 res=[res_eq_top;res_p1;res_p2;res_p3;res_p4;res_p5;res_p6;res_p7;res_p8;res_p9;res_p10;res_p11;res_p12;
			 res_R1*l_c;res_R2*l_c;res_R3*l_c;res_R4*l_c;res_R5*l_c;res_R6*l_c;res_R7*l_c;res_R8*l_c;res_R9*l_c;res_R10*l_c;res_R11*l_c;res_R12*l_c];
			 *
			 */

			//residual[0] = 10.0 - x[0];
			return true;
		}
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
	};

private:
	ros::NodeHandle node; /*!< a handle for this ROS node */

	IKFunctor* ikfunctor;

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
