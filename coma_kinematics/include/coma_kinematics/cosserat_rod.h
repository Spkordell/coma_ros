/*!
 * \cosserat_rod.h
 * \brief
 *
 *
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 19, 2014
 */

#ifndef COSSERAT_ROD_H_
#define COSSERAT_ROD_H_

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <math.h>
#include <functional>

typedef boost::array<double, 18> state_type; /* The type of container used to hold the state vector */

class cosserat_rod {
public:
	cosserat_rod(Eigen::Matrix<double, 18, 1> init_state);
	Eigen::Matrix<double, 18, 1>  integrate(double start, double end, double dt);
	static Eigen::Matrix3d hat(Eigen::Vector3d u);
	static Eigen::Vector3d vee(Eigen::Matrix3d uhat);

private:

	void write_deriv(const state_type &x, const double t);
	void deriv(const state_type &x, state_type &dxdt, double t);

	Eigen::Matrix<double, 18, 1> init_state;

	//physical parameters of the legs
	double ro;	// outer radius m
	double ri;	// inner radius mc
	double I;	//second moment of area
	double A;	//area
	double J;	//polar moment
	double E;	//Pa Youngs mod
	double G;	//Pa shear mod
	Eigen::Matrix3d K_bt_inv;
	Eigen::Matrix3d K_se_inv;



};
#endif //COSSERAT_ROD_H_
