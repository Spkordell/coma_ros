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

typedef boost::array<double, 18> state_type;

class cosserat_rod {
public:
    //physical parameters of the legs
    static double ro;	// outer radius m
    static double ri;	// inner radius mc
    static double I;	//second moment of area
    static double A;	//area
    static double J;	//polar moment
    static double E;	//Pa Youngs mod
    static double G;	//Pa shear mod

    static Eigen::Matrix3d K_bt_inv;
    static Eigen::Matrix3d K_se_inv;

	cosserat_rod();
	void integrate();
	/* The type of container used to hold the state vector */
private:
    static Eigen::Matrix3d hat(Eigen::Vector3d u);
	static void write_deriv(const state_type &x, const double t);
	static void deriv(const state_type &x, state_type &dxdt, double t);
};
#endif //COSSERAT_ROD_H_
