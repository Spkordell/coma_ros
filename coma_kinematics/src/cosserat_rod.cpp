/*!
 * \cosserat_rod.cpp
 * \brief
 *
 *
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 19, 2014
 */

#include <coma_kinematics/cosserat_rod.h>

using namespace std;
using namespace boost::numeric::odeint;
using Eigen::Matrix3d;
using Eigen::Vector3d;


Eigen::Matrix3d cosserat_rod::hat(Eigen::Vector3d u) {
	Matrix3d uhat;
	uhat << 0, -u(2), u(1),
			u(2), 0, -u(0),
			-u(1), u(0), 0;
	return uhat;
}


void cosserat_rod::deriv(const state_type &x, state_type &dxdt, double t) {
	cout << t <<endl;
	//turn ODE input into named variables
	//Vector3d p(x[0], x[1], x[2]);
	Matrix3d R;  R << x[3], x[6], x[9],
					  x[4], x[7], x[10],
					  x[5], x[8], x[11];
	Vector3d n(x[12], x[13], x[14]);
	Vector3d m(x[15], x[16], x[17]);

	//compute kinematic variables from the internal force and moment using the linear constitutive law
	Vector3d v(0, 0, 1);
	v = K_se_inv * R.transpose() * n + v;
	Vector3d u = K_bt_inv * R.transpose() * m;

	//compute state variable derivatives from cosserat rod equations
	Vector3d p_dot = R * v;
	Matrix3d R_dot = R*hat(u);
	Vector3d n_dot(0, 0, 0);
	Vector3d m_dot = -p_dot.cross(n);

	//pack back into vector
	dxdt[0] = p_dot(0);
	dxdt[1] = p_dot(1);
	dxdt[2] = p_dot(2);
	dxdt[3] = R_dot(0,0);
	dxdt[4] = R_dot(1,0);
	dxdt[5] = R_dot(2,0);
	dxdt[6] = R_dot(0,1);
	dxdt[7] = R_dot(1,1);
	dxdt[8] = R_dot(2,1);
	dxdt[9] = R_dot(0,2);
	dxdt[10] = R_dot(1,2);
	dxdt[11] = R_dot(1,2);
	dxdt[12] = n_dot(0);
	dxdt[13] = n_dot(1);
	dxdt[14] = n_dot(2);
	dxdt[15] = m_dot(0);
	dxdt[16] = m_dot(1);
	dxdt[17] = m_dot(2);

}

void cosserat_rod::write_deriv(const state_type &x, const double t) {
	/*
	cout << t;
	for(unsigned int i = 0; i < 18; i++) {
		cout << '\t' << x[i];
	}
	cout <<endl;*/
	//cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << endl;
}

void cosserat_rod::integrate() {
	const double start = 0.0;
	const double end = 20.0;
    const double dt = 0.1;
    typedef runge_kutta_dopri5<state_type> stepper_type;

    state_type x = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // initial conditions
    integrate_const(make_dense_output<stepper_type>( 1E-6, 1E-3 ), deriv, x, start, end, dt, write_deriv);
}

cosserat_rod::cosserat_rod() {
	cosserat_rod::K_bt_inv << 1/(E*I),0,0,0,1/(E*I),0,0,0,1/(J*G);
	cosserat_rod::K_se_inv << 1/(G*A),0,0,0,1/(G*A),0,0,0,1/(E*A);

}

double cosserat_rod::ro = .0018034/2;					// outer radius m
double cosserat_rod::ri = 0.00;							// inner radius mc
double cosserat_rod::I = 1/4*M_PI*(pow(ro,4)-pow(ri,4));	//second moment of area
double cosserat_rod::A = M_PI*(pow(ro,2)-pow(ri,2));		//area
double cosserat_rod::J = 2*I;							//polar moment
double cosserat_rod::E = 207*pow(10,9);					//Pa Youngs mod
double cosserat_rod::G = 79.3*pow(10,9);					//Pa shear mod
Matrix3d cosserat_rod::K_bt_inv;
Matrix3d cosserat_rod::K_se_inv;
