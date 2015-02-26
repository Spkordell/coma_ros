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

#include "ceres/ceres.h"
#include "coma_kinematics/defines.h"

template<typename T> class cosserat_rod {
	typedef boost::array<T, 18> state_type; /* The type of container used to hold the state vector */
	//typedef Eigen::Matrix<T, 18, 1> state_type; /* The type of container used to hold the state vector */

public:
	void set_init_state(Eigen::Matrix<T, 18, 1> init_state);
	Eigen::Matrix<T, 18, 1> integrate(const T start, const T end, const T dt);

	static Eigen::Matrix<T, 3, 3> hat(Eigen::Matrix<T, 3, 1> u);
	static Eigen::Matrix<T, 3, 1> vee(Eigen::Matrix<T, 3, 3> uhat);

	Eigen::Matrix<T, 18, 1> result;

private:
	void write_deriv(const state_type &x, const T t);
	void deriv(const state_type &x, state_type &dxdt, T t);

	state_type init_state;

	//physical parameters of the legs
	T ro;	// outer radius m
	T ri;	// inner radius mc
	T I;	//second moment of area
	T A;	//area
	T J;	//polar moment
	T E;	//Pa Youngs mod
	T G;	//Pa shear mod
	Eigen::Matrix<T, 3, 3> K_bt_inv;
	Eigen::Matrix<T, 3, 3> K_se_inv;

};

template<typename T> Eigen::Matrix<T, 3, 3> cosserat_rod<T>::hat(Eigen::Matrix<T, 3, 1> u) {
	Eigen::Matrix<T, 3, 3> uhat;
	uhat << T(0), -u(2), u(1), u(2), T(0), -u(0), -u(1), u(0), T(0);
	return uhat;
}

template<typename T> Eigen::Matrix<T, 3, 1> cosserat_rod<T>::vee(Eigen::Matrix<T, 3, 3> uhat) {
	Eigen::Matrix<T, 3, 1> u;
	u << uhat(2, 1), uhat(0, 2), uhat(1, 0);
	return u;
}

template<typename T> void cosserat_rod<T>::set_init_state(Eigen::Matrix<T, 18, 1> init_state) {
	//this->init_state = Tstate<T>(init_state);
	for (unsigned int i = 0; i < 18; i++) {
		this->init_state[i] = init_state[i];
	}

	//ro = T(.0018034 / 2);							// outer radius m
	ro = T(0.00198 / 2);							// outer radius m
	ri = T(0.00);									// inner radius mc
	I = T(0.25 * M_PI * (pow(ro, 4) - pow(ri, 4)));	//second moment of area
	A = T(M_PI * (pow(ro, 2) - pow(ri, 2)));		//area
	J = T(T(2) * I);								//polar moment
	E = T(207 * pow(10, 9));						//Pa Youngs mod
	G = T(79.3 * pow(10, 9));						//Pa shear mod
	K_bt_inv << T(1) / (E * I), T(0), T(0), T(0), T(1) / (E * I), T(0), T(0), T(0), T(1) / (J * G);
	K_se_inv << T(1) / (G * A), T(0), T(0), T(0), T(1) / (G * A), T(0), T(0), T(0), T(1) / (E * A);
}

template<typename T> void cosserat_rod<T>::deriv(const state_type &x, state_type &dxdt, T t) {
	using Eigen::Matrix;
	typedef Matrix<T, 3, 3> Matrix3t;
	typedef Matrix<T, 3, 1> Vector3t;

	//turn ODE input into named variables
	//Vector3t p;
	//p << x[0], x[1], x[2];
	Matrix3t R;
	R << x[3], x[6], x[9], x[4], x[7], x[10], x[5], x[8], x[11];
	Vector3t n;
	n << x[12], x[13], x[14];
	Vector3t m;
	m << x[15], x[16], x[17];

	//compute kinematic variables from the internal force and moment using the linear constitutive law
	Vector3t v;
	v << T(0), T(0), T(1);
	v = K_se_inv * R.transpose() * n + v;
	Vector3t u = K_bt_inv * R.transpose() * m;

	//compute state variable derivatives from cosserat rod equations
	Vector3t p_dot = R * v;
	Matrix3t R_dot = R * hat(u);
	Vector3t n_dot;
	n_dot << T(0), T(0), T(0);
	Vector3t m_dot = -p_dot.cross(n);

	//pack back into vector //todo: clean this
	dxdt[0] = p_dot(0);
	dxdt[1] = p_dot(1);
	dxdt[2] = p_dot(2);
	dxdt[3] = R_dot(0, 0);
	dxdt[4] = R_dot(1, 0);
	dxdt[5] = R_dot(2, 0);
	dxdt[6] = R_dot(0, 1);
	dxdt[7] = R_dot(1, 1);
	dxdt[8] = R_dot(2, 1);
	dxdt[9] = R_dot(0, 2);
	dxdt[10] = R_dot(1, 2);
	dxdt[11] = R_dot(2, 2);
	dxdt[12] = n_dot(0);
	dxdt[13] = n_dot(1);
	dxdt[14] = n_dot(2);
	dxdt[15] = m_dot(0);
	dxdt[16] = m_dot(1);
	dxdt[17] = m_dot(2);
}

template<typename T> void cosserat_rod<T>::write_deriv(const state_type &x, const T t) {
//	cout << t;
//	for (unsigned int i = 0; i < 18; i++) {
//		cout << '\t' << x[i];
//	}
//	cout << endl;
}

namespace ceres {
template<typename M, int N> inline bool operator>(const Jet<M, N>& f, int g) {
	return f > ceres::Jet<M, N>(g);
}

template<typename M, int N> inline Jet<M, N> operator/(const Jet<M, N>& f, int g) {
	return f / ceres::Jet<M, N>(g);
}

template<typename M, int N> inline Jet<M, N> max(const double f, const Jet<M, N>& g) {
	return (Jet<M, N>(f) > g) ? Jet<M, N>(f) : g;
}

}

//todo, generalize these, for values other than 84 and 7
//todo, may want to extend from numeric_limits<T> intead and only change the epsilon and is_specialized values.
//need to set the numeric limits of the jet types since odeint uses it
namespace std {
template<>
class numeric_limits<ceres::Jet<double, GS>> : public numeric_limits<double> {
};
}
namespace std {
template<>
class numeric_limits<ceres::Jet<double, SGS>> : public numeric_limits<double> {
};
}

template<typename T> class Toperations: public boost::numeric::odeint::default_operations {
public:

	template<class Fac1 = double, class Fac2 = Fac1>
	struct scale_sum2 {
		const double m_alpha1;
		const Fac2 m_alpha2;
		scale_sum2(double alpha1, Fac2 alpha2) :
				m_alpha1(alpha1), m_alpha2(alpha2) {
		}
		template<class T1, class T2, class T3>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3;
		}
		typedef void result_type;
	};
	template<class Fac1 = double, class Fac2 = Fac1, class Fac3 = Fac2>
	struct scale_sum3 {
		const double m_alpha1;
		const Fac2 m_alpha2;
		const Fac3 m_alpha3;
		scale_sum3(double alpha1, Fac2 alpha2, Fac3 alpha3) :
				m_alpha1(alpha1), m_alpha2(alpha2), m_alpha3(alpha3) {
		}
		template<class T1, class T2, class T3, class T4>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3, const T4 &t4) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4;
		}
		typedef void result_type;
	};
	template<class Fac1 = double, class Fac2 = Fac1, class Fac3 = Fac2, class Fac4 = Fac3>
	struct scale_sum4 {
		const double m_alpha1;
		const Fac2 m_alpha2;
		const Fac3 m_alpha3;
		const Fac4 m_alpha4;
		scale_sum4(double alpha1, Fac2 alpha2, Fac3 alpha3, Fac4 alpha4) :
				m_alpha1(alpha1), m_alpha2(alpha2), m_alpha3(alpha3), m_alpha4(alpha4) {
		}
		template<class T1, class T2, class T3, class T4, class T5>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3, const T4 &t4, const T5 &t5) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5;
		}
		typedef void result_type;
	};

	template<class Fac1 = double, class Fac2 = Fac1, class Fac3 = Fac2, class Fac4 = Fac3, class Fac5 = Fac4>
	struct scale_sum5 {
		const double m_alpha1;
		const Fac2 m_alpha2;
		const Fac3 m_alpha3;
		const Fac4 m_alpha4;
		const Fac5 m_alpha5;
		scale_sum5(double alpha1, Fac2 alpha2, Fac3 alpha3, Fac4 alpha4, Fac5 alpha5) :
				m_alpha1(alpha1), m_alpha2(alpha2), m_alpha3(alpha3), m_alpha4(alpha4), m_alpha5(alpha5) {
		}
		template<class T1, class T2, class T3, class T4, class T5, class T6>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3, const T4 &t4, const T5 &t5, const T6 &t6) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6;
		}
		typedef void result_type;
	};

	template<class Fac1 = double, class Fac2 = Fac1, class Fac3 = Fac2, class Fac4 = Fac3, class Fac5 = Fac4, class Fac6 = Fac5>
	struct scale_sum6 {
		const Fac1 m_alpha1;
		const Fac2 m_alpha2;
		const Fac3 m_alpha3;
		const Fac4 m_alpha4;
		const Fac5 m_alpha5;
		const Fac6 m_alpha6;
		scale_sum6(Fac1 alpha1, Fac2 alpha2, Fac3 alpha3, Fac4 alpha4, Fac5 alpha5, Fac6 alpha6) :
				m_alpha1(alpha1), m_alpha2(alpha2), m_alpha3(alpha3), m_alpha4(alpha4), m_alpha5(alpha5), m_alpha6(alpha6) {
		}

		template<class T1, class T2, class T3, class T4, class T5, class T6, class T7>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3, const T4 &t4, const T5 &t5, const T6 &t6, const T7 &t7) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7;
		}
		typedef void result_type;
	};

	template<class Fac1 = double, class Fac2 = Fac1, class Fac3 = Fac2, class Fac4 = Fac3, class Fac5 = Fac4, class Fac6 = Fac5, class Fac7 = Fac6>
	struct scale_sum7 {
		const double m_alpha1;
		const Fac2 m_alpha2;
		const Fac3 m_alpha3;
		const Fac4 m_alpha4;
		const Fac5 m_alpha5;
		const Fac6 m_alpha6;
		const Fac7 m_alpha7;
		scale_sum7(double alpha1, Fac2 alpha2, Fac3 alpha3, Fac4 alpha4, Fac5 alpha5, Fac6 alpha6, Fac7 alpha7) :
				m_alpha1(alpha1), m_alpha2(alpha2), m_alpha3(alpha3), m_alpha4(alpha4), m_alpha5(alpha5), m_alpha6(alpha6), m_alpha7(alpha7) {
		}
		template<class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
		void operator()(T1 &t1, const T2 &t2, const T3 &t3, const T4 &t4, const T5 &t5, const T6 &t6, const T7 &t7, const T8 &t8) const {
			t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8;
		}
		typedef void result_type;
	};
};

template<typename T> Eigen::Matrix<T, 18, 1> cosserat_rod<T>::integrate(const T start, const T end, const T dt) {
	namespace pl = std::placeholders;
	namespace od = boost::numeric::odeint;

	//typedef od::runge_kutta_dopri5<state_type> stepper_type;
	//typedef od::runge_kutta_dopri5<state_type, T, state_type, T, od::range_algebra, Toperations<T>> stepper_type;


	typedef od::runge_kutta_dopri5<state_type, T, state_type, T, od::range_algebra, Toperations<T>> stepper_type;
	//typedef od::runge_kutta4<state_type, T, state_type, T, od::range_algebra, Toperations<T>> stepper_type;

	//typedef od::runge_kutta_dopri5<state_type, T, state_type, T, od::vector_space_algebra> stepper_type; //use if state type is Eigen

	od::integrate_const(od::make_dense_output < stepper_type > (T(1E-6), T(1E-3)), std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3), init_state,
			start, end, dt, std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2));

	for (unsigned int i = 0; i < 18; i++) {
		result[i] = init_state[i];
	}

	return result;
}
#endif //COSSERAT_ROD_H_
