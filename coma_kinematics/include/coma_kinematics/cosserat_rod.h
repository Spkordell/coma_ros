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

//#include </usr/include/boost/numeric/odeint/algebra/array_algebra.hpp>

#include <Eigen/Dense>
#include <math.h>
#include <functional>

#include "ceres/ceres.h"

//template<typename T> class Tstate/*: boost::additive1<Tstate<T>, boost::additive2<Tstate<T>, double, boost::multiplicative2<Tstate<T>, double> > > */{
//public:
//public:
//	Eigen::Matrix<T, 18, 1> state;
//
//	Tstate() {
//	}
//
//	Tstate(const Eigen::Matrix<T, 18, 1> val) {
//		state = val;
//	}
//
//	Tstate& operator+=(const Tstate &p) {
//		for (unsigned int i; i < 18; i++) {
//			state[i] += p[i];
//		}
//		return *this;
//	}
//
//	Tstate& operator*=(const T a) {
//		for (unsigned int i; i < 18; i++) {
//			state[i] *= a;
//		}
//		return *this;
//	}
//
//	friend Tstate operator*(const T &a, const Tstate &b) {
//		Tstate r;
//		for (unsigned int i; i < 18; i++) {
//			r.state[i] = a * b.state[i];
//		}
//		return r;
//	}
//
//	friend Tstate operator+(const Tstate &a, const Tstate &b) {
//		Tstate r;
//		for (unsigned int i; i < 18; i++) {
//			r.state[i] = a.state[i] + b.state[i];
//		}
//		return r;
//	}
//
//	friend Tstate operator+(const T &a, const Tstate &b) {
//		Tstate r;
//		for (unsigned int i; i < 18; i++) {
//			r.state[i] = a + b.state[i];
//		}
//		return r;
//	}
//
//	// only required for steppers with error control
//	friend Tstate operator/(const Tstate &p1, const Tstate &p2) {
//		Tstate r;
//		for (unsigned int i = 0; i < 18; i++) {
//			r.state[i] = p1.state[i] / p2.state[i];
//		}
//		return r;
//	}
//
//	T operator [](int i) const {
//		return state[i];
//	}
//
//	friend Tstate abs(const Tstate &p) {
//		Tstate r;
//		for (unsigned int i = 0; i < 18; i++) {
//			r.state[i] = abs(p.state[i]);
//		}
//		return r;
//	}
//
//};
//
//namespace boost {
//namespace numeric {
//namespace odeint {
//// specialization of vector_space_reduce, only required for steppers with error control
//template<typename T>
//struct vector_space_reduce<Tstate<T> > {
//	template<class Value, class Op>
//	Value operator()(const Tstate<T> &p, Op op, Value init) {
//		for (unsigned int i = 0; i < 18; i++) {
//			init = op(init, p[i]);
//		}
//		return init;
//	}
//};
//}
//}
//}

template<typename T> class cosserat_rod {
	typedef boost::array<T, 18> state_type; /* The type of container used to hold the state vector */
	//typedef Eigen::Matrix<T, 18, 1> state_type; /* The type of container used to hold the state vector */
	//typedef Tstate<T> state_type; /* The type of container used to hold the state vector */

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

template<typename T> void cosserat_rod<T>::set_init_state(Eigen::Matrix<T, 18, 1> init_state) {
	//this->init_state = Tstate<T>(init_state);
	for (unsigned int i = 0; i < 18; i++) {
		this->init_state[i] = init_state[i];
	}

	ro = T(.0018034 / 2);							// outer radius m
	ri = T(0.00);									// inner radius mc
	I = T(0.25 * M_PI * (pow(ro, 4) - pow(ri, 4)));	//second moment of area
	A = T(M_PI * (pow(ro, 2) - pow(ri, 2)));			//area
	J = T(T(2) * I);										//polar moment
	E = T(207 * pow(10, 9));							//Pa Youngs mod
	G = T(79.3 * pow(10, 9));							//Pa shear mod
	K_bt_inv << T(1) / (E * I), T(0), T(0), T(0), T(1) / (E * I), T(0), T(0), T(0), T(1) / (J * G);
	K_se_inv << T(1) / (G * A), T(0), T(0), T(0), T(1) / (G * A), T(0), T(0), T(0), T(1) / (E * A);
}

template<typename T> void cosserat_rod<T>::deriv(const state_type &x, state_type &dxdt, T t) {
	//turn ODE input into named variables
	//Vector3d p(x[0], x[1], x[2]);
//	Matrix3d R;
//	R << x[3], x[6], x[9], x[4], x[7], x[10], x[5], x[8], x[11];
//	Vector3d n(x[12], x[13], x[14]);
//	Vector3d m(x[15], x[16], x[17]);
//
//	//compute kinematic variables from the internal force and moment using the linear constitutive law
//	Vector3d v(0, 0, 1);
//	v = K_se_inv * R.transpose() * n + v;
//	Vector3d u = K_bt_inv * R.transpose() * m;
//
//	//compute state variable derivatives from cosserat rod equations
//	Vector3d p_dot = R * v;
//	Matrix3d R_dot = R * hat(u);
//	Vector3d n_dot(0, 0, 0);
//	Vector3d m_dot = -p_dot.cross(n);
//
//	//pack back into vector //todo: clean this
//	dxdt[0] = p_dot(0);
//	dxdt[1] = p_dot(1);
//	dxdt[2] = p_dot(2);
//	dxdt[3] = R_dot(0, 0);
//	dxdt[4] = R_dot(1, 0);
//	dxdt[5] = R_dot(2, 0);
//	dxdt[6] = R_dot(0, 1);
//	dxdt[7] = R_dot(1, 1);
//	dxdt[8] = R_dot(2, 1);
//	dxdt[9] = R_dot(0, 2);
//	dxdt[10] = R_dot(1, 2);
//	dxdt[11] = R_dot(1, 2);
//	dxdt[12] = n_dot(0);
//	dxdt[13] = n_dot(1);
//	dxdt[14] = n_dot(2);
//	dxdt[15] = m_dot(0);
//	dxdt[16] = m_dot(1);
//	dxdt[17] = m_dot(2);
}

template<typename T> void cosserat_rod<T>::write_deriv(const state_type &x, const T t) {
//	cout << t;
//	for (unsigned int i = 0; i < 18; i++) {
//		cout << '\t' << x[i];
//	}
//	cout << endl;
}
//
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

//template<typename M, int N> inline operator int(const Jet<M, N>& g) {
//	return 1;
//}

//template<typename M, int N> inline operator Jet<M, N>(const double a) {
//  return Jet<M, N>(a);
//}

//template<typename M, int N> inline Jet<M, N> operator*(const Jet<M, N>& f, size_t g) {
//	return f * ceres::Jet<M, N>(g);
//}

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
	//typedef od::runge_kutta_dopri5<state_type, T, state_type, T, od::array_algebra> stepper_type;

	//typedef od::runge_kutta_dopri5<state_type, double, state_type, double, od::range_algebra> stepper_type;

	//typedef od::runge_kutta_dopri5<state_type, T, state_type, T, od::vector_space_algebra> stepper_type;
//	typedef od::runge_kutta_dopri5<state_type,double,state_type,double,od::vector_space_algebra> stepper_type;

	//todo: optimize this
//	state_type init_state_;
//	for (unsigned int i = 0; i < 18; i++) {
//		init_state_[i] = init_state[i];
//	}

	//std::cout << "s:" << start << "\te:" << end << "\td:" << dt << "\tinit:" << init_state.transpose() << std::endl;
	//int start_s=clock();

//	std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3);
//	std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2);
//	od::make_dense_output < stepper_type > (T(1E-6), T(1E-3));

	od::integrate_const(od::make_dense_output < stepper_type > (T(1E-6), T(1E-3)), std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3), init_state,
			start, end, dt, std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2));

//	od::integrate_const(od::make_dense_output < stepper_type > (1E-6, 1E-3), std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3), init_state, start, end,
//			dt, std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2));

//		od::integrate_const(stepper_type(), std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3), init_state, start, end,
//				dt, std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2));

	//todo: parameterize number of steps
//		od::integrate_n_steps(stepper_type() , std::bind(&cosserat_rod::deriv, *this, pl::_1, pl::_2, pl::_3), init_state, start, dt, 10 ,
//		        std::bind(&cosserat_rod::write_deriv, *this, pl::_1, pl::_2));

	//int stop_s=clock();
	//cout << "time: " << double( stop_s - start_s) / double(CLOCKS_PER_SEC)<< " seconds." << endl;

	//todo: can probably remove this since state type is now Eigen
	for (unsigned int i = 0; i < 18; i++) {
		//result[i] = init_state_[i];
		result[i] = init_state[i];
	}

	//std::cout << "result:"  << result.transpose() << std::endl << std::endl;

	return result;
}
#endif //COSSERAT_ROD_H_
