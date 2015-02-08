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
//#include <time.h>

using namespace std;
using namespace boost::numeric::odeint;

template <typename T> Eigen::Matrix<T, 3, 3> cosserat_rod<T>::hat(Eigen::Matrix<T, 3, 1> u) {
	Eigen::Matrix<T, 3, 3> uhat;
	uhat << T(0), -u(2), u(1), u(2), T(0), -u(0), -u(1), u(0), T(0);
	return uhat;
}

template <typename T> Eigen::Matrix<T, 3, 1> cosserat_rod<T>::vee(Eigen::Matrix<T, 3, 3> uhat) {
	Eigen::Matrix<T, 3, 1> u;
	u << uhat(2, 1), uhat(0, 2), uhat(1, 0);
	return u;
}






