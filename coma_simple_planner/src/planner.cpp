#include "coma_simple_planner/planner.h"

planner::planner() {
	ROS_INFO("Planner Node Started");
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");

	planner coma_planner;

	ros::spin();
	return EXIT_SUCCESS;
}
