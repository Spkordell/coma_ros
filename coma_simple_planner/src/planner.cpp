#include "coma_simple_planner/planner.h"

planner::planner() {

	// create the ROS service
	plannerService = n.advertiseService("plan_path", &planner::plan_path, this);
	solverClient = n.serviceClient < coma_kinematics::solveIK > ("solve_ik");

	ROS_INFO("Planner Node Started");
}

bool planner::plan_path(coma_simple_planner::path_request::Request &req, coma_simple_planner::path_request::Response &res) {
	coma_kinematics::solveIK srv;
	srv.request.x_pos = req.start_x_pos;
	srv.request.y_pos = req.start_y_pos;
	srv.request.z_pos = req.start_z_pos;
	srv.request.x_rot = req.start_x_rot;
	srv.request.y_rot = req.start_y_rot;
	srv.request.z_rot = req.start_z_rot;
	if (solverClient.call(srv)) {
		res.config = std::vector<coma_simple_planner::configuration>(2);
		for (unsigned int leg; leg < 12; leg++) {
			res.config[0].leg_lengths[leg] = srv.response.leg_lengths[leg];
		}
	}
	res.config[0].wrist_flex = srv.response.wrist_flex;
	res.config[0].wrist_rot = srv.response.wrist_rot;

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");

	planner coma_planner;

	ros::spin();
	return EXIT_SUCCESS;
}
