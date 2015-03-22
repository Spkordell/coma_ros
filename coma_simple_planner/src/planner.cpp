#include "coma_simple_planner/planner.h"

planner::planner() {

	// create the ROS service
	plannerService = n.advertiseService("plan_path", &planner::plan_path, this);
	solverClient = n.serviceClient < coma_kinematics::solveIK > ("solve_ik");

	ROS_INFO("Planner Node Started");
}

bool planner::plan_path(coma_simple_planner::path_request::Request &req, coma_simple_planner::path_request::Response &res) {
	int numSteps = 10;
	tf::Quaternion start_quart;
	tf::Quaternion goal_quart;
	start_quart.setRPY(req.start_x_rot,req.start_y_rot,req.start_z_rot);
	goal_quart.setRPY(req.goal_x_rot,req.goal_y_rot,req.goal_z_rot);
	coma_kinematics::solveIK srv;
	res.config = std::vector < coma_simple_planner::configuration > (numSteps);
	for (unsigned int i = 0; i < numSteps; i++) {
		srv.request.x_pos = i * ((req.goal_x_pos - req.start_x_pos) / numSteps) + req.start_x_pos;
		srv.request.y_pos = i * ((req.goal_y_pos - req.start_y_pos) / numSteps) + req.start_y_pos;
		srv.request.z_pos = i * ((req.goal_z_pos - req.start_z_pos) / numSteps) + req.start_z_pos;
//		srv.request.x_rot = i * ((req.goal_x_rot - req.start_x_rot) / numSteps) + req.start_x_rot;
//		srv.request.y_rot = i * ((req.goal_y_rot - req.start_y_rot) / numSteps) + req.start_y_rot;
//		srv.request.z_rot = i * ((req.goal_z_rot - req.start_z_rot) / numSteps) + req.start_z_rot;
		tf::Matrix3x3 m(start_quart.slerp(goal_quart,i/numSteps));
		m.getRPY(srv.request.x_rot, srv.request.y_rot, srv.request.z_rot);
		if (solverClient.call(srv)) {
			for (unsigned int leg = 0; leg < 12; leg++) {
				res.config[i].leg_lengths[leg] = srv.response.leg_lengths[leg];
			}
			res.config[i].wrist_flex = srv.response.wrist_flex;
			res.config[i].wrist_rot = srv.response.wrist_rot;
		}
	}

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "planner");

	planner coma_planner;

	ros::spin();
	return EXIT_SUCCESS;
}
