#include "coma_rviz/visualization.h"

visualization::visualization() {

	rod_pos_sub = n.subscribe < coma_rviz::vis > ("/rod_pos", 10, &visualization::rodposCallback, this);
	marker_pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 500);

	ROS_INFO("Visualization Node Started");
}

void visualization::rodposCallback(const coma_rviz::vis::ConstPtr& msg) {
	visualization_msgs::Marker points, leg[12], top_plate, mid_plate;
	points.header.frame_id = top_plate.header.frame_id = mid_plate.header.frame_id = "/base";
	points.header.stamp = top_plate.header.stamp = mid_plate.header.stamp = ros::Time::now();
	points.ns = "points";
	top_plate.ns = "top plate";
	mid_plate.ns = "bottom plate";
	points.action = top_plate.action = mid_plate.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = top_plate.pose.orientation.w = mid_plate.pose.orientation.w = 1.0;
	points.id = 0;
	top_plate.id = 13;
	mid_plate.id = 14;
	points.type = visualization_msgs::Marker::POINTS;
	top_plate.type = mid_plate.type = visualization_msgs::Marker::LINE_STRIP;
	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.002;
	points.scale.y = 0.002;
	top_plate.scale.x = 0.002;
	mid_plate.scale.x = 0.002;
	// Points are black
	points.color.a = 1.0;
	//plates are black
	mid_plate.color.a = 1.0;
	top_plate.color.a = 1.0;

	for (unsigned int i = 0; i < 12; i++) {
		leg[i].header.frame_id = points.header.frame_id;
		leg[i].header.stamp = points.header.stamp;
		leg[i].ns = "leg " + boost::lexical_cast < std::string > (i+1);
		leg[i].action = points.action;
		leg[i].pose.orientation.w = points.pose.orientation.w;

		leg[i].id = i + 1;
		leg[i].type = visualization_msgs::Marker::LINE_STRIP;

		//LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		leg[i].scale.x = 0.002;

		//legs are blue
		leg[i].color.a = 1.0;
	}

	//set leg colors
	leg[0].color.r = 1.0;leg[0].color.g = 0.0;leg[0].color.b = 0.0;    //red
	leg[1].color.r = 0.0;leg[1].color.g = 0.0;leg[1].color.b = 1.0;    //blue
	leg[2].color.r = 0.0;leg[2].color.g = 1.0;leg[2].color.b = 0.0;    //green
	leg[3].color.r = 1.0;leg[3].color.g = 0.0;leg[3].color.b = 1.0;    //magenta
	leg[4].color.r = 0.0;leg[4].color.g = 1.0;leg[4].color.b = 1.0;    //cyan
	leg[5].color.r = 1.0;leg[5].color.g = 1.0;leg[5].color.b = 0.0;    //yellow
	leg[6].color.r = 1.0;leg[6].color.g = 0.0;leg[6].color.b = 0.0;    //red
	leg[7].color.r = 0.0;leg[7].color.g = 0.0;leg[7].color.b = 1.0;    //blue
	leg[8].color.r = 0.0;leg[8].color.g = 1.0;leg[8].color.b = 0.0;    //green
	leg[9].color.r = 1.0;leg[9].color.g = 0.0;leg[9].color.b = 1.0;    //magenta
	leg[10].color.r = 0.0;leg[10].color.g = 1.0;leg[10].color.b = 1.0; //cyan
	leg[11].color.r = 1.0;leg[11].color.g = 1.0;leg[11].color.b = 0.0; //yellow



	// Create the vertices for the points and lines
	for (unsigned int i = 0; i < 12; i++) {
		for (unsigned int j = 0; j < 20; j++) {
			geometry_msgs::Point p;
			p.x = msg->rod[i].x[j];
			p.y = msg->rod[i].y[j];
			p.z = msg->rod[i].z[j];

			points.points.push_back(p);
			leg[i].points.push_back(p);
		}
		marker_pub.publish(leg[i]);

		geometry_msgs::Point p;
		p.x = msg->rod[i].x[19];
		p.y = msg->rod[i].y[19];
		p.z = msg->rod[i].z[19];
		if (i < 6) {
			top_plate.points.push_back(p);
		} else {
			mid_plate.points.push_back(p);
		}
	}
	//close loops for top/bottom plate
	geometry_msgs::Point p;
	p.x = msg->rod[0].x[19];
	p.y = msg->rod[0].y[19];
	p.z = msg->rod[0].z[19];
	top_plate.points.push_back(p);
	p.x = msg->rod[6].x[19];
	p.y = msg->rod[6].y[19];
	p.z = msg->rod[6].z[19];
	mid_plate.points.push_back(p);

	marker_pub.publish(points);
	marker_pub.publish(top_plate);
	marker_pub.publish(mid_plate);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "points_and_lines");

	visualization vis;

	ros::spin();
	return EXIT_SUCCESS;

//	ros::Rate r(30);
//
//	float f = 0.0;
//	while (ros::ok()) {

//		visualization_msgs::Marker points, line_strip, line_list;
//		points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base";
//		points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
//		points.ns = line_strip.ns = line_list.ns = "points_and_lines";
//		points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
//		points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
//
//		points.id = 0;
//		line_strip.id = 1;
//		line_list.id = 2;
//
//		points.type = visualization_msgs::Marker::POINTS;
//		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//		line_list.type = visualization_msgs::Marker::LINE_LIST;
//
//		// POINTS markers use x and y scale for width/height respectively
//		points.scale.x = 0.2;
//		points.scale.y = 0.2;
//
//		// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//		line_strip.scale.x = 0.1;
//		line_list.scale.x = 0.1;
//
//		// Points are green
//		points.color.g = 1.0f;
//		points.color.a = 1.0;
//
//		// Line strip is blue
//		line_strip.color.b = 1.0;
//		line_strip.color.a = 1.0;
//
//		// Line list is red
//		line_list.color.r = 1.0;
//		line_list.color.a = 1.0;
//
//		// Create the vertices for the points and lines
//		for (uint32_t i = 0; i < 100; ++i) {
//			float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//			float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
//
//			geometry_msgs::Point p;
//			p.x = (int32_t) i - 50;
//			p.y = y;
//			p.z = z;
//
//			points.points.push_back(p);
//			line_strip.points.push_back(p);
//
//			// The line list needs two points for each line
//			line_list.points.push_back(p);
//			p.z += 1.0;
//			line_list.points.push_back(p);
//		}
//
//		marker_pub.publish(points);
//		marker_pub.publish(line_strip);
//		marker_pub.publish(line_list);

//		r.sleep();
//
//		//f += 0.04;
//	}
}
