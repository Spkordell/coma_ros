/*!
 * \motion_demo.cpp
 * \brief Sends simple motion commands to coma for demonstration/testing purposes
 *
 * motion_demo creates a ROS node for testing/demonstrating coma
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date January 11, 2015
 */

#include <coma_demo/motion_demo.h>

using namespace std;

#define run_agility

motion_demo::motion_demo() {
	// a private handle for this ROS node (allows retrieval of relative parameters)
	ros::NodeHandle private_nh("~");

	//initialize variables
	response_received = true;

	//Build a list of of stepper positions to go to
#ifdef run_agility
	numCmds = 21; //the number of elements in the list
#endif
#ifndef run_agility
	numCmds = 6; //the number of elements in the list
#endif
	step_cmds = new std::vector<std::vector<unsigned int> >(numCmds, std::vector<unsigned int>(16));
	//element 0..11 = stepper motor steps
	//element 12 = wrist flex in degrees
	//element 13 = wrist rotate in degrees
	//element 14 = wrist open or close (1 = open, 0 = closed)
	//element 15 = delay after sending command in milliseconds

#ifdef run_agility
	//agility stuff
	//Set all steppers to zero position
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(0).at(i) = 0;
	}
	step_cmds->at(0).at(12) = 180;
	step_cmds->at(0).at(13) = 111;
	step_cmds->at(0).at(14) = 0;
	step_cmds->at(0).at(15) = 2000; //delay

	//move all steppers to top
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(1).at(i) = 1000;
	}
	step_cmds->at(1).at(12) = 180;
	step_cmds->at(1).at(13) = 111;
	step_cmds->at(1).at(14) = 1;
	step_cmds->at(1).at(15) = 5000; //delay

	//move all steppers to mid
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(2).at(i) = 500;
	}
	step_cmds->at(2).at(12) = 180;
	step_cmds->at(2).at(13) = 111;
	step_cmds->at(2).at(14) = 0;
	step_cmds->at(2).at(15) = 5000; //delay

	//seperate rods and rotate
	step_cmds->at(3).at(0) = 789;
	step_cmds->at(3).at(1) = 789;
	step_cmds->at(3).at(2) = 789;
	step_cmds->at(3).at(3) = 789;
	step_cmds->at(3).at(4) = 789;
	step_cmds->at(3).at(5) = 789;
	step_cmds->at(3).at(6) = 338;
	step_cmds->at(3).at(7) = 338;
	step_cmds->at(3).at(8) = 338;
	step_cmds->at(3).at(9) = 338;
	step_cmds->at(3).at(10) = 338;
	step_cmds->at(3).at(11) = 338;
	step_cmds->at(3).at(12) = 180;
	step_cmds->at(3).at(13) = 111;
	step_cmds->at(3).at(14) = 0;
	step_cmds->at(3).at(15) = 2000; //delay

	step_cmds->at(4).at(0) = 789;
	step_cmds->at(4).at(1) = 843;
	step_cmds->at(4).at(2) = 789;
	step_cmds->at(4).at(3) = 843;
	step_cmds->at(4).at(4) = 789;
	step_cmds->at(4).at(5) = 843;
	step_cmds->at(4).at(6) = 338;
	step_cmds->at(4).at(7) = 390;
	step_cmds->at(4).at(8) = 338;
	step_cmds->at(4).at(9) = 390;
	step_cmds->at(4).at(10) = 338;
	step_cmds->at(4).at(11) = 390;
	step_cmds->at(4).at(12) = 180;
	step_cmds->at(4).at(13) = 111;
	step_cmds->at(4).at(14) = 0;
	step_cmds->at(4).at(15) = 2000; //delay

	step_cmds->at(5).at(0) = 789;
	step_cmds->at(5).at(1) = 685;
	step_cmds->at(5).at(2) = 789;
	step_cmds->at(5).at(3) = 685;
	step_cmds->at(5).at(4) = 789;
	step_cmds->at(5).at(5) = 685;
	step_cmds->at(5).at(6) = 338;
	step_cmds->at(5).at(7) = 234;
	step_cmds->at(5).at(8) = 338;
	step_cmds->at(5).at(9) = 245;
	step_cmds->at(5).at(10) = 338;
	step_cmds->at(5).at(11) = 234;
	step_cmds->at(5).at(12) = 180;
	step_cmds->at(5).at(13) = 111;
	step_cmds->at(5).at(14) = 0;
	step_cmds->at(5).at(15) = 2000; //delay

	//seperate rods
	step_cmds->at(6).at(0) = 789;
	step_cmds->at(6).at(1) = 789;
	step_cmds->at(6).at(2) = 789;
	step_cmds->at(6).at(3) = 789;
	step_cmds->at(6).at(4) = 789;
	step_cmds->at(6).at(5) = 789;
	step_cmds->at(6).at(6) = 338;
	step_cmds->at(6).at(7) = 338;
	step_cmds->at(6).at(8) = 338;
	step_cmds->at(6).at(9) = 338;
	step_cmds->at(6).at(10) = 338;
	step_cmds->at(6).at(11) = 338;
	step_cmds->at(6).at(12) = 180;
	step_cmds->at(6).at(13) = 111;
	step_cmds->at(6).at(14) = 0;
	step_cmds->at(6).at(15) = 2000; //delay

	//small lateral move
	step_cmds->at(7).at(0) = 782;
	step_cmds->at(7).at(1) = 730;
	step_cmds->at(7).at(2) = 814;
	step_cmds->at(7).at(3) = 846;
	step_cmds->at(7).at(4) = 771;
	step_cmds->at(7).at(5) = 790;
	step_cmds->at(7).at(6) = 391;
	step_cmds->at(7).at(7) = 330;
	step_cmds->at(7).at(8) = 285;
	step_cmds->at(7).at(9) = 383;
	step_cmds->at(7).at(10) = 338;
	step_cmds->at(7).at(11) = 277;
	step_cmds->at(7).at(12) = 180;
	step_cmds->at(7).at(13) = 111;
	step_cmds->at(7).at(14) = 0;
	step_cmds->at(7).at(15) = 2000; //delay

	//recenter
	step_cmds->at(8).at(0) = 789;
	step_cmds->at(8).at(1) = 789;
	step_cmds->at(8).at(2) = 789;
	step_cmds->at(8).at(3) = 789;
	step_cmds->at(8).at(4) = 789;
	step_cmds->at(8).at(5) = 789;
	step_cmds->at(8).at(6) = 338;
	step_cmds->at(8).at(7) = 338;
	step_cmds->at(8).at(8) = 338;
	step_cmds->at(8).at(9) = 338;
	step_cmds->at(8).at(10) = 338;
	step_cmds->at(8).at(11) = 338;
	step_cmds->at(8).at(12) = 180;
	step_cmds->at(8).at(13) = 111;
	step_cmds->at(8).at(14) = 0;
	step_cmds->at(8).at(15) = 2000; //delay

	//small s-curve
	step_cmds->at(9).at(0) = 660;
	step_cmds->at(9).at(1) = 726;
	step_cmds->at(9).at(2) = 872;
	step_cmds->at(9).at(3) = 839;
	step_cmds->at(9).at(4) = 578;
	step_cmds->at(9).at(5) = 545;
	step_cmds->at(9).at(6) = 258;
	step_cmds->at(9).at(7) = 258;
	step_cmds->at(9).at(8) = 258;
	step_cmds->at(9).at(9) = 258;
	step_cmds->at(9).at(10) = 258;
	step_cmds->at(9).at(11) = 258;
	step_cmds->at(9).at(12) = 180;
	step_cmds->at(9).at(13) = 111;
	step_cmds->at(9).at(14) = 0;
	step_cmds->at(9).at(15) = 2000; //delay

	//small s-curve
	step_cmds->at(10).at(0) = 660;
	step_cmds->at(10).at(1) = 726;
	step_cmds->at(10).at(2) = 872;
	step_cmds->at(10).at(3) = 839;
	step_cmds->at(10).at(4) = 548;
	step_cmds->at(10).at(5) = 515;
	step_cmds->at(10).at(6) = 366;
	step_cmds->at(10).at(7) = 252;
	step_cmds->at(10).at(8) = 6;
	step_cmds->at(10).at(9) = 109;
	step_cmds->at(10).at(10) = 402;
	step_cmds->at(10).at(11) = 414;
	step_cmds->at(10).at(12) = 180;
	step_cmds->at(10).at(13) = 111;
	step_cmds->at(10).at(14) = 0;
	step_cmds->at(10).at(15) = 2000; //delay

	//recenter
	step_cmds->at(11).at(0) = 789;
	step_cmds->at(11).at(1) = 789;
	step_cmds->at(11).at(2) = 789;
	step_cmds->at(11).at(3) = 789;
	step_cmds->at(11).at(4) = 789;
	step_cmds->at(11).at(5) = 789;
	step_cmds->at(11).at(6) = 338;
	step_cmds->at(11).at(7) = 338;
	step_cmds->at(11).at(8) = 338;
	step_cmds->at(11).at(9) = 338;
	step_cmds->at(11).at(10) = 338;
	step_cmds->at(11).at(11) = 338;
	step_cmds->at(11).at(12) = 180;
	step_cmds->at(11).at(13) = 111;
	step_cmds->at(11).at(14) = 0;
	step_cmds->at(11).at(15) = 2000; //delay

	//move all steppers to mid
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(12).at(i) = 500;
	}
	step_cmds->at(12).at(12) = 180;
	step_cmds->at(12).at(13) = 111;
	step_cmds->at(12).at(14) = 0;
	step_cmds->at(12).at(15) = 2000; //delay

	//bend
	step_cmds->at(13).at(0) = 774;
	step_cmds->at(13).at(1) = 775;
	step_cmds->at(13).at(2) = 555;
	step_cmds->at(13).at(3) = 505;
	step_cmds->at(13).at(4) = 503;
	step_cmds->at(13).at(5) = 553;
	step_cmds->at(13).at(6) = 792;
	step_cmds->at(13).at(7) = 841;
	step_cmds->at(13).at(8) = 693;
	step_cmds->at(13).at(9) = 602;
	step_cmds->at(13).at(10) = 348;
	step_cmds->at(13).at(11) = 390;
	step_cmds->at(13).at(12) = 180;
	step_cmds->at(13).at(13) = 132;
	step_cmds->at(13).at(14) = 0;
	step_cmds->at(13).at(15) = 2000; //delay

	//move all steppers to mid
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(14).at(i) = 500;
	}
	step_cmds->at(14).at(12) = 180;
	step_cmds->at(14).at(13) = 111;
	step_cmds->at(14).at(14) = 0;
	step_cmds->at(14).at(15) = 2000; //delay

	// rotate over 180 degree workspace
	//bend to one side
	step_cmds->at(15).at(0) = 666;
	step_cmds->at(15).at(1) = 654;
	step_cmds->at(15).at(2) = 646;
	step_cmds->at(15).at(3) = 685;
	step_cmds->at(15).at(4) = 916;
	step_cmds->at(15).at(5) = 889;
	step_cmds->at(15).at(6) = 696;
	step_cmds->at(15).at(7) = 524;
	step_cmds->at(15).at(8) = 133;
	step_cmds->at(15).at(9) = 179;
	step_cmds->at(15).at(10) = 503;
	step_cmds->at(15).at(11) = 629;
	step_cmds->at(15).at(12) = 90;
	step_cmds->at(15).at(13) = 90;
	step_cmds->at(15).at(14) = 0;
	step_cmds->at(15).at(15) = 10;		 //delay;

	step_cmds->at(16).at(0) = 706;
	step_cmds->at(16).at(1) = 680;
	step_cmds->at(16).at(2) = 754;
	step_cmds->at(16).at(3) = 796;
	step_cmds->at(16).at(4) = 909;
	step_cmds->at(16).at(5) = 893;
	step_cmds->at(16).at(6) = 316;
	step_cmds->at(16).at(7) = 218;
	step_cmds->at(16).at(8) = 137;
	step_cmds->at(16).at(9) = 218;
	step_cmds->at(16).at(10) = 653;
	step_cmds->at(16).at(11) = 671;
	step_cmds->at(16).at(12) = 90;
	step_cmds->at(16).at(13) = 90;
	step_cmds->at(16).at(14) = 0;
	step_cmds->at(16).at(15) = 10; //delay

	step_cmds->at(17).at(0) = 428;
	step_cmds->at(17).at(1) = 583;
	step_cmds->at(17).at(2) = 864;
	step_cmds->at(17).at(3) = 860;
	step_cmds->at(17).at(4) = 747;
	step_cmds->at(17).at(5) = 597;
	step_cmds->at(17).at(6) = 150;
	step_cmds->at(17).at(7) = 100;
	step_cmds->at(17).at(8) = 242;
	step_cmds->at(17).at(9) = 323;
	step_cmds->at(17).at(10) = 540;
	step_cmds->at(17).at(11) = 508;
	step_cmds->at(17).at(12) = 90;
	step_cmds->at(17).at(13) = 90;
	step_cmds->at(17).at(14) = 0;
	step_cmds->at(17).at(15) = 10; //delay

	step_cmds->at(18).at(0) = 584;
	step_cmds->at(18).at(1) = 667;
	step_cmds->at(18).at(2) = 993;
	step_cmds->at(18).at(3) = 983;
	step_cmds->at(18).at(4) = 614;
	step_cmds->at(18).at(5) = 540;
	step_cmds->at(18).at(6) = 154;
	step_cmds->at(18).at(7) = 120;
	step_cmds->at(18).at(8) = 704;
	step_cmds->at(18).at(9) = 667;
	step_cmds->at(18).at(10) = 489;
	step_cmds->at(18).at(11) = 559;
	step_cmds->at(18).at(12) = 90;
	step_cmds->at(18).at(13) = 90;
	step_cmds->at(18).at(14) = 0;
	step_cmds->at(18).at(15) = 10; //delay

	step_cmds->at(19).at(0) = 548;
	step_cmds->at(19).at(1) = 667;
	step_cmds->at(19).at(2) = 953;
	step_cmds->at(19).at(3) = 877;
	step_cmds->at(19).at(4) = 353;
	step_cmds->at(19).at(5) = 310;
	step_cmds->at(19).at(6) = 291;
	step_cmds->at(19).at(7) = 374;
	step_cmds->at(19).at(8) = 712;
	step_cmds->at(19).at(9) = 705;
	step_cmds->at(19).at(10) = 337;
	step_cmds->at(19).at(11) = 261;
	step_cmds->at(19).at(12) = 90;
	step_cmds->at(19).at(13) = 90;
	step_cmds->at(19).at(14) = 0;
	step_cmds->at(19).at(15) = 2000; //10 second delay;

	//move all steppers to mid
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(20).at(i) = 500;
	}
	step_cmds->at(20).at(12) = 180;
	step_cmds->at(20).at(13) = 111;
	step_cmds->at(20).at(14) = 0;
	step_cmds->at(20).at(15) = 2000; //delay
#endif //run_agility
#ifndef run_agility
	//move all steppers to mid
	for (unsigned int i = 0; i < 12; i++) {
		step_cmds->at(0).at(i) = 500;
	}
	step_cmds->at(0).at(12) = 64;
	step_cmds->at(0).at(13) = 137;
	step_cmds->at(0).at(14) = 1;
	step_cmds->at(0).at(15) = 5000; //delay

	//seperate rods
	step_cmds->at(1).at(0) = 789;
	step_cmds->at(1).at(1) = 789;
	step_cmds->at(1).at(2) = 789;
	step_cmds->at(1).at(3) = 789;
	step_cmds->at(1).at(4) = 789;
	step_cmds->at(1).at(5) = 789;
	step_cmds->at(1).at(6) = 338;
	step_cmds->at(1).at(7) = 338;
	step_cmds->at(1).at(8) = 338;
	step_cmds->at(1).at(9) = 338;
	step_cmds->at(1).at(10) = 338;
	step_cmds->at(1).at(11) = 338;
	step_cmds->at(1).at(12) = 180;
	step_cmds->at(1).at(13) = 64;
	step_cmds->at(1).at(14) = 1;
	step_cmds->at(1).at(15) = 2000; //delay

	//bend
	step_cmds->at(2).at(0) = 553;
	step_cmds->at(2).at(1) = 753;
	step_cmds->at(2).at(2) = 1270;
	step_cmds->at(2).at(3) = 1159;
	step_cmds->at(2).at(4) = 310;
	step_cmds->at(2).at(5) = 221;
	step_cmds->at(2).at(6) = 195;
	step_cmds->at(2).at(7) = 259;
	step_cmds->at(2).at(8) = 609;
	step_cmds->at(2).at(9) = 639;
	step_cmds->at(2).at(10) = 294;
	step_cmds->at(2).at(11) = 200;
	step_cmds->at(2).at(12) = 128;
	step_cmds->at(2).at(13) = 137;
	step_cmds->at(2).at(14) = 1;
	step_cmds->at(2).at(15) = 5000; //delay

	//close claw
	step_cmds->at(3).at(0) = 553;
	step_cmds->at(3).at(1) = 753;
	step_cmds->at(3).at(2) = 1270;
	step_cmds->at(3).at(3) = 1159;
	step_cmds->at(3).at(4) = 310;
	step_cmds->at(3).at(5) = 221;
	step_cmds->at(3).at(6) = 195;
	step_cmds->at(3).at(7) = 259;
	step_cmds->at(3).at(8) = 609;
	step_cmds->at(3).at(9) = 639;
	step_cmds->at(3).at(10) = 294;
	step_cmds->at(3).at(11) = 200;
	step_cmds->at(3).at(12) = 128;
	step_cmds->at(3).at(13) = 137;
	step_cmds->at(3).at(14) = 0;
	step_cmds->at(3).at(15) = 5000; //delay

	//lift
	step_cmds->at(4).at(0) = 789;
	step_cmds->at(4).at(1) = 789;
	step_cmds->at(4).at(2) = 789;
	step_cmds->at(4).at(3) = 789;
	step_cmds->at(4).at(4) = 789;
	step_cmds->at(4).at(5) = 789;
	step_cmds->at(4).at(6) = 338;
	step_cmds->at(4).at(7) = 338;
	step_cmds->at(4).at(8) = 338;
	step_cmds->at(4).at(9) = 338;
	step_cmds->at(4).at(10) = 338;
	step_cmds->at(4).at(11) = 338;
	step_cmds->at(4).at(12) = 128;
	step_cmds->at(4).at(13) = 137;
	step_cmds->at(4).at(14) = 0;
	step_cmds->at(4).at(15) = 20; //delay

	//lift
	step_cmds->at(5).at(0) = 789;
	step_cmds->at(5).at(1) = 789;
	step_cmds->at(5).at(2) = 789;
	step_cmds->at(5).at(3) = 789;
	step_cmds->at(5).at(4) = 789;
	step_cmds->at(5).at(5) = 789;
	step_cmds->at(5).at(6) = 338;
	step_cmds->at(5).at(7) = 338;
	step_cmds->at(5).at(8) = 338;
	step_cmds->at(5).at(9) = 338;
	step_cmds->at(5).at(10) = 338;
	step_cmds->at(5).at(11) = 338;
	step_cmds->at(5).at(12) = 0;
	step_cmds->at(5).at(13) = 180;
	step_cmds->at(5).at(14) = 0;
	step_cmds->at(5).at(15) = 20000; //delay

#endif

	/*
	 *
	 //seperate rods and rotate

	 */

	/*

	 //speed test

	 //move all steppers to top
	 for (unsigned int i = 0; i < 12; i++) {
	 step_cmds->at(0).at(i) = 2000;
	 }
	 step_cmds->at(0).at(12) = 0;
	 step_cmds->at(0).at(13) = 0;
	 step_cmds->at(0).at(14) = 1;
	 step_cmds->at(0).at(15) = 10000; //10 second delay


	 //move all steppers to bottom
	 for (unsigned int i = 0; i < 12; i++) {
	 step_cmds->at(1).at(i) = 0;
	 }
	 step_cmds->at(1).at(12) = 0;
	 step_cmds->at(1).at(13) = 0;
	 step_cmds->at(1).at(14) = 1;
	 step_cmds->at(1).at(15) = 10000; //10 second delay
	 */
	//precision test
	/*
	 //move all steppers to mid
	 for (unsigned int i = 0; i < 12; i++) {
	 step_cmds->at(0).at(i) = 500;
	 }
	 step_cmds->at(0).at(12) = 180;
	 step_cmds->at(0).at(13) = 111;
	 step_cmds->at(0).at(14) = 0;
	 step_cmds->at(0).at(15) = 7000; //delay


	 step_cmds->at(1).at(0) = 774;
	 step_cmds->at(1).at(1) = 775;
	 step_cmds->at(1).at(2) = 555;
	 step_cmds->at(1).at(3) = 505;
	 step_cmds->at(1).at(4) = 503;
	 step_cmds->at(1).at(5) = 553;
	 step_cmds->at(1).at(6) = 792;
	 step_cmds->at(1).at(7) = 841;
	 step_cmds->at(1).at(8) = 693;
	 step_cmds->at(1).at(9) = 602;
	 step_cmds->at(1).at(10) = 348;
	 step_cmds->at(1).at(11) = 390;
	 step_cmds->at(1).at(12) = 180;
	 step_cmds->at(1).at(13) = 132;
	 step_cmds->at(1).at(14) = 0;
	 step_cmds->at(1).at(15) = 7000; //delay

	 //move all steppers to mid
	 for (unsigned int i = 0; i < 12; i++) {
	 step_cmds->at(2).at(i) = 500;
	 }
	 step_cmds->at(2).at(12) = 180;
	 step_cmds->at(2).at(13) = 111;
	 step_cmds->at(2).at(14) = 0;
	 step_cmds->at(2).at(15) = 7000; //delay

	 step_cmds->at(3).at(0) = 774;
	 step_cmds->at(3).at(1) = 775;
	 step_cmds->at(3).at(2) = 555;
	 step_cmds->at(3).at(3) = 505;
	 step_cmds->at(3).at(4) = 503;
	 step_cmds->at(3).at(5) = 553;
	 step_cmds->at(3).at(6) = 792;
	 step_cmds->at(3).at(7) = 841;
	 step_cmds->at(3).at(8) = 693;
	 step_cmds->at(3).at(9) = 602;
	 step_cmds->at(3).at(10) = 348;
	 step_cmds->at(3).at(11) = 390;
	 step_cmds->at(3).at(12) = 180;
	 step_cmds->at(3).at(13) = 132;
	 step_cmds->at(3).at(14) = 0;
	 step_cmds->at(3).at(15) = 7000; //delay
	 */

	/* rotate over 180 degree workspace
	 //bend to one side
	 step_cmds->at(2).at(0) = 666;
	 step_cmds->at(2).at(1) = 654;
	 step_cmds->at(2).at(2) = 646;
	 step_cmds->at(2).at(3) = 685;
	 step_cmds->at(2).at(4) = 916;
	 step_cmds->at(2).at(5) = 889;
	 step_cmds->at(2).at(6) = 696;
	 step_cmds->at(2).at(7) = 524;
	 step_cmds->at(2).at(8) = 133;
	 step_cmds->at(2).at(9) = 179;
	 step_cmds->at(2).at(10) = 503;
	 step_cmds->at(2).at(11) = 629;
	 step_cmds->at(2).at(12) = 90;
	 step_cmds->at(2).at(13) = 90;
	 step_cmds->at(2).at(14) = 0;
	 step_cmds->at(2).at(15) = 10;//delay;

	 step_cmds->at(3).at(0) = 706;
	 step_cmds->at(3).at(1) = 680;
	 step_cmds->at(3).at(2) = 754;
	 step_cmds->at(3).at(3) = 796;
	 step_cmds->at(3).at(4) = 909;
	 step_cmds->at(3).at(5) = 893;
	 step_cmds->at(3).at(6) = 316;
	 step_cmds->at(3).at(7) = 218;
	 step_cmds->at(3).at(8) = 137;
	 step_cmds->at(3).at(9) = 218;
	 step_cmds->at(3).at(10) = 653;
	 step_cmds->at(3).at(11) = 671;
	 step_cmds->at(3).at(12) = 90;
	 step_cmds->at(3).at(13) = 90;
	 step_cmds->at(3).at(14) = 0;
	 step_cmds->at(3).at(15) = 10; //delay

	 step_cmds->at(4).at(0) = 428;
	 step_cmds->at(4).at(1) = 583;
	 step_cmds->at(4).at(2) = 864;
	 step_cmds->at(4).at(3) = 860;
	 step_cmds->at(4).at(4) = 747;
	 step_cmds->at(4).at(5) = 597;
	 step_cmds->at(4).at(6) = 150;
	 step_cmds->at(4).at(7) = 100;
	 step_cmds->at(4).at(8) = 242;
	 step_cmds->at(4).at(9) = 323;
	 step_cmds->at(4).at(10) = 540;
	 step_cmds->at(4).at(11) = 508;
	 step_cmds->at(4).at(12) = 90;
	 step_cmds->at(4).at(13) = 90;
	 step_cmds->at(4).at(14) = 0;
	 step_cmds->at(4).at(15) = 10;//delay

	 step_cmds->at(5).at(0) = 584;
	 step_cmds->at(5).at(1) = 667;
	 step_cmds->at(5).at(2) = 993;
	 step_cmds->at(5).at(3) = 983;
	 step_cmds->at(5).at(4) = 614;
	 step_cmds->at(5).at(5) = 540;
	 step_cmds->at(5).at(6) = 154;
	 step_cmds->at(5).at(7) = 120;
	 step_cmds->at(5).at(8) = 704;
	 step_cmds->at(5).at(9) = 667;
	 step_cmds->at(5).at(10) = 489;
	 step_cmds->at(5).at(11) = 559;
	 step_cmds->at(5).at(12) = 90;
	 step_cmds->at(5).at(13) = 90;
	 step_cmds->at(5).at(14) = 0;
	 step_cmds->at(5).at(15) = 10; //delay

	 step_cmds->at(6).at(0) = 548;
	 step_cmds->at(6).at(1) = 667;
	 step_cmds->at(6).at(2) = 953;
	 step_cmds->at(6).at(3) = 877;
	 step_cmds->at(6).at(4) = 353;
	 step_cmds->at(6).at(5) = 310;
	 step_cmds->at(6).at(6) = 291;
	 step_cmds->at(6).at(7) = 374;
	 step_cmds->at(6).at(8) = 712;
	 step_cmds->at(6).at(9) = 705;
	 step_cmds->at(6).at(10) = 337;
	 step_cmds->at(6).at(11) = 261;
	 step_cmds->at(6).at(12) = 90;
	 step_cmds->at(6).at(13) = 90;
	 step_cmds->at(6).at(14) = 0;
	 step_cmds->at(6).at(15) = 10000;//10 second delay;
	 */

	//extrapolate between the two
	/*
	 int at = 2;
	 int numSteps = 10;
	 for(unsigned int j = 0; j < 10; j++) {
	 for (unsigned int k = 0; k < 16; k++) {
	 //i * ((goal - start) / numSteps) + start
	 step_cmds->at(at+j).at(k) = j*((goal - step_cmds->at(2).at(k))/numSteps) + step_cmds->at(2).at(k);
	 }
	 }

	 */

	// create the ROS topics
	step_cmd_out = node.advertise < coma_serial::teleop_command > ("/serial_node/step_cmd", 1000);
	resp_in = node.subscribe < std_msgs::Char > ("/serial_node/resp", 100, &motion_demo::resp_cback, this);

	ROS_INFO("COMA Motion Demo Node Started");
}

void motion_demo::resp_cback(const std_msgs::Char::ConstPtr& resp) {
	if (resp->data == 'R') {
		response_received = true;
	}
}

void motion_demo::publish_cmd() {
	static unsigned int atElem = 0;
//	static int counts[12];
	if (response_received) { //only publish if the board is ready for another command
//		response_received = false;
//		//publish a constant stream of steps
//		for (unsigned int i; i < 12; i++) {
//			counts[i] += 20;
//			cmd.stepper_counts[i] = counts[i];
//		}
//		step_cmd_out.publish(cmd);

		response_received = false;
		for (unsigned int leg; leg < 12; leg++) {
			cmd.stepper_counts[leg] = step_cmds->at(atElem).at(leg);
		}
		cmd.wrist_flex = step_cmds->at(atElem).at(12);
		cmd.wrist_rot = step_cmds->at(atElem).at(13);
		cmd.gripper_open = step_cmds->at(atElem).at(14);
		step_cmd_out.publish(cmd);
		ros::Duration(((float) step_cmds->at(atElem).at(15)) / 1000).sleep();
		if (++atElem >= numCmds) {
			atElem = 0;
		}
	}
}

int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "motion_demo");

	// initialize the demo object
	motion_demo demo;

	ros::Duration(3.0).sleep(); //short delay while everything initializes

	//home the manipulator
	demo.cmd.home = true;
	demo.cmd.wrist_flex = 0;
	demo.cmd.wrist_rot = 0;
	demo.cmd.gripper_open = 1;
	for (unsigned int leg; leg < 12; leg++) {
		demo.cmd.stepper_counts[leg] = 0;
	}
	demo.response_received = false;
	demo.step_cmd_out.publish(demo.cmd);
	demo.cmd.home = false;

	ros::Duration(20.0).sleep(); //short delay after sending homing command

	ros::Rate loop_rate(500);  //rate at which to publish arm velocity commands
	while (ros::ok()) {
		demo.publish_cmd();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
