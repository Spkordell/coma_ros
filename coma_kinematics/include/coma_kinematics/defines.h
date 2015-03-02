#ifndef DEFINES_H_
#define DEFINES_H_

//#define INCLUDE_WRIST 		//if defined, model will include a 2DOF wrist
//#define USE_MULTITHREADING 	//if defined, model will perform rod integrations in multiple threads
//#define USE_MATRIX_LOG 	//if defined, alignment residuals will be calculated using matrix logarithms instead of rodrigues' formula

#ifdef INCLUDE_WRIST
#define GS 7*12+2
#define DIST_TO_FLEX_JOINT 0.05
#define DIST_TO_GRIPPER 0.05
#else
#define GS 7*12 //define the guess size
#endif

#define SGS 7 //define the single guess size
#define INTEGRATION_STEPS 20  //note: search for 20 in cosserat_rod.h, that needs changed to if you change this
#define MIN_LEG_LENGTH_TOP 0.12
#define MIN_LEG_LENGTH_BOTTOM 0.22



#endif //DEFINES_H
