/*
 * CarStatus.h
 *
 *  Created on: Aug 5, 2017
 *      Author: frank
 */

#ifndef CARSTATE_H_
#define CARSTATE_H_

using namespace std;

class CarStatus {
	public:
		typedef enum { KL,           // 0 keep lane
	               LCL,          // 1 lane change left
	               LCR,          // 2 lane change right
	               PLCL,         // 3 prepare lane change left
	               PLCR } states;// 4 prepare lane change right

	  int lane;

	  double s;

	  double d;

	  double v;

	  double a;

	  states state;

};



#endif /* CARSTATE_H_ */
