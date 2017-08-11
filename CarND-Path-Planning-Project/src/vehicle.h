#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "car_state.h"

using namespace std;

class Vehicle {
public:


    vector<string> states_name = { "KL",           // 0 keep lane
            "LCL",          // 1 lane change left
            "LCR",          // 2 lane change right
            "PLCL",         // 3 prepare lane change left
            "PLCR" };

	typedef enum { OUTSIDE_LANE_LEFT,
	               LEFT_LANE,
	               MIDDLE_LANE,
	               RIGHT_LANE,
	OUTSIDE_LANE_RIGHT } lanes;

	typedef enum {
		CAR_ID,
		CAR_X,
		CAR_Y,
		CAR_X_V,
		CAR_Y_V,
		CAR_S,
		CAR_D
	} cardata;

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int T = 1;

  int preferred_buffer_in_front; // impacts "keep lane" behavior.

  int preferred_buffer_behind; // impacts "keep lane" behavior.

  int goal_lane = 1;

  CarStatus start = CarStatus();

  CarStatus end = CarStatus();

  double target_speed;

  double lanes_available;

  double max_acceleration;

  double max_yerk;

  double horizent = 200;

  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void updateDynamics(double s, double d, double v, double a);

  vector<CarStatus::states> getPossibleStates(CarStatus::states now_state);

  void updateState(vector <vector<double> > predictions);

  void configure(vector<double> &road_data);

  string display();

  void increment(int dt);

  vector<double> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  double costTargetSpeed(vector<double> &param);

  double costInFront(CarStatus::states state, vector<double> &param);

  double costBehind(CarStatus::states state, vector<double> &param);

  void realizeState(vector < vector<double> > predictions);

  void realize_constant_speed();

  vector<double> _max_accel_speed_for_lane(vector<vector<double> > &predictions, double lane, double s);

  void realize_keep_lane(vector< vector<double> > &predictions);

  void realizeLaneChange(vector< vector<double> > &predictions, CarStatus::states);

  void realizePrepLaneChange(vector< vector<double> > &predictions, CarStatus::states);

  vector<vector<int> > generate_predictions(int horizon);

  int getLane(double d){return floor(d / 4);};

  double getTargetVel(vector<double> &target);

  double getDeltaS(){return end.s - start.s;};

  double getDeltaD(){return end.lane * 4 + 2 - start.d;};

  std::string getStateName(CarStatus::states state){return states_name[state];};

  static void testAll();

  bool isLaneChange(){return (end.state == CarStatus::states::LCL || end.state == CarStatus::states::LCR);};

  bool isHighAcceleration(){cout << "high acc:" << (abs(end.a) >= (max_acceleration/2)) << endl;return abs(end.a) >= (max_acceleration/2);};

};

#endif
