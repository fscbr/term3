#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <queue>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class PathPlanner {
public:

	typedef enum {
		CAR_X,
		CAR_Y,
		CAR_YAW,
		CAR_S,
		CAR_D,
		CAR_S_DOT,
		CAR_D_DOT,
		CAR_S_2DOT,
		CAR_D_2DOT
	} POSITIONDATA;

	  /**
	  * Constructor
	  */
	PathPlanner();

	  /**
	  * Destructor
	  */
	  virtual ~PathPlanner();

	  /**
	   * set parameter
	   */
	  void configure(vector<double> &road_data);

	  /*
	   * calculate the dynamoic parameters of the car
	   */
	  void calcCarParameter(vector<double> &car_parameter,
	  			const vector<vector<double> > &previous_path,
	  			const vector<double> &start_pos);

	  /**
	   * create a path using past, factual and future way points
	   */
	  void createPath(vector<vector<double>> &path,
				Eigen::VectorXd &conv_state,
				const vector<vector<double> > &map_waypoints,
				const vector<vector<double> > &previous_path,
				const vector<double> &start_pos,
				const double planned_lane);

	  /**
	   * merge odl and new path and convert to map coordinates
	   */
	  void mergePath(vector<vector<double>> &merged_path,
				const vector<vector<double>> &previous_path,
				const vector<vector<double>> &path,
				const Eigen::VectorXd &conv_state,
				const vector<double> car_parameter,
				const double planned_speed,
				const double planned_lane);

	  vector<double> predictAndAdjustSeries(vector< double> coeff,
			  double T,
			  int steps,
			  vector<double> limits,
			  bool adjust);

	  vector<double> JMT(const vector< double> start,
			  const vector <double> end, double T);

private:
	  /**
	   * find the next waypoint ahead using s position
	   */
	  int getNextWaypoint(vector<double> &waypoints, double s);

	  /**
	   * Transform from Frenet s,d coordinates to Cartesian x,y
	   */
	  vector<double> getXY(const double s,const double d,const vector<double> &maps_s,const vector<double> &maps_x,const vector<double> &maps_y);

	  //horizont to plan the path
	  double horizent_x = 42;
	  //steps in a second the path is planned
	  int steps = 50;
	  double map_point_distance = 30.0;

	  queue<double> speed_queue;

	  queue<double> d_queue;

	  double current_speed=0;

	  double current_lane=1;

	  double target_speed=22;

	  double max_acceleration=9.5;

	  double max_yerk=9.5;



};



#endif /*PATHPLANNER_H*/

