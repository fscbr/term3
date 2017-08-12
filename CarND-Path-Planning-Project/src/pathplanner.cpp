#include "pathplanner.h"
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "spline.h"

#include "helper_functions.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//#define DUMP_DATA

PathPlanner::PathPlanner()
{
}

PathPlanner::~PathPlanner()
{
}


void PathPlanner::configure(vector<double> &road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    max_acceleration = road_data[2];
    max_yerk = road_data[3];
//    T = road_data[6];
}

vector<double> PathPlanner::predictAndAdjustSeries(vector< double> coeff, double T, int steps, vector<double> limits, bool adjust)
{
    int time_faktor = 1/(double)T * steps;
    double delta_t = 1/(double)steps * T;

    int counter = 1;

    vector<double> calcpoints = vector<double>();
    //iterate as long constraints are violated
    while( true)
    {
        cout << "counter:" << counter++ << " steps:" << steps << " delta_t:" << delta_t << endl;
        double max_deriv1 = 0;
        double max_deriv2 = 0;
        double sum_deriv3 = 0;
        double max_deriv3 = 0;
        double last_cp = 0;
        double last_deriv1 = 0;
        double last_deriv2 = 0;
    	calcpoints.clear();
#ifdef DUMP_DATA
        cout << "counter;t;JMT;"<<endl;
#endif
        for(int i =1;i <= steps;i++)
        {
            double total = 0;
        	double t = i * delta_t;
            for(int j = 0;j < coeff.size();j++)
            {
            	total += coeff[j] * pow( t,j);
            }
#ifdef DUMP_DATA
            cout << counter << ";" << i << ";" << total << ";"<<endl;
#endif
            calcpoints.push_back(total);
            if(i > 1)
            {
            	double deriv1 = (total - last_cp)*time_faktor;
            	if(abs(max_deriv1) < abs(deriv1))
            		max_deriv1 = deriv1;
                if(i > 2)
                {
                	double deriv2 = deriv1 - last_deriv1;
                	if(abs(max_deriv2) < abs(deriv2))
                		max_deriv2 = deriv2;

                    if(i > 3)
                    {
                    	sum_deriv3 += deriv2 - last_deriv2;
                    }
                    last_deriv2 = deriv2;
                }
            	last_deriv1 = deriv1;
            }
            last_cp = total;
        }
        max_deriv3 = sum_deriv3 / (steps - 3);

        if(!adjust)
        	break;

        double deriv1_faktor = abs(max_deriv1/limits[0]);
        double deriv2_faktor = pow(abs(max_deriv2/limits[1]),0.5);
        double deriv3_faktor = pow(abs(max_deriv3/ limits[2]),0.333);

        double faktor = max(max(deriv1_faktor,deriv2_faktor),deriv3_faktor);
        int adapted_steps = floor(steps*(faktor*1.05));

        if(abs(max_deriv1) > limits[0])
        {
        	cout << "WARN----------------- LIMIT deriv1:" << max_deriv1 << " faktor:"<< deriv1_faktor << "----------------" << endl;
        }
        if(abs(max_deriv2) > limits[1])
        {
        	cout << "WARN----------------- LIMIT deriv2:" << max_deriv2 << " faktor:"<< deriv2_faktor << "----------------" << endl;
        }
        if(abs(max_deriv3) > limits[2]) {
        	cout << "WARN----------------- LIMIT deriv3:" << max_deriv3 << " faktor:"<< deriv3_faktor << "----------------" << endl;
        }
//        cout << "counter:" << counter << " max deriv1:" << max_deriv1 << " max deriv2:" << max_deriv2 << " max deriv3:" <<  max_deriv3 << " faktor:"<< faktor << endl;

        if(faktor <= 1.0)
        {
        	break;
        }
        else {
//            cout << "adapted_steps:" << adapted_steps << endl;
           delta_t /= faktor*1.05;
            steps = adapted_steps;
        }

    }
    return calcpoints;

}

vector<double> PathPlanner::JMT(const vector< double> start,
		const vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    MatrixXd m = MatrixXd(3,3);
    m << T*T*T, T*T*T*T, T*T*T*T*T,
    	 3*T*T, 4*T*T*T, 5*T*T*T*T,
    	   6*T,  12*T*T,  20*T*T*T;

    VectorXd ds = VectorXd(3);
    ds << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),end[1] - (start[1] + start[2]*T),end[2] - start[2];

    VectorXd sf = m.colPivHouseholderQr().solve(ds);
//    cout << "JMT coeff:"<< sf.transpose() << endl;

    std::vector<double> coeff = vector<double>(6);
    coeff[0] = start[0];
    coeff[1] = start[1];
    coeff[2] = 0.5*start[2];
    coeff[3] = sf[0];
    coeff[4] = sf[1];
    coeff[5] = sf[2];

    return coeff;
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::getXY(const double s,const  double d,const  vector<double> &maps_s,const  vector<double> &maps_x,const  vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}


/**
 * find the next waypoint ahead using s position
 */
int PathPlanner::getNextWaypoint(vector<double> &waypoints, double s){

	for( int i = 0; i < waypoints.size();i++)
    {
    	if(waypoints[i] > s)
    		return i;
    }
    return 0;
}

void PathPlanner::calcCarParameter(vector<double> &car_parameter,
			const vector<vector<double> > &previous_path,
			const vector<double> &start_pos)
{

	double ref_x = start_pos[CAR_X];
	double ref_y = start_pos[CAR_Y];
	double ref_yaw = start_pos[CAR_YAW];
	double p_car_x =ref_x;
	double p_car_y =ref_y;
	double p_car_x2 =ref_x;
	double p_car_y2 =ref_y;

	//add past and actual waypoints
	size_t previous_path_size = previous_path[CAR_X].size();
	if(previous_path[0].size() < 2)
	{
		p_car_x  = ref_x  - cos(ref_yaw);
		p_car_y  = ref_y  - sin(ref_yaw);

	} else {
		if(previous_path[0].size() >=3)
		{
			p_car_x2 = previous_path[CAR_X][previous_path_size-3];
			p_car_y2 = previous_path[CAR_Y][previous_path_size-3];
		}
		p_car_x = previous_path[CAR_X][previous_path_size-2];
		p_car_y = previous_path[CAR_Y][previous_path_size-2];

		ref_x = previous_path[CAR_X][previous_path_size-1];
		ref_y = previous_path[CAR_Y][previous_path_size-1];
		ref_yaw = atan2((ref_y-p_car_y),(ref_x-p_car_x));

	}

	car_parameter[CAR_X] = start_pos[CAR_X];
	car_parameter[CAR_Y] = start_pos[CAR_Y];
	car_parameter[CAR_YAW] = ref_yaw;
	car_parameter[CAR_S] = start_pos[CAR_S];
	car_parameter[CAR_D] = start_pos[CAR_D];
	car_parameter[CAR_S_DOT] = ((previous_path[0].size() < 2)?0:ref_x - p_car_x)*steps;
	car_parameter[CAR_D_DOT] = ((previous_path[0].size() < 2)?0:ref_y - p_car_y)*steps;
	car_parameter[CAR_S_2DOT] = ((previous_path[0].size() < 3)?0:ref_x - p_car_x)*steps - (p_car_x - p_car_x2)*steps;
	car_parameter[CAR_S_2DOT] =  ((previous_path[0].size() < 2)?0:ref_y - p_car_y)*steps - (p_car_y - p_car_y2)*steps;

    cout << "start s " << car_parameter[CAR_S] << " s. " << car_parameter[CAR_S_DOT] << " s.. " << car_parameter[CAR_S_2DOT]   << endl;
    cout << "start d " << car_parameter[CAR_D] << " d. " << car_parameter[CAR_D_DOT] << " d.. " << car_parameter[CAR_D_2DOT]   << endl;


}
void PathPlanner::createPath(vector<vector<double>> &path,
			Eigen::VectorXd &conv_state,
			const vector<vector<double> > &map_waypoints,
			const vector<vector<double> > &previous_path,
			const vector<double> &start_pos,
			const double lane)
{
	path = {vector<double>(),vector<double>()};


	double ref_x = start_pos[CAR_X];
	double ref_y = start_pos[CAR_Y];
	double ref_yaw = start_pos[CAR_YAW];

	//add past and actual waypoints
	size_t previous_path_size = previous_path[CAR_X].size();
	if(previous_path[0].size() < 2)
	{
		double p_car_x  = ref_x  - cos(ref_yaw);
		double p_car_y  = ref_y  - sin(ref_yaw);

        path[0].push_back(p_car_x);
		path[1].push_back(p_car_y);

        path[0].push_back(ref_x);
		path[1].push_back(ref_y);
	} else {
		double p_car_x = previous_path[CAR_X][previous_path_size-2];
		double p_car_y = previous_path[CAR_Y][previous_path_size-2];

		ref_x = previous_path[CAR_X][previous_path_size-1];
		ref_y = previous_path[CAR_Y][previous_path_size-1];
		ref_yaw = atan2((ref_y-p_car_y),(ref_x-p_car_x));

        path[0].push_back(p_car_x);
		path[1].push_back(p_car_y);

        path[0].push_back(ref_x);
		path[1].push_back(ref_y);
	}
	//add future waypoints
	int mp_to_add = 3;
	double next_s = start_pos[CAR_S];
#ifdef DUMP_DATA
	cout << "wpcar;t;x;y;" << endl;
#endif

	for(int i = 0;i < mp_to_add;i++)
	{
		next_s += map_point_distance*2;

		vector<double> xy = getXY(next_s, 2 + 4*lane, map_waypoints[0], map_waypoints[1], map_waypoints[2]);

		path[CAR_X].push_back(xy[CAR_X]);
		path[CAR_Y].push_back(xy[CAR_Y]);

#ifdef DUMP_DATA
		cout << "wpcar;" << i  << ";" << xy[CAR_X] << ";" << xy[CAR_Y] << ";" <<";"<< next_s<< endl;
#endif
	}
	//convert to car_pos
	Eigen::VectorXd map_pos(3);
	Eigen::VectorXd  car_pos(3);

	conv_state << ref_x, ref_y, ref_yaw;

#ifdef DUMP_DATA
	cout << "wpmap;t;x;y;" << endl;
#endif
	for(int i = 0;i < path[CAR_X].size();i++)
	{
		map_pos <<  path[CAR_X][i], path[CAR_Y][i], 0;
		car_pos = convertMapToCar(conv_state, map_pos);

		path[CAR_X][i] = car_pos[CAR_X];
		path[CAR_Y][i] = car_pos[CAR_Y];
#ifdef DUMP_DATA
		cout << "wpmap;" << i  << ";" << path[CAR_X][i] << ";" << path[CAR_Y][i] << ";" << endl;
#endif
	}
}


void PathPlanner::mergePath(vector<vector<double>> &merged_path,
		const vector<vector<double>> &previous_path,
		const vector<vector<double>> &path,
		const Eigen::VectorXd &conv_state,
		const vector<double> car_parameter,
		const double planned_speed,
		const double planned_lane)
{

    tk::spline spline_car_pos_y;
	spline_car_pos_y.set_points(path[CAR_X], path[CAR_Y]);

    int merged_index = 0;
    int path_size = previous_path[CAR_X].size();
#ifdef DUMP_DATA
	cout << "mergepath;t;x;y;" << endl;
#endif
    for(int i = 0; i < path_size; i++)
    {
    	merged_path[CAR_X].push_back(previous_path[CAR_X][i]);
    	merged_path[CAR_Y].push_back(previous_path[CAR_Y][i]);
#ifdef DUMP_DATA
		cout << "mergepath_old;" << merged_index++  << ";" << merged_path[CAR_X][i] << ";" << merged_path[CAR_Y][i] << ";" << endl;
#endif
    }

	//continue to use the old path lst in a lane change situation
/*	if(current_lane != planned_lane && path_size > 10)
		current_lane = planned_lane;*/

	double temp_speed = current_speed;

	if(abs(planned_speed - current_speed) > target_speed / steps && speed_queue.size()== 0)
	{

		vector<double >start = {current_speed, 0 ,0};
		vector<double >end = {planned_speed, 0 ,0};
		vector<double> coeff = JMT(start,end, 1);

		vector<double> jmt = predictAndAdjustSeries(coeff, 1,75, {20,10,10}, false);

		//fill a queue
		for(auto v:jmt) {
			speed_queue.push(v);
		}
	}

    double horizent_y = spline_car_pos_y(horizent_x);

    double horizent_dist = sqrt(horizent_y*horizent_y+horizent_x*horizent_x);

#ifdef DUMP_DATA
    cout << "ref:" << conv_state.transpose() << endl;
	cout << "horizent_x:" << horizent_x << " horizent_y:" << horizent_y << endl;
#endif

	//convert to map_pos
	Eigen::VectorXd map_pos(3);
	Eigen::VectorXd  car_pos(3);

    double add_x = 0;
	double last_x = 0;
	double last_y = 0;
	double last_v_x = car_parameter[CAR_S_DOT];
	double last_v_y = 0;
	double next_x = 0.0;
	double next_y = 0.0;
	
    for(int i = 0; i < steps - path_size; i++)
    {

    	if(speed_queue.size() > 0)
    	{
    		temp_speed = speed_queue.front();
    		speed_queue.pop();

    		//clear queue
        	if(speed_queue.size() == 0 || abs(temp_speed - planned_speed) < target_speed/steps)
        	{
    			current_speed = temp_speed;
    			cout <<"speed queue cleaned at:" << speed_queue.size()<< endl;
    			speed_queue = queue<double>();
        	}
    	}
    	double dx = temp_speed / steps;

//    	next_x = add_x + horizent_x / planned_steps;
    	next_x = add_x + dx;
    	next_y = spline_car_pos_y(next_x);
    	add_x = next_x;


    	//calc v and a
    	double v_x = (next_x - last_x) * steps;
    	double v_y = (next_y - last_y) * steps;

    	double a_x = v_x - last_v_x;
    	double a_y = v_y - last_v_y;

    	double acc = sqrt(a_x*a_x +a_y*a_y)*50;

    	last_x = next_x;
    	last_y = next_y;
    	last_v_x = v_x;
    	last_v_y = v_y;

//    	cout << "next_x:" << next_x << " next_y:" << next_y << endl;

		car_pos <<  next_x, next_y, 0;
		map_pos = convertCarToMap(conv_state, car_pos);

		merged_path[CAR_X].push_back(map_pos[CAR_X]);
		merged_path[CAR_Y].push_back(map_pos[CAR_Y]);
#ifdef DUMP_DATA
		cout << "mergepath_new;" << merged_index++  << ";" << map_pos[CAR_X]<< ";" << map_pos[CAR_Y] << ";" << endl;
#endif
	}

}



