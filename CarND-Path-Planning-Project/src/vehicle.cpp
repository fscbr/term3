#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <numeric>
#include "car_state.h"

using namespace std;

//#define DEBUG


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {
    max_acceleration = -1;
    goal_lane = 1;
    start.state = CarStatus::states::KL;
    end.state = CarStatus::states::KL;
	start.lane = 1;
	end.lane = 1;
	T = 1;
}

Vehicle::~Vehicle() {}


void Vehicle::configure(vector<double> &road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    max_acceleration = road_data[2];
    max_yerk = road_data[3];
    preferred_buffer_in_front = road_data[4];
    preferred_buffer_behind = road_data[5];
    T = road_data[6];
}

void Vehicle::updateDynamics(double s, double d, double v, double a) {

	start.d = d;
	start.lane = floor(d/4);
	start.s = s;
	start.v = v;
	start.a = a;

	start.state = end.state;
	start.lane = end.lane;

}

double Vehicle::getTargetVel(vector<double> &target)
{
	double vx = target[CAR_X_V];
	double vy = target[CAR_Y_V];

	return sqrt(vx*vx + vy * vy);//* 0.44704;
;
}

vector<CarStatus::states> Vehicle::getPossibleStates(CarStatus::states now_state)
{
	vector<CarStatus::states> possible_states = vector<CarStatus::states>();
    for(int i =0;i < 5;i++)
    {
        switch(i){
            case CarStatus::KL:
                break;
            case CarStatus::LCL:
                if( now_state != CarStatus::PLCL ||  start.lane <= 0)
                    continue;
                break;
            case CarStatus::LCR:
                if( now_state != CarStatus::PLCR || start.lane >= 2){
                    continue;
                }
                break;
            case CarStatus::PLCL:
                if(now_state != CarStatus::KL  || start.lane <= 0){
                    continue;
                }
                break;
            case CarStatus::PLCR:
                if(now_state != CarStatus::KL || start.lane >= 2){
                    continue;
                }
                break;
        }
    	possible_states.push_back((CarStatus::states)i);
    }
	return possible_states;
}

double Vehicle::costTargetSpeed(vector<double> &param)
{
	double speed = param[0];
	double cost = 0;
	//speed advantage
	cost += (speed > 0)? max(target_speed / speed - 1,1.0):1.0;
	return cost;
}

double Vehicle::costInFront(CarStatus::states state, vector<double> &param)
{
	double speed = param[0];
	double acc = param[1];
	double space = param[2];

	double cost = 0;
	//space deficit
	cost += (space == horizent)?0.0:(space > 0)? max(min(preferred_buffer_in_front / space -1,1.0),0.0):2.0;
	//no lange change when no buffer
	cost += (state != CarStatus::states::KL && acc < -2.0)?2:0;
	//speed advantage on other lane
#ifdef DEBUG
	cost += (state != CarStatus::states::KL && speed > 0 && start.v > 0)?max(start.v/speed -1,1.0):1;
#endif
	return cost;
}

double Vehicle::costBehind(CarStatus::states state, vector<double> &param)
{
	double space = param[3];
	double acc = param[1];

	double cost = 0;
	cost += (space == horizent)?0.0:(space < 0)?2.0:max(min(preferred_buffer_behind / space -1,1.0),0.0);
	//no lange change when no buffer
	cost += (state != CarStatus::states::KL && acc < -2.0)?2:0;
	return cost;
}

/*predictions vector of vector of
0 car's unique ID,
1 car's x position in map coordinates,
2 car's y position in map coordinates,
3 car's x velocity in m/s,
4 car's y velocity in m/s,
5 car's s position in frenet coordinates,
6 car's d position in frenet coordinates.*/
void Vehicle::updateState(vector < vector<double> > predictions)
{
    vector<CarStatus::states> possible_states = getPossibleStates(start.state);

#ifdef DEBUG
    cout << "state " << states_name[start.state] << " " << display() << endl;
#endif

    double min_cost = 999999;

	vector<double> va_l = {0,0};
  	if(start.lane > 0)
  		va_l = _max_accel_speed_for_lane(predictions, start.lane -1, start.s);

  	vector<double> va_kl= _max_accel_speed_for_lane(predictions, start.lane, start.s);

	vector<double> va_r = {0,0};
  	if(start.lane < 2)
  		va_r = _max_accel_speed_for_lane(predictions, start.lane + 1, start.s);

    for(auto &ps : possible_states)
    {
        double cost_a = (start.a> max_acceleration)?1:0;
        double cost_v = costTargetSpeed(va_kl);

        double cost_l = 0;//(goal_lane - start.lane )*(goal_lane - start.lane );
/*        if((start.state == CarStatus::KL && ps == CarStatus::PLCL)||(start.state == CarStatus::PLCL && ps == CarStatus::LCL))
        {
            cost_l = (goal_lane - start.lane + (start.lane < 3)?1:0)*(goal_lane - start.lane + (start.lane < 3)?1:0);
        }
        if((start.state == CarStatus::KL && ps == CarStatus::PLCR)||(start.state == CarStatus::PLCR && ps == CarStatus::LCR))
        {
            cost_l = (goal_lane - start.lane + (start.lane > 0)?-1:0)*(goal_lane - start.lane + (start.lane > 0)?-1:0);
        }*/

        double cost_tl = 0;
        if(((start.state == CarStatus::KL && ps == CarStatus::PLCL)||(start.state == CarStatus::PLCL && (ps == CarStatus::LCL || ps == CarStatus::PLCL)))&& start.lane > 0)
        {
        	cost_tl += costInFront(ps, va_l);
         	cost_tl += costBehind(ps, va_l);
#ifdef DEBUG
        	cout << "check L " << cost_tl << endl;
#endif
        }
        if(((start.state == CarStatus::KL && ps == CarStatus::PLCR)||(start.state == CarStatus::PLCR && (ps == CarStatus::LCR|| ps == CarStatus::PLCR)))&& start.lane < 2)
        {
        	cost_tl += costInFront(ps, va_r);
        	cost_tl += costBehind(ps, va_r);
#ifdef DEBUG
        	cout << "check R " << cost_tl << endl;
#endif
        }

        if((start.state == CarStatus::KL || start.state == CarStatus::PLCL || start.state == CarStatus::PLCR) && ps == CarStatus::KL)
        {
        	cost_tl += costInFront(ps, va_kl);
//        	cost_tl += costBehind(va_kl);
#ifdef DEBUG
        	cout << "check KL " << cost_tl << endl;
#endif
        }

        double cost = cost_a + cost_v + cost_l  + cost_tl;
        if(min_cost > cost)
        {
          min_cost = cost;
          end.state = ps;
        }
#ifdef DEBUG
        cout << "cost for " << states_name[ps] << " a:" << cost_a << " v:" <<  cost_v << " tl:" <<  cost_tl << " total:" << cost << " " << endl;
#endif
    }
#ifdef DEBUG
    cout << "new state " << states_name[end.state] << endl;
#endif
}

string Vehicle::display() {

	ostringstream oss;

	oss << "s:" << start.s << " d:" << start.d << " lane:" << start.lane << " speed:" << start.v << " a:" << start.a;

    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	end.s += start.v * dt * T;
	end.v += start.a * dt * T;
}

vector<double> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
	double s = start.s + start.v * t * T + start.a * t * t  * T  * T/ 2;
	double v = start.v + start.a * t * T;
    return {start.d, s, v, start.a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<double> check1 = state_at(at_time);
    vector<double> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= T);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t;
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realizeState(vector < vector<double> > predictions) {

	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    switch(end.state)
    {
    	case CarStatus::KL:
    		realize_keep_lane(predictions);
    		break;
    	case CarStatus::LCL:
    		realizeLaneChange(predictions, end.state);
    		break;
    	case CarStatus::LCR:
    		realizeLaneChange(predictions, end.state);
    		break;
    	case CarStatus::PLCL:
    		realizePrepLaneChange(predictions, end.state);
    		break;
    	case CarStatus::PLCR:
    		realizePrepLaneChange(predictions, end.state);
    		break;
    }
	vector<double> va= _max_accel_speed_for_lane(predictions, end.lane, start.s);
	end.a = va[1];
    end.a = max(min(end.a,max_acceleration),-max_acceleration);

	end.v = max(va[0],0.0);
	end.v = min(end.v, target_speed);

    end.s = start.s + end.v * T + end.a * T * T / 2;
    end.d = end.lane * 4 +2;// - d_correct;

    if(end.state != CarStatus::KL && end.v > (target_speed -2))
    {
    	double mod_v = sqrt(end.v * end.v - 16);
    	end.a = end.v - mod_v;
    	end.v = mod_v;
    }

    cout << "realized " << states_name[end.state] << " lane:" << end.lane << " s:" << end.s << " d:" << end.d<< " a:" << end.a << " v:" << end.v << endl;

}

void Vehicle::realize_constant_speed() {
	end.a = 0;
}


vector<double> Vehicle::_max_accel_speed_for_lane(vector<vector<double> > &predictions, double lane, double s) {

	double max_v = target_speed;
	double delta_v_til_target = (target_speed - start.v)/T;
	double max_acc = min(max_acceleration, delta_v_til_target);
	double available_room_in_front= horizent;
	double available_room_behind= horizent;

    vector<vector<double> >::iterator it = predictions.begin();
    vector<vector<double> > in_front;
    while(it != predictions.end())
    {

        vector<double> car = *it;


        if((getLane(car[CAR_D]) == lane) && (car[CAR_S] > s  &&  car[CAR_S] - s  < horizent))
        {
        	in_front.push_back(car);
        }
        it++;
    }

    if(in_front.size() > 0)
    {
    	double min_s = horizent;
    	vector<double> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][CAR_S] - s) < min_s)
    		{
    			min_s = (in_front[i][CAR_S]-s);
    			leading = in_front[i];
    		}
    	}

    	double next_pos = leading[CAR_S];

    	double my_next =  start.s  + start.v*T;
    	double separation_next = next_pos - my_next;

    	available_room_in_front = separation_next - preferred_buffer_in_front;

    	double next_speed = getTargetVel(leading);
    	double delta_v_til_next = (next_speed - start.v)/T;

#ifdef DEBUG
    	cout << "lane:" << lane << " car in front delta_s:" << separation_next << " speed:" << next_speed << " s in front:" << available_room_in_front << " max_v:" << max_v << " max_acc:" << max_acc << endl;
#endif

    	if (available_room_in_front < 5)
    	{
    		max_acc = min(delta_v_til_next/T,max_acc);
    		max_v = min(next_speed, max_v);
#ifdef DEBUG
        	cout << "lane:" << lane << " car in front delta_s:" << separation_next << " speed:" << next_speed << " s in front:" << available_room_in_front << " max_v:" << max_v << " max_acc:" << max_acc << endl;
#endif
    	}
    	if (available_room_in_front < 0)
    	{
    		max_acc = max(delta_v_til_next + available_room_in_front/T,-max_acceleration);
    		next_speed += max_acc;
    		next_speed = max(next_speed ,0.0);
    		delta_v_til_next = (next_speed - start.v)/T;

    		max_v = max(min(max_v, next_speed),0.0);

    		if(lane == start.lane)
#ifdef DEBUG
    			cout << " to close next_speed" << next_speed  << " delta_v_til_next " << delta_v_til_next  << " max_v:" << max_v << " max_acc:" << max_acc <<  endl;
#endif

        	if (lane != start.lane)
        	{
        		max_v = 0;
        		max_acc = -max_acceleration;
#ifdef DEBUG
    			cout << " to close to change lane max_v:" << max_v  << " max_acc:" << max_acc <<  endl;
#endif
        	}
    	}

#ifdef DEBUG
    	cout << "lane:" << lane << " car in front delta_s:" << separation_next << " speed:" << next_speed << " max_v:" << max_v << " max_acc:" << max_acc << " s in front:" << available_room_in_front << endl;
#endif
    }

    vector<vector<double> > at_behind;
    it = predictions.begin();
    while(it != predictions.end())
    {
        vector<double> car = *it;

        if((getLane(car[CAR_D]) == lane) && (car[CAR_S] < s  &&  s - car[CAR_S] < horizent))
        {
        	at_behind.push_back(car);
        }
        it++;
    }

    //check distance behind
    if(at_behind.size() > 0)
    {

#ifdef DEBUG
    	cout << "car behinds:" << at_behind.size() << endl;
#endif
    	double max_s = -1000;
    	vector<double> nearest_behind;
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][CAR_S]) > max_s)
    		{
    			max_s = at_behind[i][CAR_S];
    			nearest_behind = at_behind[i];
    		}
    	}
    	double target_vel = getTargetVel(nearest_behind);
    	double delta_v = start.v - target_vel;
    	double delta_s = start.s - nearest_behind[CAR_S] + delta_v;
    	available_room_behind = delta_s - preferred_buffer_behind - 7;//lenght of car?

#ifdef DEBUG
    	cout << "lane:" << lane << " car at behind delta_s:" << delta_s << " speed:" << target_vel << " max_acc:" << max_acc << " s behind:" << available_room_behind << endl;
#endif

    	if (available_room_behind < 0 && lane != start.lane)
    	{
    		max_v = 0;
    		max_acc = -max_acceleration;
#ifdef DEBUG
			cout << " to close to change lane max_v:" << max_v  << " max_acc:" << max_acc <<  endl;
#endif
    	}

    }
	max_acc = min(max_acc, max_acceleration);
	max_acc = max(max_acc, -max_acceleration);

	max_v = max(max_v, 0.0);

	if(max_acc >0)
		max_v = min(min(max_v, start.v + max_acc), target_speed);
	else
		max_v = min(max(max_v, start.v + max_acc), target_speed);

    vector<double>va ={max_v,max_acc,available_room_in_front, available_room_behind};

#ifdef DEBUG
	cout << "lane:" << lane << " max speed:" << max_v << " max a:" << max_acc << " s in front:" << available_room_in_front << " s behind:" << available_room_behind<< endl;
#endif
    return va;

}


void Vehicle::realize_keep_lane(vector < vector<double> > &predictions) {
	vector<double> va= _max_accel_speed_for_lane(predictions, start.lane, start.s);
	end.lane = start.lane;
	end.a = va[1];
	end.v = start.v + (end.a + start.a)/2 * T;
}

void Vehicle::realizeLaneChange(vector < vector<double> > &predictions, CarStatus::states state) {
	int delta = 0;
    if (state == CarStatus::states::LCL)
    {
    	delta = -1;
    }
    else if (state == CarStatus::states::LCR)
    {
    	delta = 1;
    } else {
    	cout << ("not allowed lane change") << endl;
    	return;
    }
	end.lane = start.lane + delta;
}

void Vehicle::realizePrepLaneChange(vector < vector<double> > &predictions, CarStatus::states state) {

	end.lane = start.lane;
}

void Vehicle::testAll()
{

	vector<double> config = {20,3,9, 50, 10, 15, 1};

	double count = 1.0;
	vector<double> slist = {0,50,70,91,109,130,200,300};

	for(auto s : slist)
	{
		vector<double> vlist = {10, 15, 20, 25};
		for(auto speed : vlist)
		{
			vector<double> dlist = {2,6,10};
			for(auto d : dlist)
			{
				vector < vector<double> > predictions;
				vector<double> car = {count++,0,0,speed,0,s,d};
				predictions.push_back(car);

				Vehicle vehicle = Vehicle();
				vehicle.configure(config);
				vehicle.updateDynamics(100,6,10,8);
				vehicle.updateState(predictions);
				vehicle.realizeState(predictions);

				cout << "test:" << count << " car speed:" << speed << " s:" << s << " d:" << d;
				cout << " result start:" << vehicle.getStateName(vehicle.start.state) << " s:" << vehicle.start.s << " v:" << vehicle.start.v << " a: " << vehicle.start.a;
				cout << " end state: " << vehicle.getStateName(vehicle.end.state) << " s:" << vehicle.end.s << " v:" << vehicle.end.v << " a: " << vehicle.end.a << endl;

			}
		}
	}

}

