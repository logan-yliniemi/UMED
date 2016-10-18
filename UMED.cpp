//
// Created by Logan Yliniemi on 10/17/16.
// Last Edit by Logan Yliniemi on 10/17/16.
//  Copyright © 2016 Logan Yliniemi. All rights reserved.
//

#include <iostream>

using namespace std;
#include <time.h>
#include <random>
#include <cassert>

/// uniform random between lower and upper bounds.
#define uniform_random(low,up) ((double)rand()/RAND_MAX*(up-low) + low)


///////////////////// %%%%%%%%%%%%%%%%%% BEGIN CLASS DECLARATIONS %%%%%%%%%%%%%%%%%% /////////////////////

class vehicle;
class waypoint;
class environment;
class Point_of_Interest;
class agent;
class policy;
class scoreboard;
class parameters;

class vehicle{
public:
    int id;
    double x;
    double y;
    double z;
    bool general_comm_link;
    vector<bool> to_agent_comm_link;
    
    void init();
    void start_based_on_policy(policy* pPol, int num_agents);
};

class waypoint{
public:
    double x;
    double y;
    double z;
    void create_surface(parameters* pPar);
    void create_random(parameters* pPar);
    void mutate(parameters* pPar,bool surf);
    void boundaries(parameters* pPar);
};

class agent{
public:
    vehicle V;
    vector<policy> policies;
    double local;
    double global;
    double true_difference;
    double limited_difference;
    vector<double> obs_distance; /// best observations of each POI
    
    /// functions to initialize.
    void init(parameters* pPar);
    void init_obs_distance(); /// TODO
};

class policy{
public:
    vector<waypoint> WP;
    void init(parameters* pPar);
};

class environment{
public:
    vector<Point_of_Interest> POIs;
    void init(parameters* pPar);
};

class Point_of_Interest{
public:
    double x;
    double y;
    double z;
    double val;
    void init(parameters* pPar);
};

class scoreboard{
public:
    vector<double> Global_Best;
    /// TODO: Class to track performance of the team over generations
    void init(); /// TODO
};

class parameters{
public:
    const int num_agents = 1;
    const int num_vehicles = num_agents; // 1 vehicle per agent
    const int num_POI = 5*num_agents;
    const int pop_size = 100;
    const int num_waypoints = 10;
    
    const double mutation_size = 5.0;
    
    const double max_x = 100;
    const double max_y = 100;
    const double max_z = 10;
    
    const double min_x = 0;
    const double min_y = 0;
    const double min_z = -100;
    
    const double min_poi_value = 1;
    const double max_poi_value = 100;
    
    void init();
};
///////////////////// %%%%%%%%%%%%%%%%%% END CLASS DECLARATIONS %%%%%%%%%%%%%%%%%% /////////////////////

///////////////////// %%%%%%%%%%%%%%%%%% BEGIN CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////

/////// BGN AGENT FUNCTIONS ///////
void agent::init(parameters* pPar){
    
    /// create a vector of policies
    for(int pop=0; pop<pPar->pop_size; pop++){
        policy X;
        X.init(pPar);
        policies.push_back(X);
    }
    
    /// create a vector of waypoints for each policy (taken care of in policy.init())
    
}
void agent::init_obs_distance(){
    /// TODO
}
/////// END AGENT FUNCTIONS ///////

/////// BGN Policy FUNCTIONS ///////
void policy::init(parameters* pPar){
    /// create num_waypoints quantity of waypoints.
    for(int w=0; w<pPar->num_waypoints; w++){
        waypoint single;
        if(w==0){
            single.create_surface(pPar);
        }
        else{
            single.create_random(pPar);
        }
        WP.push_back(single);
    }
    assert(WP.size() == pPar->num_waypoints);
}
/////// END Policy FUNCTIONS ///////

/////// BGN VEHICLE FUNCTIONS ///////
void vehicle::init(){
    /// each vehicle gets a unique ID
    static int index;
    id = index;
    index++;
}
void vehicle::start_based_on_policy(policy* pPol, int num_agents){
    x = pPol->WP.at(0).x;
    y = pPol->WP.at(0).y;
    z = pPol->WP.at(0).z;
    
    general_comm_link = false;
    
    for(int agent = 0; agent < num_agents; agent++){
        to_agent_comm_link.push_back(false);
    }

}
/////// END VEHICLE FUNCTIONS ///////

/////// BGN WAYPOINT FUNCTIONS ///////

void waypoint::create_surface(parameters* pPar){
    /// first waypoint is defined to be on the surface.
    x = uniform_random(pPar->min_x,pPar->max_x);
    y = uniform_random(pPar->min_y,pPar->max_y);
    z = 0.001;

}
void waypoint::create_random(parameters* pPar){
    x = uniform_random(pPar->min_x,pPar->max_x);
    y = uniform_random(pPar->min_y,pPar->max_y);
    z = uniform_random(pPar->min_z,pPar->max_z);
}
void waypoint::mutate(parameters* pPar,bool surf){
    x += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    y += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    if(surf==false){
    z += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    }
    boundaries(pPar);
}
void waypoint::boundaries(parameters* pPar){
    /// keeps mutated waypoints within boundaries of system.
    // upper bounds
    if(x > pPar->max_x){
        x = pPar->max_x;
    }
    if(y > pPar->max_y){
        y = pPar->max_y;
    }
    if(z > pPar->max_z){
        z = pPar->max_z;
    }
    // lower bounds
    if(x < pPar->min_x){
        x = pPar->min_x;
    }
    if(y < pPar->min_y){
        y = pPar -> min_y;
    }
    if(z < pPar->min_z){
        z = pPar -> min_z;
    }
}
/////// END WAYPOINT FUNCTIONS ///////

/////// BGN ENVIRONMENT FUNCTIONS ///////
void environment::init(parameters* pPar){
    /// initialize all POIs
    for(int poi=0; poi<pPar->num_POI; poi++){
        Point_of_Interest P;
        P.init(pPar);
        POIs.push_back(P);
    }
}
/////// END ENVIRONMENT FUNCTIONS ///////

/////// BGN POI FUNCTIONS ///////
void Point_of_Interest::init(parameters* pPar){
    x = uniform_random(pPar->min_x,pPar->max_x);
    y = uniform_random(pPar->min_y,pPar->max_y);
    z = uniform_random(pPar->min_z,pPar->max_z);
    /// guarantee that POI is below surface:
    while(z>0){
        /// keep choosing random values.
        z = uniform_random(pPar->min_z,pPar->max_z);
    }
    
    val = uniform_random(pPar->min_poi_value, pPar->max_poi_value);
}
/////// END POI FUNCTIONS ///////

/////// BGN SCOREBOARD FUNCTIONS ///////
void scoreboard::init(){
    /// TODO
}
/////// END SCOREBOARD FUNCTIONS ///////

/////// BGN PARAMETERS FUNCTIONS ///////
void parameters::init(){
    /// This space left intentionally blank.
    /// All parameters are set in the class definition.
}
/////// END PARAMETERS FUNCTIONS ///////

/////// BGN OTHER FUNCTIONS ///////

/////// END OTHER FUNCTIONS ///////



///////////////////// %%%%%%%%%%%%%%%%%% END CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////



int main() {
    srand((unsigned)time(NULL));
    cout << "Start of Program" << endl;
    
    parameters P;
    parameters* pPar = &P;
    
    vector<agent> Team;
    for(int a=0; a<pPar->num_agents; a++){
        agent AA;
        AA.init(pPar);
        Team.push_back(AA);
    }
    assert(Team.size() == pPar->num_agents);
    
    environment E;
    E.init(pPar);
    
    
    
    cout << "End of Program" << endl;
    return 0;
}
























