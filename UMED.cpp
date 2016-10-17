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

///////////////////// %%%%%%%%%%%%%%%%%% BEGIN CLASS DECLARATIONS %%%%%%%%%%%%%%%%%% /////////////////////

class vehicle;
class waypoint;
class environment;
class Point_of_Interest;
class agent;
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
    
    void init(); /// TODO
};

class waypoint{
public:
    double x;
    double y;
    double z;
    void create_random(parameters* pP); /// TODO
    void mutate(parameters* pP); /// TODO
    void boundaries(parameters* pP); /// TODO
};

class agent{
public:
    vehicle V;
    vector<waypoint> WP;
    double local;
    double global;
    double true_difference;
    double limited_difference;
    vector<double> obs_distance; /// best observations of each POI
    
    /// functions to initialize.
    void init(parameters* pP); /// TODO
    void init_obs_distance(); /// TODO
};

class environment{
public:
    vector<Point_of_Interest> POIs;
};

class Point_of_Interest{
public:
    double x;
    double y;
    double z;
    double val;
    void init(parameters* pP); /// TODO
};

class scoreboard{
public:
    vector<double> Global_Best;
    /// TODO: Class to track performance of the team over generations
    void init(); /// TODO
};

class parameters{
public:
    int num_agents = 1;
    int num_vehicles = num_agents; // 1 vehicle per agent
    int num_POI = 5*num_agents;
    int pop_size = 100;
    int num_waypoints = 10;
    
    double max_x = 100;
    double max_y = 100;
    double max_z = 10;
    
    double min_x = 0;
    double min_y = 0;
    double min_z = -100;
    
    void init();
};
///////////////////// %%%%%%%%%%%%%%%%%% END CLASS DECLARATIONS %%%%%%%%%%%%%%%%%% /////////////////////

///////////////////// %%%%%%%%%%%%%%%%%% BEGIN CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////

/////// BGN AGENT FUNCTIONS ///////
void agent::init(parameters* pP){
    /// TODO
}
void agent::init_obs_distance(){
    /// TODO
}
/////// END AGENT FUNCTIONS ///////

/////// BGN VEHICLE FUNCTIONS ///////
void vehicle::init(){
    /// TODO
}
/////// END VEHICLE FUNCTIONS ///////

/////// BGN WAYPOINT FUNCTIONS ///////
void waypoint::create_random(parameters* pP){
    
} /// TODO
void waypoint::mutate(parameters* pP){
    
}/// TODO
void waypoint::boundaries(parameters* pP){
    
} /// TODO
/////// END WAYPOINT FUNCTIONS ///////

/////// BGN POI FUNCTIONS ///////
void Point_of_Interest::init(parameters* pP){
    
}
/////// END POI FUNCTIONS ///////

/////// BGN SCOREBOARD FUNCTIONS ///////
void scoreboard::init(){
    
}
/////// END SCOREBOARD FUNCTIONS ///////

/////// BGN PARAMETERS FUNCTIONS ///////
void parameters::init(){
    /// This space left intentionally blank.
}
/////// END PARAMETERS FUNCTIONS ///////

/////// BGN OTHER FUNCTIONS ///////

/////// END OTHER FUNCTIONS ///////



///////////////////// %%%%%%%%%%%%%%%%%% END CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////



int main() {
    srand((unsigned)time(NULL));
    cout << "Start of Program" << endl;
    
    parameters P;
    parameters* pP = &P;
    
    vector<agent> Team;
    for(int a=0; a<pP->num_agents; a++){
        agent AA;
        AA.init(pP);
        Team.push_back(AA);
    }
    assert(Team.size() == pP->num_agents);
    
    cout << "End of Program" << endl;
    return 0;
}
