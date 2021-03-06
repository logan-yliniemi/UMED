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
#include <fstream>

/// uniform random between lower and upper bounds.
#define uniform_random(low,up) ((double)rand()/RAND_MAX*(up-low) + low)

/// max function
double max(double a, double b){
    if(a>b){return a;}
    return b;
}

bool test_functions = false;
bool reset_id = false;

///////////////////// %%%%%%%%%%%%%%%%%% BEGIN CLASS DECLARATIONS %%%%%%%%%%%%%%%%%% /////////////////////

class vehicle;
class waypoint;
class environment;
class Point_of_Interest;
class agent;
class policy;
class observation;
class scoreboard;
class parameters;
class testparameters;
class tests;

void single_generation(vector<agent>*pA,environment* pE,parameters* pPar, int SR, int gen,FILE* p_file,vector<double>* p_best_true_global);
void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar, int gen,FILE* p_file,vector<double>* p_best_true_global);
void stat_run(vector<agent>*pA,environment* pE,parameters* pPar, int SR,FILE* p_file,vector<double>* p_best_true_global);
void advance(vector<agent>*pA,environment* pE,parameters* pPar, int wpnum);

class vehicle{
public:
    int id;
    double x;
    double y;
    double z;
    bool general_comm_link;
    vector<bool> to_agent_comm_link;
    
    void init();
    void start_based_on_policy(policy& rPol, int num_agents);
    void move_to_wp(policy& rPol, int wpnum);
};

class waypoint{
public:
    double x;
    double y;
    double z;
    void create_surface(parameters* pPar);
    void create_random(parameters* pPar);
    void mutate(parameters* pPar);
    void boundaries(parameters* pPar);
};

class agent{
public:
    vehicle V;
    int id;
    vector<policy> policies;
    
    vector<observation> my_observations; /// 1 x POIs
    vector< vector<observation> > others_observations; /// agents x POIs
    
    vector<bool> comms_P2P_available;
    int active_policy_index;
    
    /// functions to initialize.
    void init(parameters* pPar);
    void init_obs_distance(parameters* pPar);
    void start_generation();
    void start_simulation(parameters* pPar);
    void select_fresh_policy();
    
    void observe_poi_distances(environment* pE,parameters* pPar);
    void establish_comms_links(vector<agent>* pA,parameters* pPar);
    void exchange_information_P2P(vector<agent>* pA,parameters* pPar);
    void exchange_information_general(parameters* pPar);
    
    void calc_local(vector<agent>* pA, environment* pE, parameters* pPar);
    void calc_true_global(vector<agent>* pA, environment* pE, parameters* pPar);
    void calc_limited_global(vector<agent>* pA, environment* pE, parameters* pPar);
    void calc_true_difference(vector<agent>* pA, environment* pE, parameters* pPar);
    void calc_limited_difference(vector<agent>* pA, environment* pE, parameters* pPar);
    void LGD_to_fitness(vector<agent>* pA, environment* pE, parameters* pPar);
};

class policy{
public:
    vector<waypoint> WP;
    void init(parameters* pPar);
    
    void start_generation();
    void mutate(parameters* pPar);
    double fitness; /// whichever signal we are using to learn
    double local;
    double true_global;
    double true_difference;
    double limited_global;
    double limited_difference;
    
    int times_selected;
    bool active;
};

class observation{
public:
    double observation_distance;
    int observer_id;
    observation(double d, int id);
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
    int num_agents = 10;
    int num_vehicles = num_agents; // 1 vehicle per agent
    int num_POI = 5*num_agents;
    int pop_size = 500;
    int num_waypoints = 8;
    
    double mutation_size = 5.0;
    
    double max_x = 100;
    double max_y = 100;
    double max_z = 10;
    
    double min_x = 0;
    double min_y = 0;
    double min_z = -100;
    
    double min_poi_value = 1;
    double max_poi_value = 100;
    
    double percent_mutate = 50;
    
    int STAT_RUNS = 30;
    int GENERATIONS = 100;
    bool allow_general_comm_link = true;
    
    double P2P_commlink_dist = 5;
    double maximum_observation_distance = 5;
    
    void init();
    void single_agent_test_overwrite();
};

class tests{
    vector<agent> A;
    environment E;
    parameters P;
    
    environment* pE = &E;
    parameters* pPar = &P;
    vector<agent>* pA = &A;
    
public:
    void init();
    void single_agent_multi_poi();
    void single_agent_check_waypoints_poi();
    void multi_agent_different_path();
    void multi_agent_same_path_no_joy();
    void multi_agent_same_path_with_joy();
    void multi_agent_different_path_different_poi();
    void communication_test();
    void three_agents_limited_joy();
    void three_rover_not_surfacing();
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
        // sets all the polices staring waypoint for an agent the same
        if (pop>0)
        {
            policies.at(pop).WP.at(0) = policies.at(0).WP.at(0);
        }
    }
    /// create a vector of waypoints for each policy (taken care of in policy.init())
    
    /// give each agent a unique ID;
    static int x;
    if (reset_id) {
        id = 0;
        x = 0;
        reset_id = false;
    }
    id = x;
    x++;
    
}
void agent::init_obs_distance(parameters* pPar){
    bool VERBOSE = false;
    double DBL_MAX = std::numeric_limits<double>::max();
    my_observations.clear();
    others_observations.clear();
    for(int p = 0; p<pPar->num_POI; p++){
        observation my(DBL_MAX, id);
        my_observations.push_back(my);
    }
    
    if(VERBOSE){
        cout<<my_observations.at(0).observation_distance<<"\t";
    }
    
    for(int a=0; a<pPar->num_agents; a++){
        vector<observation> otherv;
        for(int p=0; p<pPar->num_POI; p++){
            observation other(DBL_MAX, -1);
            otherv.push_back(other);
        }
        others_observations.push_back(otherv);
    }
    assert(my_observations.size() == pPar->num_POI);
    assert(others_observations.size() == pPar->num_agents);
    assert(others_observations.at(0).size() == pPar->num_POI);
    
}
void agent::start_generation(){
    for(int p=0; p<policies.size(); p++){
        policies.at(p).start_generation();
    }
    active_policy_index = -1;
}
void agent::start_simulation(parameters* pPar){
    init_obs_distance(pPar);
}
void agent::select_fresh_policy(){
    /// selects a random policy that has not been selected this generation:
    /// "times selected" is zero.
    int rnd = rand()%policies.size();
    /// "if the policy has already been used, choose the next one".
    while(policies.at(rnd).times_selected>0){
        rnd++;
        /// "if we get to the end, go to the beginning".
        if(rnd==policies.size()){
            rnd=0;
        }
    }
    policies.at(rnd).times_selected++; /// rnd policy has been selected.
    policies.at(rnd).active = 1; /// rnd policy is active.
    active_policy_index = rnd; /// active policy is rnd.
}
void agent::observe_poi_distances(environment* pE, parameters* pPar){
    for(int p = 0; p<pPar->num_POI; p++){
        double delx = V.x - pE->POIs.at(p).x;
        double dely = V.y - pE->POIs.at(p).y;
        double delz = V.z - pE->POIs.at(p).z;
        double dist = sqrt(delx * delx + dely * dely + delz * delz);
        if(dist > pPar->maximum_observation_distance){
            continue;
        }
        if(dist < my_observations.at(p).observation_distance){
            my_observations.at(p).observation_distance = dist;
            my_observations.at(p).observer_id = id;
        }
    }
}
void agent::establish_comms_links(vector<agent>* pA, parameters* pPar){
    comms_P2P_available.clear();
    comms_P2P_available.reserve(pPar->num_agents);
    for(int a=0; a<pPar->num_agents; a++){
        if(a == id){
            comms_P2P_available.push_back(false);
            continue;
        }
        double delx = V.x - pA->at(a).V.x;
        double dely = V.x - pA->at(a).V.y;
        double delz = V.x - pA->at(a).V.z;
        double dist = sqrt(delx * delx + dely * dely + delz * delz);
        if(dist < pPar->P2P_commlink_dist){
            comms_P2P_available.push_back(true);
        }
        else if((V.z >= 0) && (pA->at(a).V.z >= 0) && (pPar->allow_general_comm_link)){
//        else if((V.z >= 0)  && (pPar->allow_general_comm_link)){
            comms_P2P_available.push_back(true);
        }
        else{
            comms_P2P_available.push_back(false);
        }
    }
    assert(comms_P2P_available.size() == pPar->num_agents);
    assert(comms_P2P_available.at(id) == false);
}
void agent::exchange_information_P2P(vector<agent>* pA, parameters* pPar){
    /// talking to agent "a".
    for(int a=0; a<pPar->num_agents; a++){
        if(comms_P2P_available.at(a) == false){
            continue;
        }
        
        /// if the comms link is available...
        for(int p = 0; p<pPar->num_POI; p++){
            /// update my observations for every POI into the agent a's database.
            if(my_observations.at(p).observation_distance < pA->at(a).others_observations.at(id).at(p).observation_distance){
                pA->at(a).others_observations.at(id).at(p).observation_distance = my_observations.at(p).observation_distance;
                pA->at(a).others_observations.at(id).at(p).observer_id = id;
            }
        }
        /// update my knowledge of every other "b" agent into a's database database.
        for(int b=0; b<pPar->num_agents; b++){
            if(b==a){
                // but don't bother telling b about a if b is a.
                continue;
            }
            for(int p=0; p<pPar->num_POI; p++){
                if(others_observations.at(b).at(p).observation_distance < pA->at(a).others_observations.at(b).at(p).observation_distance){
                    pA->at(a).others_observations.at(b).at(p).observation_distance = others_observations.at(a).at(p).observation_distance;
                    pA->at(a).others_observations.at(b).at(p).observer_id = others_observations.at(a).at(p).observer_id;
                }
            }
        }
        
        /// the following two happen during the other running this function:
        /// update the other's observations into my database.
        /// update the other's knowledge of other's into my database.
    }
}
void agent::calc_local(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).local = 0;
    double L = 0;
    for(int p=0; p<pPar->num_POI; p++){
        double value = pE->POIs.at(p).val;
        double delta = max(1.0,my_observations.at(p).observation_distance);
        double contribution = value/delta;
        L+=contribution;
    }
    policies.at(active_policy_index).local = L;
    return;
}
void agent::calc_true_global(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).true_global = 0;
    /// LYLY Note that this is current calculated "agents" times, and does not strictly need to be; the true_global could be calculated once and broadcast.
    double dis;
    double g = 0;
    vector<double> closest(pPar->num_POI,std::numeric_limits<double>::max());
    for(int p=0; p<pPar->num_POI; p++){
        for(int a=0; a<pPar->num_agents; a++){
            dis = pA->at(a).my_observations.at(p).observation_distance;
            if(dis < closest.at(p)){
                closest.at(p) = dis;
            }
        }
    }
    
    double value, delta, contribution;
    for(int p=0; p<pPar->num_POI; p++){
        value = pE->POIs.at(p).val;
        delta = max(1.0,closest.at(p));
        contribution = value/delta;
        g+=contribution;
    }
    
    policies.at(active_policy_index).true_global = g;
    return;
}
void agent::calc_limited_global(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).limited_global = 0;
    /// TODO use "others observations".
    double limg = 0;
    
    double dis;
    vector<double> closest(pPar->num_POI,std::numeric_limits<double>::max());
    for(int p=0; p<pPar->num_POI; p++){
        /// consider my observations...
        dis = my_observations.at(p).observation_distance;
        if(dis < closest.at(p)){
            closest.at(p) = dis;
        }
        /// and everyone else's that i know about...
        for(int a=0; a<pPar->num_agents; a++){
            dis = others_observations.at(a).at(p).observation_distance;
            if(dis < closest.at(p)){
                closest.at(p) = dis;
            }
        }
    }
    
    /// calculated limg based on the closest of all of these.
    double value, delta, contribution;
    for(int p=0; p<pPar->num_POI; p++){
        value = pE->POIs.at(p).val;
        delta = max(1.0,closest.at(p));
        contribution = value/delta;
        limg+=contribution;
    }
    
    policies.at(active_policy_index).limited_global = limg;
    return;
}
void agent::calc_true_difference(vector<agent>* pA, environment* pE, parameters* pPar){
    bool VERBOSE = false;
    policies.at(active_policy_index).true_difference = 0;
    double true_cf = 0;
    /// TODO use all agent's "my observations"
    if(VERBOSE){
        for (int rover_number = 0; rover_number<pA->size(); rover_number++) {
            cout<<"This is for rover::"<<rover_number<<endl;
            for (int temp = 0; temp < pA->at(rover_number).my_observations.size(); temp++) {
                cout<<pA->at(rover_number).my_observations.at(temp).observation_distance<<"\t";
                cout<<pA->at(rover_number).my_observations.at(temp).observer_id<<"\t";
                cout<<endl;
            }
            cout<<endl;
        }
    }
    vector<double> closest(pPar->num_POI,std::numeric_limits<double>::max());
    double dis;
    for(int a=0; a<pPar->num_agents; a++){
        if(a==id){
            continue;
        }
        for(int p=0; p<pPar->num_POI; p++){
            dis = pA->at(a).my_observations.at(p).observation_distance;
            if(dis<closest.at(p)){
                closest.at(p) = dis;
            }
        }
    }
    
    /// calculate true_cf based on the closest of these.
    double value, delta, contribution;
    for(int p=0; p<pPar->num_POI; p++){
        value = pE->POIs.at(p).val;
        delta = max(1.0,closest.at(p));
        contribution = value/delta;
        true_cf+=contribution;
    }
    
    policies.at(active_policy_index).true_difference = policies.at(active_policy_index).true_global - true_cf;
    return;
}
void agent::calc_limited_difference(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).limited_difference = 0;
    double limited_cf = 0;
    /// use "others observations.
    
    ////////////////////////////////
    double dis;
    vector<double> closest(pPar->num_POI,std::numeric_limits<double>::max());
    for(int p=0; p<pPar->num_POI; p++){
        /// consider only everyone else's that i know about...
        for(int a=0; a<pPar->num_agents; a++){
            dis = others_observations.at(a).at(p).observation_distance;
            if(dis < closest.at(p)){
                closest.at(p) = dis;
            }
        }
    }
    
    /// calculated limcf based on the closest of all of these.
    double value, delta, contribution;
    for(int p=0; p<pPar->num_POI; p++){
        value = pE->POIs.at(p).val;
        delta = max(1.0,closest.at(p));
        contribution = value/delta;
        limited_cf+=contribution;
    }
    ////////////////////////////////
    
    policies.at(active_policy_index).limited_difference = policies.at(active_policy_index).limited_global - limited_cf;
    return;
}
void agent::LGD_to_fitness(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).fitness = policies.at(active_policy_index).local;
    /// TODO should indicate this from within the parameters class.
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
void policy::start_generation(){
    local = 0;
    true_global = 0;
    true_difference = 0;
    limited_global = 0;
    limited_difference = 0;
    times_selected = 0;
    fitness = 0;
    active = false;
}
void policy::mutate(parameters* pPar){
    for(int w=1; w<WP.size(); w++){
        double r_num = ((double) rand() / (RAND_MAX));
        //percent chance that the waypoint in question will mutate
        if (r_num<=pPar->percent_mutate/100)
        {
            //cout << "waypoint" << "\t" << w << endl;
            WP.at(w).mutate(pPar);
        }
    }
}
/////// END Policy FUNCTIONS ///////

/////// BGN Observation FUNCTIONS ///////
observation::observation(double d, int i){
    observation_distance = d;
    observer_id = i;
}
/////// END Observation FUNCTIONS ///////

/////// BGN VEHICLE FUNCTIONS ///////
void vehicle::init(){
    /// each vehicle gets a unique ID
    static int index;
    id = index;
    index++;
}
void vehicle::start_based_on_policy(policy& rPol, int num_agents){
    x = rPol.WP.at(0).x;
    y = rPol.WP.at(0).y;
    z = rPol.WP.at(0).z;
    //cout << "starting telem" << "\t" << x << "\t" << y << "\t" << z << endl;
    
    general_comm_link = false;
    
    to_agent_comm_link.clear();
    for(int agent = 0; agent < num_agents; agent++){
        to_agent_comm_link.push_back(false);
    }
    assert(to_agent_comm_link.size() == num_agents);
}
void vehicle::move_to_wp(policy& rPol, int wp){
    x = rPol.WP.at(wp).x;
    y = rPol.WP.at(wp).y;
    z = rPol.WP.at(wp).z;
    //cout << "new telem" << "\t" << x << "\t" << y << "\t" << z << endl;
    
    if(z > 0){
        general_comm_link = true;
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
void waypoint::mutate(parameters* pPar){
    x += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    y += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    z += uniform_random(-pPar->mutation_size,pPar->mutation_size);
    boundaries(pPar);
}
void waypoint::boundaries(parameters* pPar){
    // keeps mutated waypoints within boundaries of system.
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
void parameters::single_agent_test_overwrite(){
    num_agents = 1;
    num_vehicles = num_agents;
    pop_size = 1;
    num_POI = 2;
    num_waypoints = 4;
    
    P2P_commlink_dist = 300;
    maximum_observation_distance = 100;
}
/////// END PARAMETERS FUNCTIONS ///////

/////// BGN TESTS FUNCTIONS ///////
void tests::init(){
    P.single_agent_test_overwrite();
    agent AA;
    A.push_back(AA);
}

void tests::single_agent_multi_poi(){
    P.init();
    A.at(0).init(pPar);
    E.init(pPar);
    
    double delta = 0.001;
    
    P.maximum_observation_distance = 1;
    
    /// ONE AGENT, TWO POIS, WE CARE ABOUT ONE.
    E.POIs.at(0).x = 10;
    E.POIs.at(0).y = 10;
    E.POIs.at(0).z = -10;
    E.POIs.at(0).val = 100;
    E.POIs.at(1).x = 90;
    E.POIs.at(1).y = 90;
    E.POIs.at(1).z = -90;
    E.POIs.at(1).val = 100;
    
    A.at(0).policies.at(0).WP.at(0).x = 0;
    A.at(0).policies.at(0).WP.at(0).y = 0;
    A.at(0).policies.at(0).WP.at(0).z = 0;
    
    A.at(0).policies.at(0).WP.at(1).x = 0;
    A.at(0).policies.at(0).WP.at(1).y = 0;
    A.at(0).policies.at(0).WP.at(1).z = 0;
    
    A.at(0).policies.at(0).WP.at(2).x = 0;
    A.at(0).policies.at(0).WP.at(2).y = 0;
    A.at(0).policies.at(0).WP.at(2).z = 0;
    
    A.at(0).policies.at(0).WP.at(3).x = 0;
    A.at(0).policies.at(0).WP.at(3).y = 0;
    A.at(0).policies.at(0).WP.at(3).z = 0;
    
    A.at(0).start_generation();
    A.at(0).start_simulation(pPar);
    A.at(0).select_fresh_policy();
    
    FILE* p_obj;
    p_obj=fopen("temp","a");
    vector<double> temp_vec;
    vector<double>* p_temp_vec= &temp_vec;
    for (int i=0; i<1; i++) {
        temp_vec.push_back(1);
    }
    single_simulation(pA,pE,pPar,0,p_obj,p_temp_vec);
    
    policy* pPol = &A.at(0).policies.at(0);
    if(P.maximum_observation_distance > 18){
        /// local == difference;
        assert(pPol->local == pPol->true_difference);
        /// local == global;
        assert(pPol->local == pPol->true_global);
        /// ld == difference;
        assert(pPol->limited_difference == pPol->true_difference);
        /// lg == global;
        //assert(pPol->limited_global == pPol->true_global);
    }
    else{
        assert(pPol->local < delta);
        assert(pPol->true_global < delta);
        assert(pPol->true_difference < delta);
        assert(pPol->limited_global < delta);
        assert(pPol->limited_difference < delta);
    }
    
    double L1 = pPol->local;
    
    
    assert(true);
    
    A.at(0).policies.at(0).WP.at(0).x = 0;
    A.at(0).policies.at(0).WP.at(0).y = 0;
    A.at(0).policies.at(0).WP.at(0).z = 0;
    
    A.at(0).policies.at(0).WP.at(1).x = 10;
    A.at(0).policies.at(0).WP.at(1).y = 10;
    A.at(0).policies.at(0).WP.at(1).z = -10;
    
    A.at(0).policies.at(0).WP.at(2).x = 0;
    A.at(0).policies.at(0).WP.at(2).y = 0;
    A.at(0).policies.at(0).WP.at(2).z = 0;
    
    A.at(0).policies.at(0).WP.at(3).x = 0;
    A.at(0).policies.at(0).WP.at(3).y = 0;
    A.at(0).policies.at(0).WP.at(3).z = 0;
    
    A.at(0).start_generation();
    A.at(0).start_simulation(pPar);
    A.at(0).select_fresh_policy();
    
    single_simulation(pA,pE,pPar,0,p_obj,p_temp_vec);
    
    double L2 = pPol->local;
    
    assert(L2 > L1);
    
    A.at(0).policies.at(0).WP.at(0).x = 0;
    A.at(0).policies.at(0).WP.at(0).y = 0;
    A.at(0).policies.at(0).WP.at(0).z = 0;
    
    A.at(0).policies.at(0).WP.at(1).x = 10;
    A.at(0).policies.at(0).WP.at(1).y = 10;
    A.at(0).policies.at(0).WP.at(1).z = -10;
    
    A.at(0).policies.at(0).WP.at(2).x = 90;
    A.at(0).policies.at(0).WP.at(2).y = 90;
    A.at(0).policies.at(0).WP.at(2).z = -90;
    
    A.at(0).policies.at(0).WP.at(3).x = 0;
    A.at(0).policies.at(0).WP.at(3).y = 0;
    A.at(0).policies.at(0).WP.at(3).z = 0;
    
    A.at(0).start_generation();
    A.at(0).start_simulation(pPar);
    A.at(0).select_fresh_policy();
    
    single_simulation(pA,pE,pPar,0,p_obj,p_temp_vec);
    fclose(p_obj);
    
    double L3 = pPol->local;
    assert(L3 > L2);
    
    /// local == difference;
    assert(pPol->local == pPol->true_difference);
    /// local == global;
    assert(pPol->local == pPol->true_global);
    /// ld == difference;
    assert(pPol->limited_difference == pPol->true_difference);
    /// lg == global;
    assert(pPol->limited_global == pPol->true_global);
    
}

void tests::single_agent_check_waypoints_poi(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 1;
    pPar->num_vehicles = 1;
    pPar->pop_size =1;
    pPar->num_POI = 2;
    pPar->num_waypoints =4;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist=1;
    
    //Create a agent and initalize its values
    agent a_1;
    A.clear();
    A.push_back(a_1);
    A.at(0).init(pPar);
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    //E.POIs.clear();
    E.POIs.at(0).x = 10;
    E.POIs.at(0).y = 10;
    E.POIs.at(0).z = -10;
    E.POIs.at(0).val = 100;
    E.POIs.at(1).x = 90;
    E.POIs.at(1).y = 90;
    E.POIs.at(1).z = -90;
    E.POIs.at(1).val = 100;
    
    
    //Agent location
    A.at(0).policies.at(0).WP.at(0).x = 0;
    A.at(0).policies.at(0).WP.at(0).y = 0;
    A.at(0).policies.at(0).WP.at(0).z = 0;
    
    A.at(0).policies.at(0).WP.at(1).x = 10;
    A.at(0).policies.at(0).WP.at(1).y = 10;
    A.at(0).policies.at(0).WP.at(1).z = -10;
    
    A.at(0).policies.at(0).WP.at(2).x = 90;
    A.at(0).policies.at(0).WP.at(2).y = 90;
    A.at(0).policies.at(0).WP.at(2).z = -90;
    
    A.at(0).policies.at(0).WP.at(3).x = 0;
    A.at(0).policies.at(0).WP.at(3).y = 0;
    A.at(0).policies.at(0).WP.at(3).z = 0;
    
    
    A.at(0).start_generation(); // all policies have everything to zero
    A.at(0).start_simulation(pPar);
    A.at(0).select_fresh_policy();
    A.at(0).V.start_based_on_policy(A.at(0).policies.at(0), pPar->num_agents); //placed at first waypoint
    
    for (int temp_1 =0; temp_1<A.at(0).policies.at(0).WP.size(); temp_1++) {
        int dex = pA->at(0).active_policy_index;
        policy P = pA->at(0).policies.at(dex);
        pA->at(0).V.move_to_wp(P,temp_1);
        
        assert(A.at(0).policies.at(0).WP.at(temp_1).x == A.at(0).V.x);
        assert(A.at(0).policies.at(0).WP.at(temp_1).y == A.at(0).V.y);
        assert(A.at(0).policies.at(0).WP.at(temp_1).z == A.at(0).V.z);
        
        if(VERBOSE){
            cout<<"Location of Agent::"<<endl;
            cout<<A.at(0).V.x<<"\t"<<A.at(0).V.y<<"\t"<<A.at(0).V.z<<endl;
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
        
    }
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    //assert to check all values to calculated values
    if(VERBOSE){
        cout<<"fitness:"<<A.at(0).policies.at(0).fitness<<endl;                   //fitness;
        cout<<"local::"<<A.at(0).policies.at(0).local<<endl;                      //local;
        cout<<"true_global::"<<A.at(0).policies.at(0).true_global<<endl;          //true_global;
        cout<<"true_difference::"<<A.at(0).policies.at(0).true_difference<<endl;  //true_difference;
        cout<<"limited_global::"<<A.at(0).policies.at(0).limited_global<<endl;    //limited_global;
        cout<<"limited_difference::"<<A.at(0).policies.at(0).limited_difference<<endl;  //limited_difference;
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).local);
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_global == A.at(0).policies.at(0).limited_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(0).policies.at(0).limited_difference);
    
    //
    reset_id = false;
}

void tests::multi_agent_different_path(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
//    reset_id = true;
//    A.clear();
//    pA->clear();
//    E.POIs.clear();
    
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 2;
    pPar->num_waypoints =6;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist=0;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    E.POIs.at(0).x = 10;
    E.POIs.at(0).y = 10;
    E.POIs.at(0).z = -10;
    E.POIs.at(0).val = 100;
    E.POIs.at(1).x = 90;
    E.POIs.at(1).y = 90;
    E.POIs.at(1).z = -90;
    E.POIs.at(1).val = 100;
    
    
    //Agent location
    int waypoint_location = 0;
    for (int temp_rover_number = 0; temp_rover_number<pPar->num_agents; temp_rover_number++) {
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -10;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -90;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        if (temp_rover_number == 0) {
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 95;
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 95;
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -95;
        }else{
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 92;
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 92;
            A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -92;
        }
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -100;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
        
        
    }
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if (rover_number == 1) {
                assert(A.at(rover_number).policies.at(0).WP.at(time_step).x != A.at(rover_number-1).policies.at(0).WP.at(time_step).x);
            }
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
                //pA->at(a).V.z
                cout<<pA->at(rover_number).V.x<<"\t"<<pA->at(rover_number).V.y<<"\t"<<pA->at(rover_number).V.z<<endl;
                
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    //assert to check all values to calculated values
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;      //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;                //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl; //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;//true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl; //limited_global;
        cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl;//limited_difference;
            
        }
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(1).policies.at(0).fitness);
    assert(A.at(0).policies.at(0).local == A.at(1).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(1).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).limited_global == A.at(1).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).limited_difference == A.at(1).policies.at(0).limited_difference);
    
    reset_id = false;
}

void tests::multi_agent_same_path_no_joy(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 2;
    pPar->num_waypoints =5;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist=1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    E.POIs.at(0).x = 10;
    E.POIs.at(0).y = 10;
    E.POIs.at(0).z = -10;
    E.POIs.at(0).val = 100;
    E.POIs.at(1).x = 90;
    E.POIs.at(1).y = 90;
    E.POIs.at(1).z = -90;
    E.POIs.at(1).val = 100;
    
    
    //Agent location
    for (int temp_rover_number = 0; temp_rover_number<pPar->num_agents; temp_rover_number++) {
        int waypoint_location = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
        waypoint_location++;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -10;
        waypoint_location++;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -90;
        waypoint_location++;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -100;
        waypoint_location++;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
    }
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in same path but no communication
    //for each time step
    for (int time_step = 0; time_step<=A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            if (rover_number == 1) {
                time_step--;
            }
            if (rover_number == 0 && time_step == A.at(0).policies.at(0).WP.size()) {
                continue;
            }
            if (rover_number == 1 && time_step == -1) {
                break;
            }
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
        
        
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
                
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
        
        time_step++;
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    //assert to check all values to calculated values
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;                         //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;                            //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl;                //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;            //true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl;          //limited_global;
            cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl;  //limited_difference;
            
        }
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(1).policies.at(0).fitness);
    assert(A.at(0).policies.at(0).local == A.at(1).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(1).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).limited_global == A.at(1).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).limited_difference == A.at(1).policies.at(0).limited_difference);
    
}

void tests::multi_agent_same_path_with_joy(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 2;
    pPar->num_waypoints =5;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist=1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    E.POIs.at(0).x = 10;
    E.POIs.at(0).y = 10;
    E.POIs.at(0).z = -10;
    E.POIs.at(0).val = 100;
    E.POIs.at(1).x = 100;
    E.POIs.at(1).y = 100;
    E.POIs.at(1).z = -100;
    E.POIs.at(1).val = 100;
    
    
    //Agent location
    int waypoint_location = 0;
    for (int temp_rover_number = 0; temp_rover_number<pPar->num_agents; temp_rover_number++) {
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 0;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 10;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -10;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 90;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -90;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 100;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = -100;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 120;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 120;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 0;
        
    }
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if (rover_number == 1 && time_step != 2 ) {
                assert(A.at(rover_number).policies.at(0).WP.at(time_step).x != A.at(rover_number-1).policies.at(0).WP.at(time_step).x);
            }
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
        
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    //assert to check all values to calculated values
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"fitness:"<<A.at(0).policies.at(0).fitness<<endl;                         //fitness;
            cout<<"local::"<<A.at(0).policies.at(0).local<<endl;                            //local;
            cout<<"true_global::"<<A.at(0).policies.at(0).true_global<<endl;                //true_global;
            cout<<"true_difference::"<<A.at(0).policies.at(0).true_difference<<endl;            //true_difference;
            cout<<"limited_global::"<<A.at(0).policies.at(0).limited_global<<endl;          //limited_global;
            cout<<"limited_difference::"<<A.at(0).policies.at(0).limited_difference<<endl;  //limited_difference;
            
        }
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(1).policies.at(0).fitness);
    assert(A.at(0).policies.at(0).local == A.at(1).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(1).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).limited_global == A.at(1).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).limited_difference == A.at(1).policies.at(0).limited_difference);

    
}

void tests::multi_agent_different_path_different_poi(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 6;
    pPar->num_waypoints = 14;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist = 1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    //Food for agent_1
    for (int temp=0, temp_1=10; temp<3; temp++) {
        E.POIs.at(temp).x =temp_1;
        E.POIs.at(temp).y = temp_1;
        E.POIs.at(temp).z = -temp_1;
        E.POIs.at(temp).val = 100;
        temp_1 += 5;
    }
    //Food for agent_2
    for (int temp=3, temp_1=100; temp<6; temp++) {
        E.POIs.at(temp).x =temp_1;
        E.POIs.at(temp).y = temp_1;
        E.POIs.at(temp).z = -temp_1;
        E.POIs.at(temp).val = 100;
        temp_1 -= 25;
    }
    
    //Agent location
    for (int temp = 0, temp_1=0; temp<pPar->num_waypoints; temp++) {
        A.at(0).policies.at(0).WP.at(temp).x = temp_1;
        A.at(0).policies.at(0).WP.at(temp).y = temp_1;
        A.at(0).policies.at(0).WP.at(temp).z = -temp_1;
        temp_1 += 5;
        if (temp == 5) {
            A.at(0).policies.at(0).WP.at(temp).x = 22;
            A.at(0).policies.at(0).WP.at(temp).y = 22;
            A.at(0).policies.at(0).WP.at(temp).z = -22;
        }
        if (temp >= 6) {
            A.at(0).policies.at(0).WP.at(temp).x = 13;
            A.at(0).policies.at(0).WP.at(temp).y = 13;
            A.at(0).policies.at(0).WP.at(temp).z = -13;
        }
    }
    
    for (int temp = 0, temp_1=200; temp<pPar->num_waypoints; temp++) {
        A.at(1).policies.at(0).WP.at(temp).x = temp_1;
        A.at(1).policies.at(0).WP.at(temp).y = temp_1;
        A.at(1).policies.at(0).WP.at(temp).z = -temp_1;
        if (temp == 0 ) {
            A.at(1).policies.at(0).WP.at(temp).x = 200;
            A.at(1).policies.at(0).WP.at(temp).y = 200;
            A.at(1).policies.at(0).WP.at(temp).z = 0;
        }
        temp_1 -= 25;
        if (temp >= 8) {
            A.at(1).policies.at(0).WP.at(temp).x = 100;
            A.at(1).policies.at(0).WP.at(temp).y = 100;
            A.at(1).policies.at(0).WP.at(temp).z = -100;
        }
    }
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"For rover::"<<rover_number<<endl;
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;            //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;               //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl; //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;  //true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl;          //limited_global;
            cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl;  //limited_difference;
            cout<<endl;
            
        }
    }
    assert(A.at(0).policies.at(0).fitness == A.at(1).policies.at(0).fitness);
    assert(A.at(0).policies.at(0).local == A.at(1).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(1).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).limited_global == A.at(1).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).limited_difference == A.at(1).policies.at(0).limited_difference);

//    cout<<"Check Location"<<endl;
}

void tests::communication_test(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 6;
    pPar->num_waypoints = 14;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist = 1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    //Food for agent_1
    for (int temp=0, temp_1=10; temp<3; temp++) {
        E.POIs.at(temp).x =temp_1;
        E.POIs.at(temp).y = temp_1;
        E.POIs.at(temp).z = -temp_1;
        E.POIs.at(temp).val = 100;
        temp_1 += 5;
    }
    //Food for agent_2
    for (int temp=3, temp_1=100; temp<6; temp++) {
        E.POIs.at(temp).x =temp_1;
        E.POIs.at(temp).y = temp_1;
        E.POIs.at(temp).z = -temp_1;
        E.POIs.at(temp).val = 100;
        temp_1 -= 25;
    }
    
    //Agent location
    for (int temp = 0, temp_1=0; temp<pPar->num_waypoints; temp++) {
        A.at(0).policies.at(0).WP.at(temp).x = temp_1;
        A.at(0).policies.at(0).WP.at(temp).y = temp_1;
        A.at(0).policies.at(0).WP.at(temp).z = -temp_1;
        temp_1 += 5;
        if (temp == 5) {
            A.at(0).policies.at(0).WP.at(temp).x = 25;
            A.at(0).policies.at(0).WP.at(temp).y = 25;
            A.at(0).policies.at(0).WP.at(temp).z = -25;
        }
        if (temp >= 6) {
            A.at(0).policies.at(0).WP.at(temp).x = 25;
            A.at(0).policies.at(0).WP.at(temp).y = 25;
            A.at(0).policies.at(0).WP.at(temp).z = -25;
        }
    }
    
    for (int temp = 0, temp_1=200; temp<pPar->num_waypoints; temp++) {
        A.at(1).policies.at(0).WP.at(temp).x = temp_1;
        A.at(1).policies.at(0).WP.at(temp).y = temp_1;
        A.at(1).policies.at(0).WP.at(temp).z = -temp_1;
        if (temp == 0 ) {
            A.at(1).policies.at(0).WP.at(temp).x = 200;
            A.at(1).policies.at(0).WP.at(temp).y = 200;
            A.at(1).policies.at(0).WP.at(temp).z = 0;
        }
        temp_1 -= 25;
        if (temp >= 8) {
            A.at(1).policies.at(0).WP.at(temp).x = 100;
            A.at(1).policies.at(0).WP.at(temp).y = 100;
            A.at(1).policies.at(0).WP.at(temp).z = -100;
        }
    }
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"For rover::"<<rover_number<<endl;
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;            //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;               //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl; //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;  //true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl;          //limited_global;
            cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl;  //limited_difference;
            cout<<endl;
            
        }
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(1).policies.at(0).fitness);
    assert(A.at(0).policies.at(0).local == A.at(1).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(1).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).limited_global == A.at(1).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).limited_difference == A.at(1).policies.at(0).limited_difference);

//    cout<<"Check Location"<<endl;
    
    
}

void tests::three_agents_limited_joy(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 3;
    pPar->num_vehicles = 3;
    pPar->pop_size = 1;
    pPar->num_POI = 5;
    pPar->num_waypoints = 7;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist = 1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    agent a_3;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    A.push_back(a_3);
    
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    E.POIs.at(0).x =5;
    E.POIs.at(0).y =5;
    E.POIs.at(0).z =-5;
    E.POIs.at(1).x =25;
    E.POIs.at(1).y =25;
    E.POIs.at(1).z =-25;
    E.POIs.at(2).x =35;
    E.POIs.at(2).y =35;
    E.POIs.at(2).z =-35;
    E.POIs.at(3).x =50;
    E.POIs.at(3).y =50;
    E.POIs.at(3).z =-50;
    E.POIs.at(4).x =78;
    E.POIs.at(4).y =78;
    E.POIs.at(4).z =-78;
    E.POIs.at(0).val=100;
    E.POIs.at(1).val=100;
    E.POIs.at(2).val=100;
    E.POIs.at(3).val=100;
    E.POIs.at(4).val=100;
    
    
    //Agent way point
    //Agent 1
    for (int temp =0,temp_1 =0; temp<7; temp++) {
        A.at(0).policies.at(0).WP.at(temp).x=temp_1;
        A.at(0).policies.at(0).WP.at(temp).y=temp_1;
        A.at(0).policies.at(0).WP.at(temp).z=-temp_1;
        temp_1+=5;
        if (temp == 6) {
            A.at(0).policies.at(0).WP.at(temp).x=120;
            A.at(0).policies.at(0).WP.at(temp).y=120;
            A.at(0).policies.at(0).WP.at(temp).z=0;
        }
    }
    //Agent 2
    A.at(1).policies.at(0).WP.at(0).x = 30;
    A.at(1).policies.at(0).WP.at(0).y = 30;
    A.at(1).policies.at(0).WP.at(0).z = 0;
    A.at(1).policies.at(0).WP.at(1).x = 25;
    A.at(1).policies.at(0).WP.at(1).y = 25;
    A.at(1).policies.at(0).WP.at(1).z = -25;
    A.at(1).policies.at(0).WP.at(2).x = 10;
    A.at(1).policies.at(0).WP.at(2).y = 10;
    A.at(1).policies.at(0).WP.at(2).z = -10;
    A.at(1).policies.at(0).WP.at(3).x = 50;
    A.at(1).policies.at(0).WP.at(3).y = 50;
    A.at(1).policies.at(0).WP.at(3).z = -50;
    A.at(1).policies.at(0).WP.at(4).x = 60;
    A.at(1).policies.at(0).WP.at(4).y = 60;
    A.at(1).policies.at(0).WP.at(4).z = -60;
    A.at(1).policies.at(0).WP.at(5).x = 110;
    A.at(1).policies.at(0).WP.at(5).y = 110;
    A.at(1).policies.at(0).WP.at(5).z = -110;
    A.at(1).policies.at(0).WP.at(6).x = 140;
    A.at(1).policies.at(0).WP.at(6).y = 140;
    A.at(1).policies.at(0).WP.at(6).z = 0;
    //Agent 2
    A.at(2).policies.at(0).WP.at(0).x = 40;
    A.at(2).policies.at(0).WP.at(0).y = 40;
    A.at(2).policies.at(0).WP.at(0).z = 0;
    A.at(2).policies.at(0).WP.at(1).x = 40;
    A.at(2).policies.at(0).WP.at(1).y = 40;
    A.at(2).policies.at(0).WP.at(1).z = -40;
    A.at(2).policies.at(0).WP.at(2).x = 35;
    A.at(2).policies.at(0).WP.at(2).y = 35;
    A.at(2).policies.at(0).WP.at(2).z = -35;
    A.at(2).policies.at(0).WP.at(3).x = 50;
    A.at(2).policies.at(0).WP.at(3).y = 50;
    A.at(2).policies.at(0).WP.at(3).z = -50;
    A.at(2).policies.at(0).WP.at(4).x = 75;
    A.at(2).policies.at(0).WP.at(4).y = 75;
    A.at(2).policies.at(0).WP.at(4).z = -75;
    A.at(2).policies.at(0).WP.at(5).x = 100;
    A.at(2).policies.at(0).WP.at(5).y = 100;
    A.at(2).policies.at(0).WP.at(5).z = -100;
    A.at(2).policies.at(0).WP.at(6).x = 100;
    A.at(2).policies.at(0).WP.at(6).y = 100;
    A.at(2).policies.at(0).WP.at(6).z = 0;
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"For rover::"<<rover_number<<endl;
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;            //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;               //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl; //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;  //true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl;//limited_global;
            cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl; //limited_difference;
            cout<<endl;
            
        }
    }
    assert(A.at(0).policies.at(0).limited_global == A.at(0).policies.at(0).true_global);
    assert(A.at(1).policies.at(0).limited_global == A.at(1).policies.at(0).true_global);
    assert(A.at(2).policies.at(0).limited_global == A.at(2).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(0).policies.at(0).limited_difference);
    assert(A.at(1).policies.at(0).true_difference == A.at(1).policies.at(0).limited_difference);
    assert(A.at(2).policies.at(0).true_difference == A.at(2).policies.at(0).limited_difference);
    assert(A.at(2).policies.at(0).true_global == A.at(1).policies.at(0).true_global);
    assert(A.at(2).policies.at(0).true_global == A.at(0).policies.at(0).true_global);
    
    
}

void tests::three_rover_not_surfacing(){
    bool VERBOSE = false;
    reset_id = true;
    A.clear();
    E.POIs.clear();
    pA->clear();
    
    //Set all the values
    pPar->num_agents = 3;
    pPar->num_vehicles = 3;
    pPar->pop_size = 1;
    pPar->num_POI = 5;
    pPar->num_waypoints = 7;
    pPar->maximum_observation_distance = 1;
    pPar->P2P_commlink_dist = 1;
    
    //Create a agent and initalize its values
    agent a_1;
    agent a_2;
    agent a_3;
    A.clear();
    A.push_back(a_1);
    A.push_back(a_2);
    A.push_back(a_3);
    
    for (int temp_num_agents = 0; temp_num_agents<pPar->num_agents; temp_num_agents++) {
        A.at(temp_num_agents).init(pPar);
    }
    
    
    //Create environment
    E.init(pPar);
    
    double delta = 0.001; // This value is used in calculation of local values
    
    /// Create location for POI
    E.POIs.at(0).x =5;
    E.POIs.at(0).y =5;
    E.POIs.at(0).z =-5;
    E.POIs.at(1).x =25;
    E.POIs.at(1).y =25;
    E.POIs.at(1).z =-25;
    E.POIs.at(2).x =35;
    E.POIs.at(2).y =35;
    E.POIs.at(2).z =-35;
    E.POIs.at(3).x =50;
    E.POIs.at(3).y =50;
    E.POIs.at(3).z =-50;
    E.POIs.at(4).x =78;
    E.POIs.at(4).y =78;
    E.POIs.at(4).z =-78;
    E.POIs.at(0).val=100;
    E.POIs.at(1).val=100;
    E.POIs.at(2).val=100;
    E.POIs.at(3).val=100;
    E.POIs.at(4).val=100;
    
    
    //Agent way point
    //Agent 1
    for (int temp =0,temp_1 =0; temp<7; temp++) {
        A.at(0).policies.at(0).WP.at(temp).x=temp_1;
        A.at(0).policies.at(0).WP.at(temp).y=temp_1;
        A.at(0).policies.at(0).WP.at(temp).z=-temp_1;
        temp_1+=5;
        if (temp == 6) {
            A.at(0).policies.at(0).WP.at(temp).x=120;
            A.at(0).policies.at(0).WP.at(temp).y=120;
            A.at(0).policies.at(0).WP.at(temp).z=-160;
        }
    }
    //Agent 2
    A.at(1).policies.at(0).WP.at(0).x = 30;
    A.at(1).policies.at(0).WP.at(0).y = 30;
    A.at(1).policies.at(0).WP.at(0).z = 0;
    A.at(1).policies.at(0).WP.at(1).x = 25;
    A.at(1).policies.at(0).WP.at(1).y = 25;
    A.at(1).policies.at(0).WP.at(1).z = -25;
    A.at(1).policies.at(0).WP.at(2).x = 10;
    A.at(1).policies.at(0).WP.at(2).y = 10;
    A.at(1).policies.at(0).WP.at(2).z = -10;
    A.at(1).policies.at(0).WP.at(3).x = 50;
    A.at(1).policies.at(0).WP.at(3).y = 50;
    A.at(1).policies.at(0).WP.at(3).z = -50;
    A.at(1).policies.at(0).WP.at(4).x = 60;
    A.at(1).policies.at(0).WP.at(4).y = 60;
    A.at(1).policies.at(0).WP.at(4).z = -60;
    A.at(1).policies.at(0).WP.at(5).x = 110;
    A.at(1).policies.at(0).WP.at(5).y = 110;
    A.at(1).policies.at(0).WP.at(5).z = -110;
    A.at(1).policies.at(0).WP.at(6).x = 140;
    A.at(1).policies.at(0).WP.at(6).y = 140;
    A.at(1).policies.at(0).WP.at(6).z = -150;
    //Agent 2
    A.at(2).policies.at(0).WP.at(0).x = 40;
    A.at(2).policies.at(0).WP.at(0).y = 40;
    A.at(2).policies.at(0).WP.at(0).z = 0;
    A.at(2).policies.at(0).WP.at(1).x = 40;
    A.at(2).policies.at(0).WP.at(1).y = 40;
    A.at(2).policies.at(0).WP.at(1).z = -40;
    A.at(2).policies.at(0).WP.at(2).x = 35;
    A.at(2).policies.at(0).WP.at(2).y = 35;
    A.at(2).policies.at(0).WP.at(2).z = -35;
    A.at(2).policies.at(0).WP.at(3).x = 50;
    A.at(2).policies.at(0).WP.at(3).y = 50;
    A.at(2).policies.at(0).WP.at(3).z = -50;
    A.at(2).policies.at(0).WP.at(4).x = 75;
    A.at(2).policies.at(0).WP.at(4).y = 75;
    A.at(2).policies.at(0).WP.at(4).z = -75;
    A.at(2).policies.at(0).WP.at(5).x = 100;
    A.at(2).policies.at(0).WP.at(5).y = 100;
    A.at(2).policies.at(0).WP.at(5).z = -100;
    A.at(2).policies.at(0).WP.at(6).x = 100;
    A.at(2).policies.at(0).WP.at(6).y = 100;
    A.at(2).policies.at(0).WP.at(6).z = -400;
    
    //Set in agents along vehicles in water
    for (int rover_number = 0; rover_number<pPar->num_agents; rover_number++) {
        A.at(rover_number).start_generation(); // all policies have everything to zero
        A.at(rover_number).start_simulation(pPar);
        A.at(rover_number).select_fresh_policy();
        A.at(rover_number).V.start_based_on_policy(A.at(rover_number).policies.at(0), pPar->num_agents); //placed at first waypoint
    }
    
    //each iteration move in different path
    //for each time step
    for (int time_step = 0; time_step<A.at(0).policies.at(0).WP.size(); time_step++) {
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            int dex = pA->at(rover_number).active_policy_index;
            policy P = pA->at(rover_number).policies.at(dex);
            pA->at(rover_number).V.move_to_wp(P,time_step);
            
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).x == A.at(rover_number).V.x);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).y == A.at(rover_number).V.y);
            assert(A.at(rover_number).policies.at(0).WP.at(time_step).z == A.at(rover_number).V.z);
            
            if(VERBOSE){
                cout<<"Location of Agent::"<<rover_number<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
            }
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).observe_poi_distances(pE,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).establish_comms_links(pA,pPar);
        }
        for (int rover_number =0 ; rover_number<pPar->num_agents; rover_number++) {
            pA->at(rover_number).exchange_information_P2P(pA,pPar);
        }
    }
    
    //calculate the values
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    
    if(VERBOSE){
        for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
            cout<<"For rover::"<<rover_number<<endl;
            cout<<"fitness:"<<A.at(rover_number).policies.at(0).fitness<<endl;            //fitness;
            cout<<"local::"<<A.at(rover_number).policies.at(0).local<<endl;               //local;
            cout<<"true_global::"<<A.at(rover_number).policies.at(0).true_global<<endl; //true_global;
            cout<<"true_difference::"<<A.at(rover_number).policies.at(0).true_difference<<endl;  //true_difference;
            cout<<"limited_global::"<<A.at(rover_number).policies.at(0).limited_global<<endl;//limited_global;
            cout<<"limited_difference::"<<A.at(rover_number).policies.at(0).limited_difference<<endl; //limited_difference;
            cout<<endl;
            
        }
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).local);
    assert(A.at(1).policies.at(0).fitness == A.at(1).policies.at(0).local);
    assert(A.at(2).policies.at(0).fitness == A.at(2).policies.at(0).local);
    assert(A.at(0).policies.at(0).true_difference <= A.at(0).policies.at(0).limited_difference);
    assert(A.at(1).policies.at(0).true_difference <= A.at(1).policies.at(0).limited_difference);
    assert(A.at(2).policies.at(0).true_difference <= A.at(2).policies.at(0).limited_difference);
    assert(A.at(0).policies.at(0).true_global >= A.at(0).policies.at(0).limited_global);
    assert(A.at(1).policies.at(0).true_global >= A.at(1).policies.at(0).limited_global);
    assert(A.at(2).policies.at(0).true_global >= A.at(2).policies.at(0).limited_global);
}

/////// END TESTS FUNCTIONS ///////

/////// BGN OTHER FUNCTIONS ///////

/////// END OTHER FUNCTIONS ///////



///////////////////// %%%%%%%%%%%%%%%%%% END CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////

void stat_run(vector<agent>*pA,environment* pE,parameters* pPar,int SR, FILE* p_file,vector<double>* p_best_true_global);
void single_generation(vector<agent>*pA,environment* pE,parameters* pPar,int SR,int gen,FILE* p_file,vector<double>* p_best_true_global);
void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar, int gen,FILE* p_file);
void advance(vector<agent>*pA,environment* pE,parameters* pPar, int wpnum);

void stat_run(vector<agent>*pA,environment* pE,parameters* pPar, int SR, FILE* p_file,vector<double>* p_best_true_global){
    cout << "STAT RUN\t\t" << SR << endl;
    for(int gen = 0; gen<pPar->GENERATIONS; gen++){
        p_best_true_global->clear();
        for (int rover_number=0; rover_number < pPar->num_agents; rover_number++) {
            p_best_true_global->push_back(-999999.9999);
        }
        
        single_generation(pA,pE,pPar,SR,gen,p_file,p_best_true_global);
//        cout<<"best::"<<endl;
//        for (int rover_number =0 ; rover_number < p_best_true_global->size(); rover_number++) {
//            cout<<p_best_true_global->at(rover_number)<<"\t";
//        }
//        cout<<endl;
        for (int rover_number =0; rover_number<1; rover_number++) {
            fprintf(p_file,"%f \t ", p_best_true_global->at(rover_number));
        }
        p_best_true_global->clear();
    }
}

void single_generation(vector<agent>*pA,environment* pE,parameters* pPar, int SR, int gen,FILE* p_file,vector<double>* p_best_true_global){
    cout << "GENERATION\t" << SR << " :: " << gen << endl;
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).start_generation();           //acts as a reset for entire population
    }
    //controls the amount of simulatiosn ran in a generation by the amount of policies
    for(int sim=0; sim<pPar->pop_size; sim++){
        //cout << "-------------------------------------------------------------------" << endl;
        //cout << "simulation" << "\t" << sim << endl;
        /// select a policy for each agent.
        for(int a=0; a<pPar->num_agents; a++){
            pA->at(a).select_fresh_policy();
        }
        /// simulate based on selected policy.
        single_simulation(pA, pE, pPar,gen,p_file,p_best_true_global);
        
    }
    /// DOWNSELECT
    int rand1, rand2;
    double fit1, fit2;
    for(int a=0; a<pA->size(); a++){
        /// for each agent...
        /// policies / 2 times
        for(int reduce = 0; reduce<pPar->pop_size/2; reduce++){
            /// kill a policy by binary selection.
            rand1 = rand()%pA->at(a).policies.size();
            rand2 = rand()%pA->at(a).policies.size();
            while(rand2 == rand1){rand2 = rand()%pA->at(a).policies.size();}
            /// keep selecting rand2 until they are distinct.
            int stat_run_case = 5;
            switch (stat_run_case) {
                case 1:
                    fit1 = pA->at(a).policies.at(rand1).fitness;
                    fit2 = pA->at(a).policies.at(rand2).fitness;
                    break;
                case 2:
                    fit1 = pA->at(a).policies.at(rand1).limited_difference;
                    fit2 = pA->at(a).policies.at(rand2).limited_difference;
                    break;
                case 3:
                    fit1 = pA->at(a).policies.at(rand1).true_difference;
                    fit2 = pA->at(a).policies.at(rand2).true_difference;
                    break;
                case 4:
                    fit1 = pA->at(a).policies.at(rand1).true_global;
                    fit2 = pA->at(a).policies.at(rand2).true_global;
                    break;
                case 5:
                    fit1 = pA->at(a).policies.at(rand1).limited_global;
                    fit2 = pA->at(a).policies.at(rand2).limited_global;
                    break;
                default:
                    fit1 = pA->at(a).policies.at(rand1).fitness;
                    fit2 = pA->at(a).policies.at(rand2).fitness;
                    break;
            }
            //fit1 = pA->at(a).policies.at(rand1).fitness;
            //fit2 = pA->at(a).policies.at(rand2).fitness;
            if(fit1 > fit2){
                /// kill2
                pA->at(a).policies.erase(pA->at(a).policies.begin() + rand2);
            }
            else{
                /// kill1
                pA->at(a).policies.erase(pA->at(a).policies.begin() + rand1);
            }
        }
        /// REPOPULATE
        /// choose random survivors to repopulate.
        //cout << endl;
        //cout << "agent" << "\t" << a << endl;
        for(int repop = 0; repop<pPar->pop_size/2; repop++){
            //int spot = rand()%pA->size();     //we are looking at policies not agents
            int spot = rand()%pA->at(a).policies.size();
            //cout << "vec size" << "\t" << pA->at(a).policies.size() << endl;
            //pA->push_back(pA->at(spot));
            pA->at(a).policies.push_back(pA->at(a).policies.at(spot));
            //mutation should happen here
            //cout << spot << endl;
            //cout << endl;
            pA->at(a).policies.back().mutate(pPar);
            //cout << "in" << endl;
        }
        /// should be good to go for next generation at this point.
    }
}

void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar, int gen, FILE* p_file,vector<double>* p_best_true_global){
    for(int a=0; a<pPar->num_agents; a++){
        int dex = pA->at(a).active_policy_index;
        policy P = pA->at(a).policies.at(dex);
        //cout << "agent" << "\t" << a << endl;
        pA->at(a).V.start_based_on_policy(P,pPar->num_agents);
        pA->at(a).start_simulation(pPar);   //resets policy observation data
    }
    /// vehicles at starting position.
    for(int ts = 1; ts<pPar->num_waypoints; ts++){
        advance(pA,pE,pPar,ts);
    }
    /// vehicles have advanced through their paths and shared all available information with each other.
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).calc_local(pA,pE,pPar);
        pA->at(a).calc_true_global(pA,pE,pPar);
        pA->at(a).calc_true_difference(pA,pE,pPar);
        pA->at(a).calc_limited_global(pA,pE,pPar);
        pA->at(a).calc_limited_difference(pA,pE,pPar);
        pA->at(a).LGD_to_fitness(pA,pE,pPar);
    }
    
    for (int rover_number =0; rover_number < pPar->num_agents ; rover_number++) {
        int active = pA->at(rover_number).active_policy_index;
        if (pA->at(rover_number).policies.at(active).true_global > p_best_true_global->at(rover_number)) {
            p_best_true_global->at(rover_number) = pA->at(rover_number).policies.at(active).true_global;
        }
    }
    
    /*for (int rover_number = 0 ; rover_number<pPar->num_agents; rover_number++) {
        
        int active = pA->at(rover_number).active_policy_index;
        
        //cout<<"For rover::"<<rover_number<<endl;
        fprintf(p_file, "%d \t", gen);
        fprintf(p_file, "%d \t", rover_number);
        fprintf(p_file, "%d \t", active);
        
        //cout<<"fitness:"<<pA->at(rover_number).policies.at(active).fitness<<endl;            //fitness;
        fprintf(p_file, "%f \t",pA->at(rover_number).policies.at(active).fitness);
        
        //cout<<"local::"<<pA->at(rover_number).policies.at(active).local<<endl;               //local;
        fprintf(p_file, "%f \t",pA->at(rover_number).policies.at(active).local);
        
        //cout<<"true_global::"<<pA->at(rover_number).policies.at(active).true_global<<endl; //true_global;
        fprintf(p_file, "%f \t ",pA->at(rover_number).policies.at(active).true_global);
        
        //cout<<"true_difference::"<<pA->at(rover_number).policies.at(active).true_difference<<endl;  //true_difference;
        fprintf(p_file, "%f \t ",pA->at(rover_number).policies.at(active).true_difference);
        
        //cout<<"limited_global::"<<pA->at(rover_number).policies.at(active).limited_global<<endl;          //limited_global;
        fprintf(p_file, "%f \t",pA->at(rover_number).policies.at(active).limited_global);
        
        //cout<<"limited_difference::"<<pA->at(rover_number).policies.at(active).limited_difference<<endl;  //limited_difference;
        fprintf(p_file, "%f \t",pA->at(rover_number).policies.at(active).limited_difference);
        fprintf(p_file, "\n");
        //cout<<endl;
        
    }*/
    
    //fclose(p_file);
}

void advance(vector<agent>*pA,environment* pE,parameters* pPar, int wpnum){
    for(int a=0; a<pPar->num_agents; a++){
        int dex = pA->at(a).active_policy_index;
        policy P = pA->at(a).policies.at(dex);
        //cout << "agent" << "\t" << a << endl;
        pA->at(a).V.move_to_wp(P,wpnum);
    }
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).observe_poi_distances(pE,pPar);
    }
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).establish_comms_links(pA,pPar);
    }
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).exchange_information_P2P(pA,pPar);
    }
}

void print_parameters(parameters* pPar){
    FILE* p_file_parameter;
    p_file_parameter = fopen("parameter.txt", "a");
    fprintf(p_file_parameter, "Number of Agents :%d \n",pPar->num_agents);
    fprintf(p_file_parameter, "Number of POI's :%d \n",pPar->num_POI);
    fprintf(p_file_parameter, "Number of Way Points :%d \n",pPar->num_waypoints);
    fprintf(p_file_parameter, "Policy Size :%d \n",pPar->pop_size);
    fprintf(p_file_parameter,"Maximum Oberservation Distance :%f \n" ,pPar->maximum_observation_distance );
    fprintf(p_file_parameter, "P2P Communication Distance :%f \n",pPar->P2P_commlink_dist);
    fclose(p_file_parameter);
}

int main() {
    srand((unsigned)time(NULL));
    cout << "Start of Program" << endl;
    
    //tests T;
    //T.init();
    //T.single_agent_multi_poi();
    //return 0;
    if (test_functions) {
        tests T_obj;
        T_obj.init();
        T_obj.single_agent_multi_poi();
        T_obj.single_agent_check_waypoints_poi();
        T_obj.multi_agent_different_path();
        T_obj.multi_agent_same_path_no_joy();   //no communication
        T_obj.multi_agent_same_path_with_joy();    //communication
        T_obj.multi_agent_different_path_different_poi(); //They travel different path and look at different POI's
        T_obj.communication_test(); //They travel different path and look at different POI's
        T_obj.three_agents_limited_joy();
        T_obj.three_rover_not_surfacing();
    }
    
    if(!test_functions){
        cout<<"Start Function!!"<<endl;
        parameters P;
        parameters* pPar = &P;
        
        string date_time_stamp = "datetime";
        FILE* p_file;
        p_file = fopen("stat.txt", "a");
        
        print_parameters(pPar);
        for(int i=0; i<pPar->STAT_RUNS; i++){
            reset_id = true;
            
            vector<agent> Team;
            vector<agent>* pA = &Team;
            for(int a=0; a<pPar->num_agents; a++){
                agent AA;
                AA.init(pPar);
                Team.push_back(AA);
            }
            assert(Team.size() == pPar->num_agents);
            
            environment E;
            environment* pE = &E;
            E.init(pPar);
            
            vector<double> best_true_global;
            vector<double>* p_best_true_global = &best_true_global;
            for (int rover_number=0; rover_number < pPar->num_agents; rover_number++) {
                best_true_global.push_back(-999999.9999);
            }
            
            //// Assume team, environment have been initialized 10/19/16
            stat_run(pA,pE,pPar,i,p_file,p_best_true_global);
            fprintf(p_file, "\n");
        }
        fclose(p_file);
    }
    
    cout << "End of Program" << endl;
    //}
    return 0;
}

