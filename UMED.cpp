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

/// max function
double max(double a, double b){
    if(a>b){return a;}
    return b;
}

bool test_functions = true;
bool ak = true;

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

void single_generation(vector<agent>*pA,environment* pE,parameters* pPar, int SR, int gen);
void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar);
void stat_run(vector<agent>*pA,environment* pE,parameters* pPar, int SR);
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
    void mutate(parameters* pPar,bool surf);
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
    int num_agents = 3;
    int num_vehicles = num_agents; // 1 vehicle per agent
    int num_POI = 5*num_agents;
    int pop_size = 100;
    int num_waypoints = 10;
    
    double mutation_size = 5.0;
    
    double max_x = 100;
    double max_y = 100;
    double max_z = 10;
    
    double min_x = 0;
    double min_y = 0;
    double min_z = -100;
    
    double min_poi_value = 1;
    double max_poi_value = 100;
    
    int STAT_RUNS = 3;
    int GENERATIONS = 100;
    bool allow_general_comm_link = true;
    
    double P2P_commlink_dist = 300;
    double maximum_observation_distance = 100;
    
    void init();
    void single_agent_test_overwrite();
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
    
    /// give each agent a unique ID;
    static int x;
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
    for(int p=0; p<pPar->num_POI; p++){
        double value = pE->POIs.at(p).val;
        double delta = max(1.0,my_observations.at(p).observation_distance);
        double contribution = value/delta;
        policies.at(active_policy_index).local+=contribution;
    }
    return;
}
void agent::calc_true_global(vector<agent>* pA, environment* pE, parameters* pPar){
    policies.at(active_policy_index).true_global = 0;
    /// LYLY Note that this is current calculated "agents" times, and does not strictly need to be; the true_global could be calculated once and broadcast.
    double dis;
    double g=0;
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
    double limg;
    
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
    policies.at(active_policy_index).true_difference = 0;
    double true_cf = 0;
    /// TODO use all agent's "my observations"
    
    vector<double> closest(pPar->num_POI,std::numeric_limits<double>::max());
    double dis;
    for(int a=0; a<pPar->num_agents; a++){
        if(a==id){
            continue;
        }
        for(int p=0; p<pPar->num_agents; p++){
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
    
    /// calculated limg based on the closest of all of these.
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



/////// BGN OTHER FUNCTIONS ///////

/////// END OTHER FUNCTIONS ///////



///////////////////// %%%%%%%%%%%%%%%%%% END CLASS FUNCTIONS %%%%%%%%%%%%%%%%%% /////////////////////

void stat_run(vector<agent>*pA,environment* pE,parameters* pPar);
void single_generation(vector<agent>*pA,environment* pE,parameters* pPar,int SR,int gen);
void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar);
void advance(vector<agent>*pA,environment* pE,parameters* pPar, int wpnum);

void stat_run(vector<agent>*pA,environment* pE,parameters* pPar, int SR){
    cout << "STAT RUN\t\t" << SR << endl;
    for(int gen = 0; gen<pPar->GENERATIONS; gen++){
        single_generation(pA,pE,pPar,SR,gen);
    }
}

void single_generation(vector<agent>*pA,environment* pE,parameters* pPar, int SR, int gen){
    cout << "GENERATION\t" << SR << " :: " << gen << endl;
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).start_generation();
    }
    for(int sim=0; sim<pPar->pop_size; sim++){
        /// select a policy for each agent.
        for(int a=0; a<pPar->num_agents; a++){
            pA->at(a).select_fresh_policy();
        }
        /// simulate based on selected policy.
        single_simulation(pA, pE, pPar);
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
            fit1 = pA->at(a).policies.at(rand1).fitness;
            fit2 = pA->at(a).policies.at(rand2).fitness;
            if(fit1 > fit2){
                /// kill2
                pA->erase(pA->begin()+rand2);
            }
            else{
                /// kill1
                pA->erase(pA->begin()+rand1);
            }
        }
    }
    /// REPOPULATE
    /// choose random survivors to repopulate.
    for(int repop = 0; repop<pPar->pop_size/2; repop++){
        int spot = rand()%pA->size();
        pA->push_back(pA->at(spot));
    }
    /// should be good to go for next generation at this point.
}

void single_simulation(vector<agent>*pA,environment* pE,parameters* pPar){
    for(int a=0; a<pPar->num_agents; a++){
        int dex = pA->at(a).active_policy_index;
        policy P = pA->at(a).policies.at(dex);
        pA->at(a).V.start_based_on_policy(P,pPar->num_agents);
        pA->at(a).start_simulation(pPar);
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
}

void advance(vector<agent>*pA,environment* pE,parameters* pPar, int wpnum){
    for(int a=0; a<pPar->num_agents; a++){
        int dex = pA->at(a).active_policy_index;
        policy P = pA->at(a).policies.at(dex);
        pA->at(a).V.move_to_wp(P,wpnum);
    }
    for(int a=0; a<pPar->num_agents; a++){
        pA->at(a).observe_poi_distances(pE,pPar);
        pA->at(a).establish_comms_links(pA,pPar);
        pA->at(a).exchange_information_P2P(pA,pPar);
    }
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
//        T_obj.init();
//        T_obj.single_agent_multi_poi();
//        T_obj.single_agent_check_waypoints_poi();
//        T_obj.multi_agent_different_path();
//        T_obj.multi_agent_same_path_no_joy();   //no communication
//        T_obj.multi_agent_same_path_with_joy();    //communication
        T_obj.multi_agent_different_path_different_poi(); //They travel different path and look at different POI's
    }
    
    if(!test_functions){
        parameters P;
        parameters* pPar = &P;
        
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
        
        //// Assume team, environment have been initialized 10/19/16
        
        for(int i=0; i<pPar->STAT_RUNS; i++){
            stat_run(pA,pE,pPar,i);
        }
        
        cout << "End of Program" << endl;
    }
    return 0;
}

