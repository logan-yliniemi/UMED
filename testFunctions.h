//
//  testFunctions.h
//  underwaterDomain
//
//  Created by ak on 11/8/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef testFunctions_h
#define testFunctions_h
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
};

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
    
    single_simulation(pA,pE,pPar);
    
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
    
    single_simulation(pA,pE,pPar);
    
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
    
    single_simulation(pA,pE,pPar);
    
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
            cout<<"Location of POI::"<<endl;
            cout<<E.POIs.at(0).x<<"\t"<<E.POIs.at(0).y<<"\t"<<E.POIs.at(0).z<<endl;
            cout<<E.POIs.at(1).x<<"\t"<<E.POIs.at(1).y<<"\t"<<E.POIs.at(1).z<<endl;
        }
        
        pA->at(0).observe_poi_distances(pE,pPar);
        pA->at(0).establish_comms_links(pA,pPar);
        pA->at(0).exchange_information_P2P(pA,pPar);
        
        if(VERBOSE){
            cout<<A.at(0).my_observations.at(0).observation_distance<<endl;
            cout<<A.at(0).my_observations.at(1).observation_distance<<endl;
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
        cout<<"fitness:"<<A.at(0).policies.at(0).fitness;                         //fitness;
        cout<<"local::"<<A.at(0).policies.at(0).local;                            //local;
        cout<<"true_global::"<<A.at(0).policies.at(0).true_global;                //true_global;
        cout<<"true_difference::"<<A.at(0).policies.at(0).true_difference;            //true_difference;
        cout<<"limited_global::"<<A.at(0).policies.at(0).limited_global;          //limited_global;
        cout<<"limited_difference::"<<A.at(0).policies.at(0).limited_difference;  //limited_difference;
    }
    
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).local);
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).limited_global );
    assert(A.at(0).policies.at(0).fitness == A.at(0).policies.at(0).limited_difference);
    
    assert(A.at(0).policies.at(0).local == A.at(0).policies.at(0).true_global);
    assert(A.at(0).policies.at(0).local == A.at(0).policies.at(0).true_difference);
    assert(A.at(0).policies.at(0).local == A.at(0).policies.at(0).limited_global);
    assert(A.at(0).policies.at(0).local== A.at(0).policies.at(0).limited_difference);
    
    assert(A.at(0).policies.at(0).true_difference == A.at(0).policies.at(0).limited_global);
    assert(A.at(0).policies.at(0).true_difference == A.at(0).policies.at(0).limited_difference);
    
    assert(A.at(0).policies.at(0).limited_global == A.at(0).policies.at(0).limited_difference);
    
    //
}

void tests::multi_agent_different_path(){
    bool VERBOSE = false;
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 2;
    pPar->num_waypoints =4;
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
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 100;
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
                cout<<"Location of Agent::"<<endl;
                cout<<A.at(rover_number).V.x<<"\t"<<A.at(rover_number).V.y<<"\t"<<A.at(rover_number).V.z<<endl;
                cout<<"Location of POI::"<<endl;
                cout<<E.POIs.at(0).x<<"\t"<<E.POIs.at(0).y<<"\t"<<E.POIs.at(0).z<<endl;
                cout<<E.POIs.at(1).x<<"\t"<<E.POIs.at(1).y<<"\t"<<E.POIs.at(1).z<<endl;
            }
            pA->at(rover_number).observe_poi_distances(pE,pPar);
            pA->at(rover_number).establish_comms_links(pA,pPar);
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

void tests::multi_agent_same_path_no_joy(){
    bool VERBOSE = false;
    //Set all the values
    pPar->num_agents = 2;
    pPar->num_vehicles = 2;
    pPar->pop_size = 1;
    pPar->num_POI = 2;
    pPar->num_waypoints =4;
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
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 100;
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
                //                cout<<"Location of POI::"<<endl;
                //                cout<<E.POIs.at(0).x<<"\t"<<E.POIs.at(0).y<<"\t"<<E.POIs.at(0).z<<endl;
                //                cout<<E.POIs.at(1).x<<"\t"<<E.POIs.at(1).y<<"\t"<<E.POIs.at(1).z<<endl;
            }
            pA->at(rover_number).observe_poi_distances(pE,pPar);
            pA->at(rover_number).establish_comms_links(pA,pPar);
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


void tests::multi_agent_same_path_with_joy(){
    bool VERBOSE = true;
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
    E.POIs.at(1).z = 100;
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
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 100;
        
        (temp_rover_number>0)?waypoint_location--:waypoint_location++;
        
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).x = 120;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).y = 120;
        A.at(temp_rover_number).policies.at(0).WP.at(waypoint_location).z = 120;
        
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
                //                cout<<"Location of POI::"<<endl;
                //                cout<<E.POIs.at(0).x<<"\t"<<E.POIs.at(0).y<<"\t"<<E.POIs.at(0).z<<endl;
                //                cout<<E.POIs.at(1).x<<"\t"<<E.POIs.at(1).y<<"\t"<<E.POIs.at(1).z<<endl;
            }
            pA->at(rover_number).observe_poi_distances(pE,pPar);
            pA->at(rover_number).establish_comms_links(pA,pPar);
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
    
}

void tests::multi_agent_different_path_different_poi(){
    
}


/////// END TESTS FUNCTIONS ///////

#endif /* testFunctions_h */
