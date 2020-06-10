#include "tests/mlsa/model_tests.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/mlsa_models/cplex/flow_model.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>
#include <fstream>

using namespace std;
using namespace utils;
using namespace tests::mlsa_flow;
using namespace models;
using namespace models::gvrp_models;

Model_tests::Model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Model_tests::run() {
  //experiments
    //setup
  string solution_name = "mlsa_flow_";
  ofstream resultsFile;
  string instance_part;
  vector<vector<int>> routes;
  resultsFile.open (solution_name + string("results.csv"));
  resultsFile<<"Instance,Solution,C,F,Beta,T,C service time, F service time,GAP,Cost,Time,Status"<<endl;
  list<Vrp_instance> vrp_instances;
  int i = 0;
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("UchoaEtAl/"));
    //instance list
  for (const string& instance : instances){
    Vrp_instance vrp_instance = read_uchoa_vrp_instance(PROJECT_INSTANCES_PATH + string("UchoaEtAl/") + instance);
    vrp_instances.push_back(vrp_instance);
  }
    //executions
  auto vrp_instance = vrp_instances.begin();
  for (const string& instance : instances) {
    cout<<instance<<endl;
    //get vrp sol
    stringstream ss (instance);
    getline(ss, instance_part, '.');
    instance_part = PROJECT_SOLUTIONS_PATH + string("UchoaEtAl/") + instance_part + ".sol";
    routes = read_uchoa_vrp_solution (instance_part);
    //run
//    Flow_model flow_model (*vrp_instance, execution_time, calculateVRPSolutionCost(routes, *vrp_instance)/2);  
//    Flow_model flow_model (*vrp_instance, execution_time, calculateRouteAverageCost(routes, *vrp_instance));  
    double longestEdge = 0.0;
    for (size_t i = 0; i < vrp_instance->customers.size() + 1; ++i)
      for (size_t j = 0; j < vrp_instance->customers.size() + 1; ++j)
        longestEdge = max(vrp_instance->distances[i][j], longestEdge);
    Flow_model flow_model (*vrp_instance, execution_time, longestEdge/2);  
    try {
      flow_model.max_num_feasible_integer_sol = nIntSol;
      flow_model.VERBOSE = VERBOSE;
      pair<Gvrp_instance, Mip_solution_info > sol = flow_model.run();    
      string sol_file_name = solution_name + to_string(sol.first.customers.size()) + string ("c") + to_string(sol.first.afss.size()) + string ("f") + to_string(flow_model.vehicleFuelCapacity) + string("B-") + instance;
      sol.first.write_in_csv(PROJECT_INSTANCES_PATH + string("new/") + sol_file_name);
      resultsFile<<instance<<";"<<sol_file_name<<";"<<sol.first.customers.size()<<";"<<sol.first.afss.size()<<";"<<sol.first.vehicleFuelCapacity<<";"<<sol.first.timeLimit<<";"<<sol.first.customers.front().serviceTime<<";"<<sol.first.afss.front().serviceTime<<";"<<sol.second.gap<<";"<<int(sol.second.cost)<<"."<<int(sol.second.cost*100)%100<<";"<<sol.second.elapsed_time<<";"<<sol.second.status<<endl;
    } catch (string s){
      cout<<"Error:"<<s;
    } catch (const Mip_solution_info& excSolInfo){
      resultsFile<<instance<<";-;-;-;-;-;-;-;"<<excSolInfo.gap<<";"<<excSolInfo.cost<<";"<<excSolInfo.elapsed_time<<";"<<excSolInfo.status<<endl;
    } catch (...) {
      cout<<"Another error"<<endl;
    }
    vrp_instance++;
    i++;
  }
  resultsFile.close();
  resultsFile.clear();
}
