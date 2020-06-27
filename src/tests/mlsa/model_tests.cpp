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
#include <ilcplex/ilocplex.h>

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
  double afssRatio[] = {0.10, 0.20, 0.30};
  ofstream resultsFile;
  string instance_part;
  vector<vector<int>> routes;
  resultsFile.open (solution_name + string("results.csv"));
  resultsFile<<"Instance;Solution;C;F;Beta;T;C service time; F service time;GAP;Cost;Time;Status;Ratio"<<endl;
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
    for (double ratio : afssRatio) {
      Mip_solution_info mip_solution_info;
      Gvrp_instance * gvrp_instance = nullptr;
      double vehicleFuelCapacity = 0.0;
      //bs
      for (double l = 0.0, r = longestEdge, m = r/2.0; l <= r; m = l + (r - l)/2.0) {
        int afssLB = vrp_instance->customers.size() * ratio;
        try {
          Flow_model flow_model (*vrp_instance, execution_time, m);  
          flow_model.max_num_feasible_integer_sol = nIntSol;
          flow_model.VERBOSE = VERBOSE;
          pair<Gvrp_instance, Mip_solution_info > sol = flow_model.run();    
          if (sol.first.afss.size() >= afssLB) {
            gvrp_instance = new Gvrp_instance (sol.first);
            mip_solution_info = sol.second;
            vehicleFuelCapacity = m;
            if (sol.first.afss.size() == afssLB)
              break;
            l = m + 1.0;
          } else 
            r = m - 1.0;
        } catch (string s){
          cout<<"Error:"<<s;
        } catch (...) {
          cout<<"Another error"<<endl;
        }
      }
      //write in file
      if (gvrp_instance == nullptr)
          resultsFile<<instance<<";-;-;-;-;-;-;-;"<<mip_solution_info.gap<<";"<<mip_solution_info.cost<<";"<<mip_solution_info.elapsed_time<<";"<<mip_solution_info.status<<";"<<ratio<<endl;
      else {
        string sol_file_name = solution_name + to_string(gvrp_instance->customers.size()) + string ("c") + to_string(gvrp_instance->afss.size()) + string ("f") + to_string(vehicleFuelCapacity) + string("B-") + instance;
        gvrp_instance->write_in_csv(PROJECT_INSTANCES_PATH + string("new/") + sol_file_name);
        resultsFile<<instance<<";"<<sol_file_name<<";"<<gvrp_instance->customers.size()<<";"<<gvrp_instance->afss.size()<<";"<<gvrp_instance->vehicleFuelCapacity<<";"<<gvrp_instance->timeLimit<<";"<<gvrp_instance->customers.front().serviceTime<<";"<<gvrp_instance->afss.front().serviceTime<<";"<<mip_solution_info.gap<<";"<<int(mip_solution_info.cost)<<"."<<int(mip_solution_info.cost*100)%100<<";"<<mip_solution_info.elapsed_time<<";"<<mip_solution_info.status<<";"<<ratio<<endl;
        delete gvrp_instance;
      }
    }
    vrp_instance++;
    i++;
    break;
  }
  resultsFile.close();
  resultsFile.clear();
}
