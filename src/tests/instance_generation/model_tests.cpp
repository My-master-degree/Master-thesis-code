#include "tests/instance_generation/model_tests.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/instance_generation_model/instance_generation_model.hpp"
#include "models/instance_generation_model/subcycle_user_constraint.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>

using namespace std;
using namespace utils;
using namespace tests::instance_generation;
using namespace models;
using namespace models::gvrp_models;

Model_tests::Model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

double calculateVRPSolutionCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance) {
  const size_t sroutes = routes.size();
  double cost = 0;
  for (size_t i = 0; i < sroutes; ++i) 
    for (size_t j = 1; j < routes[i].size(); ++j) 
      cost += vrp_instance.distances[routes[i][j]][routes[i][j - 1]];
  return cost;
}

double calculateLargestRouteCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance) {
  const size_t sroutes = routes.size();
  double largestCost = 0;
  for (size_t i = 0; i < sroutes; ++i) {
    double cost = 0;
    for (size_t j = 1; j < routes[i].size(); ++j) 
      cost += vrp_instance.distances[routes[i][j]][routes[i][j - 1]];
    largestCost = max(largestCost, cost);
  }
  return largestCost;
}

double calculateRouteAverageCost (const vector<vector<int>>& routes, const Vrp_instance& vrp_instance) {
  return calculateVRPSolutionCost(routes, vrp_instance) / routes.size();
}


void Model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("UchoaEtAl/"));
  list<Vrp_instance> vrp_instances;
  int i = 0;
  for (const string& instance : instances){
    Vrp_instance vrp_instance = read_uchoa_vrp_instance(PROJECT_INSTANCES_PATH + string("UchoaEtAl/") + instance);
    vrp_instances.push_back(vrp_instance);
  }
    //executions
  auto vrp_instance = vrp_instances.begin();
  ofstream resultsFile;
  string instance_part;
  vector<vector<int>> routes;
      //model only
  solution_name = "instance_generation_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    //get vrp sol
    stringstream ss (instance);
    getline(ss, instance_part, '.');
    instance_part = PROJECT_SOLUTIONS_PATH + string("UchoaEtAl/") + instance_part + ".sol";
    routes = read_uchoa_vrp_solution (instance_part);
    //run
    Instance_generation_model instance_generation_model (*vrp_instance, calculateVRPSolutionCost(routes, *vrp_instance)/2, execution_time);  
    instance_generation_model.user_constraints.push_back(new Subcycle_user_constraint(instance_generation_model));
    execute_model(instance_generation_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    vrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
}

void Model_tests::execute_model(Instance_generation_model& instance_generation_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    instance_generation_model.max_num_feasible_integer_sol = nIntSol;
    instance_generation_model.VERBOSE = VERBOSE;
    pair<Gvrp_instance, Mip_solution_info > sol = instance_generation_model.run();    
    sol.first.write_in_csv(PROJECT_INSTANCES_PATH + string("new/") + string(prefix_solution_files)  + to_string(sol.first.customers.size()) + string ("c") + to_string(sol.first.afss.size()) + string ("f-") + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void Model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("results.csv"));
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
}

void Model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
