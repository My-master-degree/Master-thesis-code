#include "tests/gvrp/cubic_model_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/mip_start.hpp"
#include "models/gvrp_models/cplex/cubic_model/improved_cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/cplex/cubic_model/no_consecutive_afs_visit_preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/max_afs_visit_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/max_distance_route_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/min_distance_route_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/energy_lb_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/energy_ub_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/energy_lifting_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/routes_order_constraint.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <vector>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;

Cubic_model_tests::Cubic_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Cubic_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
//  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("new/consec/"));
  list<Gvrp_instance> gvrp_instances;
  vector<double> lambdas (instances.size());
  int i = 0;
  for (const string& instance : instances){
//    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    Gvrp_instance gvrp_instance = matheus_instance_reader(PROJECT_INSTANCES_PATH + string("new/consec/") + instance);
    gvrp_instances.push_back(gvrp_instance); 
    i++; 
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "cubic_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    //keep this line, for some strange bug it is necessary to define the time limit explicitly 
    cubic_model.time_limit = execution_time;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
    break;
  }
  closeResultFile(resultsFile);
}

void Cubic_model_tests::execute_model(Cubic_model& cubic_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    cubic_model.max_num_feasible_integer_sol = nIntSol;
    cubic_model.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = cubic_model.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("Cubic/") + string(prefix_solution_files)  + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void Cubic_model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("results.csv"));
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
}

void Cubic_model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
