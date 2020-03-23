#include "tests/gvrp/cubic_model_tests.hpp"
#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/mip_solution_info.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/mip_start_cubic_model.hpp"
#include "models/cubic_model/improved_cubic_model.hpp"
#include "models/cubic_model/subcycle_user_constraint_cubic_model.hpp"
#include "models/cubic_model/invalid_edge_preprocessing.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_3.hpp"
#include "models/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "models/cubic_model/no_consecutive_afs_visit_preprocessing.hpp"
#include "models/cubic_model/max_afs_visit_constraint.hpp"
#include "models/cubic_model/max_distance_route_constraint.hpp"
#include "models/cubic_model/min_distance_route_constraint.hpp"
#include "models/cubic_model/energy_lb_constraint.hpp"
#include "models/cubic_model/energy_ub_constraint.hpp"
#include "models/cubic_model/energy_lifting_constraint.hpp"
#include "models/cubic_model/routes_order_constraint.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <list>
#include <vector>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models;

Cubic_model_tests::Cubic_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Cubic_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<Gvrp_instance> gvrp_instances;
  vector<double> lambdas (instances.size());
  int i = 0;
  for (const string& instance : instances){
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    gvrp_instances.push_back(gvrp_instance); lambdas[i] = calculateGvrpInstanceLambdaFactor (gvrp_instance); i++; }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "model_only_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
      //user constraints
  solution_name = "subcycle_user_constraint_";
  cout<<solution_name<<endl;
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    cout<<instance<<endl;
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.user_constraints.push_back(new Subcycle_user_constraint_cubic_model(cubic_model));
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
    //preprocessings
      //invalid edge preprocessing 
  solution_name = "invalid_edge_preprocesing_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
      //invalid edge preprocessing 2
  solution_name = "invalid_edge_preprocesing_2";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing_2(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
    break;
  }
  closeResultFile(resultsFile);
    //extra constraints
  solution_name = "max_distance_route_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Max_distance_route_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "max_afs_visit_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Max_afs_visit_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "min_distance_route_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Min_distance_route_constraint(cubic_model, lambdas[i]));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "energy_lb_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Energy_lb_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "energy_ub_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Energy_ub_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "energy_lifting_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Energy_lifting_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "routes_order_constraint_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    cubic_model.extra_constraints.push_back(new Routes_order_constraint(cubic_model));
;
    execute_model(cubic_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
  solution_name = "all_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    cout<<instance<<endl;
    Cubic_model cubic_model (*gvrp_instance, execution_time);  
    Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (*gvrp_instance);
    Gvrp_solution gvrp_solution = gvrp_feasible_solution_heuristic.run();
//    Mip_start_cubic_model cubic_model (*gvrp_instance, execution_time, gvrp_solution);  
    //frac cuts
    cubic_model.user_constraints.push_back(new Subcycle_user_constraint_cubic_model(cubic_model));
    //preprocessings
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing(cubic_model));
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing_2(cubic_model));
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing_3(cubic_model));
    cubic_model.preprocessings.push_back(new Invalid_edge_preprocessing_4(cubic_model));
    cubic_model.preprocessings.push_back(new No_consecutive_afs_visit_preprocessing(cubic_model));
    //extra constraints
    cubic_model.extra_constraints.push_back(new Max_distance_route_constraint(cubic_model));
    cubic_model.extra_constraints.push_back(new Max_afs_visit_constraint(cubic_model));
    cubic_model.extra_constraints.push_back(new Min_distance_route_constraint(cubic_model, lambdas[i]));
    cubic_model.extra_constraints.push_back(new Energy_lb_constraint(cubic_model));
    cubic_model.extra_constraints.push_back(new Energy_ub_constraint(cubic_model));
    cubic_model.extra_constraints.push_back(new Energy_lifting_constraint(cubic_model));
    cubic_model.extra_constraints.push_back(new Routes_order_constraint(cubic_model));
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
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("EMH/") + string(prefix_solution_files)  + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
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
