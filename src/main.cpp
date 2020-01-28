#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/gvrp_feasible_solution_heuristic.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/util.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/mip_start_compact_model.hpp"
#include "utils/cplex/improved_compact_model.hpp"
#include "utils/cplex/subcycle_user_constraint_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing.hpp"
#include "utils/cplex/max_afs_visit_constraint.hpp"
#include "utils/cplex/max_distance_route_constraint.hpp"
#include "utils/cplex/min_distance_route_constraint.hpp"
#include "utils/cplex/energy_lb_constraint.hpp"
#include "utils/cplex/energy_ub_constraint.hpp"
#include "utils/cplex/energy_lifting_constraint.hpp"
#include "utils/cplex/routes_order_constraint.hpp"
#include "SampleConfig.h"

#include <string>
#include <iomanip>
#include <set>
#include <queue>
#include <map>
#include <typeinfo>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;


void execute_model(Compact_model& compact_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);

int main (int argc, char **argv)
{ 
  unsigned int execution_time = 120,
               nIntSol = -1;
  bool VERBOSE = false;  
  //getting params
  for (int i = 0; i < argc; i++)
    if (strcmp(argv[i], "-time") == 0)
      execution_time = stoi(argv[++i]);
    else if (strcmp(argv[i], "-verbose") == 0) {
      if (strcmp(argv[++i], "true") == 0 ||  strcmp(argv[i], "1") == 0) {
        VERBOSE = true; 
      }
    } else if (strcmp(argv[i], "-nIntSol") == 0) {
      nIntSol = stoi(argv[++i]);
      if (nIntSol <= 0)
        nIntSol = 2100000000;
    }
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //end results file
  ofstream resultsFile;
  resultsFile.open ("results.csv");
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<Gvrp_instance> gvrp_instances;
  vector<double> lambdas (instances.size());
  int i = 0;
  for (const string& instance : instances){
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    gvrp_instances.push_back(gvrp_instance);
    lambdas[i] = calculateGvrpInstanceLambdaFactor (gvrp_instance);
    i++;
  }
    //executions
      //user constraints
  solution_name = "subcycle_user_constraint_";
  i = 0;
  auto gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.user_constraints.push_back(new Subcycle_user_constraint_compact_model(compact_model));
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
    //preprocessings
  solution_name = "invalid_edge_preprocesing_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.preprocessings.push_back(new Invalid_edge_preprocessing(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
    //extra constraints
  solution_name = "max_distance_route_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Max_distance_route_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "max_afs_visit_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Max_afs_visit_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "min_distance_route_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Min_distance_route_constraint(compact_model, lambdas[i]));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "energy_lb_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Energy_lb_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "energy_ub_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Energy_ub_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "energy_lifting_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Energy_lifting_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "routes_order_constraint_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.extra_constraints.push_back(new Routes_order_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  solution_name = "all_";
  i = 0;
  gvrp_instance = gvrp_instances.begin();
  for (const string& instance : instances) {
    Compact_model compact_model (*gvrp_instance, execution_time);  
    compact_model.user_constraints.push_back(new Subcycle_user_constraint_compact_model(compact_model));
    compact_model.preprocessings.push_back(new Invalid_edge_preprocessing(compact_model));
    compact_model.extra_constraints.push_back(new Max_distance_route_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Max_afs_visit_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Min_distance_route_constraint(compact_model, lambdas[i]));
    compact_model.extra_constraints.push_back(new Energy_lb_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Energy_ub_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Energy_lifting_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Routes_order_constraint(compact_model));
;
    execute_model(compact_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  //write csv
  resultsFile.close(); 
  return 0;
}

void execute_model(Compact_model& compact_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    compact_model.max_num_feasible_integer_sol = nIntSol;
    compact_model.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = compact_model.run();    
    sol.first.write_in_file(PROJECT_INSTANCES_PATH + string("solutions/EMH/") + string(prefix_solution_files)  + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  }
}
