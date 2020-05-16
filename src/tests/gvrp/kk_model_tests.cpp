#include "tests/gvrp/kk_model_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/kk_model/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/cplex/kk_model/subcycle_user_constraint.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::kk_model;

KK_model_tests::KK_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void KK_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<Gvrp_instance> gvrp_instances;
  int i = 0;
  for (const string& instance : instances){
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    gvrp_instances.push_back(gvrp_instance);
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "kk_model_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    cout<<*gvrp_instance<<endl;
    KK_model kk_model (*gvrp_instance, execution_time);  
    /*
    kk_model.preprocessings.push_back(new Invalid_edge_preprocessing(kk_model));
    kk_model.preprocessings.push_back(new Invalid_edge_preprocessing_2(kk_model));
    kk_model.preprocessings.push_back(new Invalid_edge_preprocessing_3(kk_model));
    kk_model.preprocessings.push_back(new Invalid_edge_preprocessing_4(kk_model));
    kk_model.user_constraints.push_back(new Subcycle_user_constraint(kk_model));
    */
    execute_model(kk_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
}

void KK_model_tests::execute_model(KK_model& kk_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    kk_model.max_num_feasible_integer_sol = nIntSol;
    kk_model.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = kk_model.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("KK/") + string(prefix_solution_files) + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void KK_model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("kk_results.csv"));
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
}

void KK_model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
