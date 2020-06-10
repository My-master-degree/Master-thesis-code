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
  solution_name = "mlsa_flow_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    //get vrp sol
    stringstream ss (instance);
    getline(ss, instance_part, '.');
    instance_part = PROJECT_SOLUTIONS_PATH + string("UchoaEtAl/") + instance_part + ".sol";
    /*
    //custom reading
    ifstream inFile;
    //read file
    inFile.open(instance_part);
    //setup
    string buff, 
           line;
    stringstream ss_;
    vector<vector<int>> routes;
    //get routes
    while (getline(inFile, line)) {
      ss_.str(line);
      ss_>>buff;
      if (buff == "Cost" || buff == "cost")
        break;
      ss_>>buff;
      vector<int> route;
      for (ss_>>buff; !ss_.eof(); ss_>>buff) 
        route.push_back(stoi(buff, NULL));
      if (route.size() == 0 || route.back() != stoi(buff, NULL)) 
        route.push_back(stoi(buff, NULL));
      routes.push_back(route);
      ss_.clear();
    }
    inFile.close();
    //create new output
    ofstream solutionFile;
    solutionFile.open (PROJECT_SOLUTIONS_PATH + ori + ".sol");
    solutionFile <<endl<<routes.size()<<endl<<endl<<endl;
    for (const vector<int>& route : routes) {
      solutionFile<<"0 1 1 - - "<<route.size() + 2<<" 0 ";
      for (int n : route)
        solutionFile<<n<<" ";
      solutionFile<<"0"<<endl;
    }
    solutionFile.close(); 
    */
    routes = read_uchoa_vrp_solution (instance_part);
    //run
//    Flow_model flow_model (*vrp_instance, execution_time, calculateVRPSolutionCost(routes, *vrp_instance)/2);  
    Flow_model flow_model (*vrp_instance, execution_time, calculateRouteAverageCost(routes, *vrp_instance));  
    execute_model(flow_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    vrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
}

void Model_tests::execute_model(Flow_model& flow_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    flow_model.max_num_feasible_integer_sol = nIntSol;
    flow_model.VERBOSE = VERBOSE;
    pair<Gvrp_instance, Mip_solution_info > sol = flow_model.run();    
    sol.first.write_in_csv(PROJECT_INSTANCES_PATH + string("new/") + string(prefix_solution_files)  + to_string(sol.first.customers.size()) + string ("c") + to_string(sol.first.afss.size()) + string ("f") + to_string(flow_model.vehicleFuelCapacity) + string("B-") + instance_name);
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
