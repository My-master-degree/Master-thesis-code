#include "tests/gvrp/local_searchs_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/local_search_strategy_enum.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/local_searchs/merge.hpp"
#include "models/gvrp_models/local_searchs/swap.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"

#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

void Local_searchs_tests::run() {
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
  solution_name = "local_searchs_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    //get solution
    Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (*gvrp_instance);
    Gvrp_solution gvrp_solution = gvrp_feasible_solution_heuristic.run();
    //local searchs
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    Merge merge (*gvrp_instance, gvrp_solution, BEST_IMPROVEMENT);  
    Gvrp_solution newSolution = merge.run();    
    Swap swap (*gvrp_instance, newSolution, BEST_IMPROVEMENT);  
    //newSolution = swap.run();    
    cout<<newSolution<<endl;
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    double cost = newSolution.calculateCost();
    //write
    newSolution.write_in_file(PROJECT_SOLUTIONS_PATH + string("LS/ls_") + instance);
    resultsFile<<instance<<";"<<solution_name<<";"<<int(cost)<<"."<<int(cost*100)%100<<";"<<elapsed<<endl;
    gvrp_instance++;
    i++;
    break;
  }
  closeResultFile(resultsFile);
}

void Local_searchs_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("kk_results.csv"));
  resultsFile<<"Instance,Solution,Cost,Time"<<endl;
}

void Local_searchs_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
