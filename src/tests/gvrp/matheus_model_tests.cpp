#include "tests/gvrp/matheus_model_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/mip_start.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/cplex/matheus_model/subcycle_user_constraint.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>
#include <float.h>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model;

Matheus_model_tests::Matheus_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Matheus_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
//  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("new/non-consec/"));
  list<Gvrp_instance> gvrp_instances;
  for (const string& instance : instances) {
//    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    Gvrp_instance gvrp_instance = matheus_instance_reader(PROJECT_INSTANCES_PATH + string("new/non-consec/") + instance);
    gvrp_instances.push_back(gvrp_instance);
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "matheus_model_";
  openResultFile(resultsFile, solution_name);
  int i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
//    Gvrp_feasible_solution_heuristic gfsh (*gvrp_instance);
//    Gvrp_solution gvrp_solution = gfsh.run();
//    Mip_start matheus_model (*gvrp_instance, execution_time, gvrp_solution);  
    Matheus_model matheus_model (*gvrp_instance, execution_time);  



    //md preprocessings
    list<list<int>> edgesMD;
    list<pair<int, int>> edges_;
    edges_ = get_invalid_edges_3 (*gvrp_instance, *matheus_model.gvrp_afs_tree);
    for (const pair<int, int>& edge : edges_)
      edgesMD.push_back({matheus_model.customersC0Indexes[edge.first], matheus_model.customersC0Indexes[edge.second]});
    for (size_t i = 0; i < matheus_model.c0.size(); ++i) {
      const Vertex * vertexI = matheus_model.c0[i];
      for (size_t j = 0; j < matheus_model.c0.size(); ++j) {
        const Vertex * vertexJ = matheus_model.c0[j];
        for (size_t f_ = 0; f_ < matheus_model.f0.size(); ++f_) {
          bool valid = false;
          for (size_t f = 0; f < matheus_model.f0.size(); ++f) {
            const Vertex * vertexF = matheus_model.f0[f];
            for (size_t r = 0; r < matheus_model.f0.size(); ++r) {
              const Vertex * vertexR = matheus_model.f0[r];
              if (matheus_model.afsToCustomerFuel(f, i) + matheus_model.customerToAfsFuel(i, f_) <= matheus_model.instance.vehicleFuelCapacity 
                  && matheus_model.afsToCustomerFuel(f_, j) + matheus_model.customerToAfsFuel(j, r) <= matheus_model.instance.vehicleFuelCapacity                      && matheus_model.gvrp_afs_tree->times[f] + matheus_model.instance.time(vertexF->id, vertexI->id) + matheus_model.time(i, f_, j) + vertexJ->serviceTime + matheus_model.instance.time(vertexJ->id, vertexR->id) + matheus_model.gvrp_afs_tree->times[r] <= matheus_model.instance.timeLimit) { 
                valid = true;
                f = matheus_model.f0.size();
                break;
              }
            }
          }
          if (!valid) 
            edgesMD.push_back({i, f_, j});
        }
      }
    }
    //lh reprocessings
    list<list<int>> edgesLH;
    for (const Vertex& i : matheus_model.instance.customers) 
      for (const Vertex& j : matheus_model.instance.customers) {
        //check if edge (i, j) can be feasible
        double minFuelI = DBL_MAX, 
               minFuelJ = DBL_MAX;
        for (const Vertex& f : matheus_model.instance.afss) {
          minFuelI = min(minFuelI, matheus_model.instance.fuel(f.id, i.id));
          minFuelJ = min(minFuelJ, matheus_model.instance.fuel(j.id, f.id));
        }
        minFuelI = min(minFuelI, matheus_model.instance.fuel(matheus_model.instance.depot.id, i.id));
        minFuelJ = min(minFuelJ, matheus_model.instance.fuel(j.id, matheus_model.instance.depot.id));
        if (minFuelI + matheus_model.instance.fuel(i.id, j.id) + minFuelJ > matheus_model.instance.vehicleFuelCapacity || 
            matheus_model.instance.time(matheus_model.instance.depot.id, i.id) + i.serviceTime + matheus_model.instance.time(i.id, j.id) + j.serviceTime + matheus_model.instance.time(j.id, matheus_model.instance.depot.id) > matheus_model.instance.timeLimit) 
          edgesLH.push_back({matheus_model.customersC0Indexes[i.id], matheus_model.customersC0Indexes[j.id]});
      }
    double minFuelI, minFuelJ;
    for (const Vertex& i : gvrp_instance->customers) {
      //min fuel I
      minFuelI = DBL_MAX;
      for (const Vertex& f : gvrp_instance->afss) 
        minFuelI = min(minFuelI, gvrp_instance->fuel(f.id, i.id));
      minFuelI = min(minFuelI, gvrp_instance->fuel(gvrp_instance->depot.id, i.id));
      for (const Vertex& j : gvrp_instance->customers) {
        //min fuel J
        minFuelJ = DBL_MAX;
        for (const Vertex& f : gvrp_instance->afss) 
          minFuelJ = min(minFuelJ, gvrp_instance->fuel(j.id, f.id));
        minFuelJ = min(minFuelJ, gvrp_instance->fuel(j.id, gvrp_instance->depot.id));
        for (const Vertex& f : gvrp_instance->afss) 
          if (minFuelI + gvrp_instance->fuel(i.id, f.id) > gvrp_instance->vehicleFuelCapacity 
              || gvrp_instance->fuel(f.id, j.id) + minFuelJ > gvrp_instance->vehicleFuelCapacity 
              || gvrp_instance->time(gvrp_instance->depot.id, i.id) + gvrp_instance->time(i.id, f.id) + i.serviceTime + gvrp_instance->time(f.id, j.id) + f.serviceTime + gvrp_instance->time(j.id, gvrp_instance->depot.id) + j.serviceTime > gvrp_instance->timeLimit) { 
            int fIndex = matheus_model.afssF0Indexes[f.id];
            edgesLH.push_back({matheus_model.customersC0Indexes[i.id], fIndex, matheus_model.customersC0Indexes[j.id]});
          }
      }
    }
    //check
    for (list<int> preproLH : edgesLH) {
      bool valid = false;
      for (list<int> preproMD : edgesMD) 
        if (preproLH == preproMD) {
          valid = true;
          break;
        }
      if (!valid) {
        cout<<"missing ";
        for (int i : preproLH)
          cout<<i<<", ";
        cout<<endl;
      }
    }




    execute_model(matheus_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name + instance<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<";"<<matheus_model.nGreedyLP<<";"<<matheus_model.nPreprocessings1<<";"<<matheus_model.nPreprocessings2<<";"<<matheus_model.nPreprocessings3<<";"<<matheus_model.nPreprocessings4<<endl;
    ++gvrp_instance;
    ++i;
  }
  closeResultFile(resultsFile);
}

void Matheus_model_tests::execute_model(Matheus_model& matheus_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    matheus_model.max_num_feasible_integer_sol = nIntSol;
    matheus_model.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = matheus_model.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("MD/") + string(prefix_solution_files) + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void Matheus_model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("md_results.csv"));
  resultsFile<<"Instance;Solution;GAP;Cost;Time;Status;GreedyLP;preprocessing1;preprocessing2;preprocessing3;preprocessing4"<<endl;
}

void Matheus_model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
