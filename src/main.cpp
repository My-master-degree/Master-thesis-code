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
    } else if (strcmp(argv[i], "-nIntSol") == 0)
      nIntSol = stoi(argv[++i]);
  //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  ofstream resultsFile;
  resultsFile.open ("results.csv");
  resultsFile<<"Instance, GAP, Cost, Time,Status"<<endl;
  Compact_model *complete_compact_model;
  double lambda; 
  for (auto instance : instances){
    //read instance
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
/*
    //HERE BEGINS A NEW TEST INSTANCE
    cout<<fixed;
    cout<<setprecision(2);
    //  create new instance
//    int numCustomersAtDepot = 2,
        numCustomersAtAfs = 2;
    set<int> firstLevelAfss;
    set<int> usedCustomers;
    set<int> usedAfss;
    //bfs for the first level
    int curr = gvrp_instance.depot.id;
    for (auto afs: gvrp_instance.afss)
      //if afs is a valid afs
      if (gvrp_instance.distances[curr][afs.id] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity)
        firstLevelAfss.insert(afs.id);          
    //getting customers closer to depot
    int i = 0;
    for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end() && i < numCustomersAtDepot ; ++it){
      Vertex* customer = &(*it);
      if (gvrp_instance.distances[customer->id][gvrp_instance.depot.id] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity / 2.0){
        usedCustomers.insert(customer->id);
        i++;
      }
    }
    //getting customers closer to afss
    i = 0;
    for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end() && i < numCustomersAtAfs; ++it){
      Vertex* customer = &(*it);
      for (auto afs : firstLevelAfss){      
        if (gvrp_instance.distances[customer->id][afs] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity / 2.0 && gvrp_instance.distances[customer->id][gvrp_instance.depot.id] * gvrp_instance.vehicleFuelConsumptionRate > gvrp_instance.vehicleFuelCapacity / 2.0){
          usedCustomers.insert(customer->id);
          usedAfss.insert(afs);
          i++;
          break;
        }
      }
    }
    //checking
    if (int(usedCustomers.size()) < numCustomersAtDepot + numCustomersAtAfs)
      continue;
    //removing customers
    for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end();){
      if (!usedCustomers.count(it->id))
        it = gvrp_instance.customers.erase(it);
      else
        it++;
    }
    //removing afss 
    for (auto it = gvrp_instance.afss.begin(); it != gvrp_instance.afss.end();){
      if (!usedAfss.count(it->id))
        it = gvrp_instance.afss.erase(it);
      else
        it++;
    }
    //print distances
//    set<int> all = usedCustomers;
//   all.insert(usedAfss.begin(), usedAfss.end());
//   all.insert(gvrp_instance.depot.id);
//   for (auto id : all)
//     cout<<"    "<<id<<" ";
//   cout<<endl;
//   for (auto id : all){
//     cout<<id;
//     for (auto id1 : all){
//       double fuel = gvrp_instance.distances[id][id1] * gvrp_instance.vehicleFuelConsumptionRate;
///        fuel = int(fuel * 100.0)/100.0;
//       cout<<" "<<(fuel < 10 ? "0" : "")<<fuel;
//     }
//     cout<<endl;
//   }
    //update distance matrix
    for (int j = int(gvrp_instance.distances.size()) - 1; j >= 0; j--)
      if (j != gvrp_instance.depot.id && !usedAfss.count(j) && !usedCustomers.count(j))
        gvrp_instance.distances.erase(gvrp_instance.distances.begin() + j);    
      else
        for (int k = int(gvrp_instance.distances[j].size()) - 1; k >= 0; k--)
          if (k != gvrp_instance.depot.id && !usedAfss.count(k) && !usedCustomers.count(k))
            gvrp_instance.distances[j].erase(gvrp_instance.distances[j].begin() + k);
    //print distances
//    for (int i = 0; i < int(gvrp_instance.distances.size()); i++)
//      cout<<"    "<<i<<" ";
//    cout<<endl;
    for (int i = 0; i < int(gvrp_instance.distances.size()); i++){
//      cout<<i;
      for (int j = 0; j < int(gvrp_instance.distances[i].size()); j++){
//        double fuel = gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate;
//        fuel = int(fuel * 100.0)/100.0;
//        cout<<" "<<(fuel < 10 ? "0" : "")<<fuel;
      }
//      cout<<endl;
    }
    //print times
//    for (int i = 0; i < int(gvrp_instance.distances.size()); i++)
//      cout<<"    "<<i<<" ";
//    cout<<endl;
    for (int i = 0; i < int(gvrp_instance.distances.size()); i++){
//      cout<<i;
      for (int j = 0; j < int(gvrp_instance.distances[i].size()); j++){
//        double time = gvrp_instance.distances[i][j] / gvrp_instance.vehicleAverageSpeed;
//        fuel = int(fuel * 100.0)/100.0;
//        cout<<" "<<(time < 10 ? "0" : "")<<time;
      }
//      cout<<endl;
    }
    //update ids
    int id = 1;
    for (auto it = gvrp_instance.afss.begin(); it != gvrp_instance.afss.end(); it++){
      it->id = id++; 
    }
    for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end(); it++){
      it->id = id++; 
    }
    //HERE ENDS A NEW TEST INSTANCE
*/
    //settings
    lambda = calculateGvrpInstanceLambdaFactor (gvrp_instance);
    Preprocessing_compact_model* preprocessings[] = {new Invalid_edge_preprocessing(compact_model)};
    Extra_constraint_compact_model* extra_constraints[] = {
      new Max_distance_route_constraint(compact_model),
      new Max_afs_visit_constraint(compact_model),
      new Min_distance_route_constraint(compact_model, lambda),
      new Energy_lb_constraint(compact_model),
      new Energy_ub_constraint(compact_model),
      new Energy_lifting_constraint(compact_model),
      new Routes_order_constraint(compact_model)
    };
    //user constraints
    User_constraint_compact_model* user_constraints[] = {new Subcycle_user_constraint_compact_model(compact_model)};
    for (const User_constraint_compact_model* user_constraint : user_constraints) {
      Compact_model compact_model = Compact_model (gvrp_instance, execution_time);  
      compact_model.user_constraints.push_back(user_constraint);
    }
    //extra constraints
    //user cuts
//    compact_model.user_constraints.push_back();
    //preprocessings
//   compact_model.preprocessings.push_back(new Invalid_edge_preprocessing(compact_model));
//   //extra constraints
//   compact_model.extra_constraints.push_back(new Max_distance_route_constraint(compact_model));
//   compact_model.extra_constraints.push_back(new Max_afs_visit_constraint(compact_model));
//   compact_model.extra_constraints.push_back(new Min_distance_route_constraint(compact_model, lambda));
//   compact_model.extra_constraints.push_back(new Energy_lb_constraint(compact_model));
//   compact_model.extra_constraints.push_back(new Energy_ub_constraint(compact_model));
//   compact_model.extra_constraints.push_back(new Energy_lifting_constraint(compact_model));
//   compact_model.extra_constraints.push_back(new Routes_order_constraint(compact_model));
    //custom settings
    /*
    if (nIntSol > 0)
      compact_model.max_num_feasible_integer_sol = nIntSol;
    compact_model.VERBOSE = VERBOSE;
    //run model
    Mip_solution_info mipSolInfo;
    try{
      pair<Gvrp_solution, Mip_solution_info > sol = compact_model.run();    
      sol.first.write_in_file(PROJECT_INSTANCES_PATH + string("solutions/EMH/") + instance);
      mipSolInfo = sol.second;
    } catch (string s){
      cout<<"Error here:"<<s;
    } catch (Mip_solution_info excSolInfo){
      mipSolInfo = excSolInfo;
    }
   cout<<mipSolInfo;
    resultsFile<<instance<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    */
  }
  //write csv
  resultsFile.close(); 
  return 0;
}
