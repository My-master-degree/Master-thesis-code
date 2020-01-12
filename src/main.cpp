#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/util.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/improved_compact_model.hpp"
#include "utils/cplex/subcycle_user_constraint_compact_model.hpp"
#include "utils/cplex/invalid_edge_preprocessing.hpp"
#include "utils/cplex/max_afs_visit_constraint.hpp"
#include "utils/cplex/max_distance_route_constraint.hpp"
#include "SampleConfig.h"

#include <string>
#include <set>
#include <queue>
#include <map>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;

int main (int argc, char **argv)
{ 
  unsigned int execution_time = 120;
  bool VERBOSE = false;  
  //getting params
  for (int i = 0; i < argc; i++)
    if (strcmp(argv[i], "-time") == 0)
      execution_time = stoi(argv[++i]);
    else if (strcmp(argv[i], "-verbose") == 0)
      if (strcmp(argv[++i], "true") == 0 ||  strcmp(argv[i], "1") == 0)
        VERBOSE = true; 
  //instance list
  string instances[] = {
    "S1_20c3sU1.txt",
    "S1_20c3sU2.txt",
    "S1_20c3sU3.txt",
    "S1_20c3sU4.txt",
    "S1_20c3sU5.txt",
    "S1_20c3sU6.txt",
    "S1_20c3sU7.txt",
    "S1_20c3sU8.txt",
    "S1_20c3sU9.txt",
    "S1_20c3sU10.txt",
    "S2_20c3sC1.txt",
    "S2_20c3sC2.txt",
    "S2_20c3sC3.txt",
    "S2_20c3sC4.txt",
    "S2_20c3sC5.txt",
    "S2_20c3sC6.txt",
    "S2_20c3sC7.txt",
    "S2_20c3sC8.txt",
    "S2_20c3sC9.txt",
    "S2_20c3sC10.txt",
    "S3_S1_2i6s.txt",
    "S3_S1_4i6s.txt",
    "S3_S1_6i6s.txt",
    "S3_S1_8i6s.txt",
    "S3_S2_10i6s.txt", 
    "S3_S2_2i6s.txt",
    "S3_S2_4i6s.txt",
    "S3_S2_6i6s.txt",
    "S3_S2_8i6s.txt",
    "S3_S1_10i6s.txt",
    "S4_S1_4i2s.txt",
    "S4_S1_4i4s.txt",
    "S4_S1_4i6s.txt",
    "S4_S1_4i8s.txt",
    "S4_S1_4i10s.txt",
    "S4_S2_4i2s.txt",
    "S4_S2_4i4s.txt",
    "S4_S2_4i6s.txt",
    "S4_S2_4i8s.txt",
    "S4_S2_4i10s.txt",
    "Large_VA_Input_111c_21s.txt",
    "Large_VA_Input_111c_22s.txt",
    "Large_VA_Input_111c_24s.txt",
    "Large_VA_Input_111c_26s.txt",
    "Large_VA_Input_111c_28s.txt",
    "Large_VA_Input_200c_21s.txt",
    "Large_VA_Input_300c_21s.txt",
    "Large_VA_Input_350c_21s.txt",
    "Large_VA_Input_400c_21s.txt",
    "Large_VA_Input_450c_21s.txt",
    "Large_VA_Input_500c_21s.txt",
  };

  ofstream resultsFile;
  resultsFile.open ("results.csv");
  resultsFile<<"Instance, GAP, Cost, Time,Status"<<endl;
  for (auto instance : instances){
    //read instance
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_PATH + string("instances/") + instance);
//*/
    //HERE BEGINS A NEW TEST INSTANCE
    //  create new instance
    int numCustomersAtDepot = 2,
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
    set<int> all = usedCustomers;
    all.insert(usedAfss.begin(), usedAfss.end());
    all.insert(gvrp_instance.depot.id);
    for (auto id : all)
      cout<<"  "<<id<<" ";
    cout<<endl;
    for (auto id : all){
      cout<<id;
      for (auto id1 : all){
        double fuel = gvrp_instance.distances[id][id1] * gvrp_instance.vehicleFuelConsumptionRate;
        fuel = int(fuel * 100.0)/100.0;
        cout<<" "<<fuel;
      }
      cout<<endl;
    }
    //update distance matrix
    for (int j = gvrp_instance.distances.size() - 1; j >= 0; j--)
      if (j != gvrp_instance.depot.id && !usedAfss.count(j) && !usedCustomers.count(j))
        gvrp_instance.distances.erase(gvrp_instance.distances.begin() + j);    
      else
        for (int k = gvrp_instance.distances[j].size() - 1; k >= 0; k--)
          if (k != gvrp_instance.depot.id && !usedAfss.count(k) && !usedCustomers.count(k))
            gvrp_instance.distances[j].erase(gvrp_instance.distances[j].begin() + k);
    //print distances
    for (int i = 0; i < gvrp_instance.distances.size(); i++)
      cout<<"  "<<i<<" ";
    cout<<endl;
    for (int i = 0; i < gvrp_instance.distances.size(); i++){
      cout<<i;
      for (int j = 0; j < gvrp_instance.distances[i].size(); j++){
        double fuel = gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate;
        fuel = int(fuel * 100.0)/100.0;
        cout<<" "<<fuel;
      }
      cout<<endl;
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
//*/
    cout<<instance<<endl;
    cout<<gvrp_instance<<endl;
    //settings
    Compact_model compact_model(gvrp_instance, execution_time);
//    Improved_compact_model compact_model(gvrp_instance, execution_time);    
    //custom settings
      //user cuts
    compact_model.user_constraints.push_back(new Subcycle_user_constraint_compact_model(compact_model));
      //preprocessings
    compact_model.preprocessings.push_back(new Invalid_edge_preprocessing(compact_model));
      //extra constraints
    compact_model.extra_constraints.push_back(new Max_distance_route_constraint(compact_model));
    compact_model.extra_constraints.push_back(new Max_afs_visit_constraint(compact_model));
    //custom settings
    compact_model.max_num_feasible_integer_sol = 1;
    compact_model.VERBOSE = VERBOSE;
    //run model
    Mip_solution_info mipSolInfo;
    try{
      pair<Gvrp_solution, Mip_solution_info > sol = compact_model.run();    
      sol.first.write_in_file(PROJECT_PATH + string("solutions/") + instance + string(".sol"));
      mipSolInfo = sol.second;
    } catch (string s){
      cout<<"Error here:"<<s;
    } catch (Mip_solution_info excSolInfo){
      mipSolInfo = excSolInfo;
    }
      cout<<mipSolInfo;
    resultsFile<<instance<<","<<mipSolInfo.gap<<","<<mipSolInfo.cost<<","<<mipSolInfo.elapsed_time<<","<<mipSolInfo.status<<endl;
    break;
  }
  //write csv
  resultsFile.close(); 
  return 0;
}
